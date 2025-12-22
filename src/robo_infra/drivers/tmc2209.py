"""TMC2209 stepper motor driver.

This module provides a driver for the Trinamic TMC2209 stepper motor driver,
which supports UART configuration, StealthChop silent operation, and
StallGuard sensorless homing.

The TMC2209 is commonly used in 3D printers, CNC machines, and robotics for
precise, silent stepper motor control.

Example:
    >>> from robo_infra.core.bus import get_serial
    >>> from robo_infra.drivers.tmc2209 import TMC2209Driver
    >>>
    >>> # Create driver with UART connection
    >>> uart = get_serial("/dev/ttyUSB0", 115200)
    >>> driver = TMC2209Driver(uart=uart, address=0)
    >>> driver.connect()
    >>>
    >>> # Configure motor current
    >>> driver.set_current(run_current=1.2, hold_current=0.6)
    >>>
    >>> # Enable StealthChop for silent operation
    >>> driver.enable_stealthchop()
    >>>
    >>> # Set microstepping
    >>> driver.set_microstepping(16)
    >>>
    >>> # Read StallGuard value for sensorless homing
    >>> sg_value = driver.get_stallguard()
    >>> print(f"StallGuard: {sg_value}")
    >>>
    >>> driver.disconnect()

Hardware Reference:
    - Supply voltage: 4.75V to 29V
    - Motor current: up to 2.8A RMS (with heatsink)
    - Microstepping: 1 to 256 (interpolated to 256)
    - UART baud rate: 9600 to 500000 (default 115200)
    - Single-wire UART (TX and RX on same pin)
"""

from __future__ import annotations

import logging
import os
import struct
import time
from dataclasses import dataclass
from enum import IntEnum, IntFlag
from typing import TYPE_CHECKING

from robo_infra.core.driver import (
    Driver,
    DriverConfig,
    DriverState,
    register_driver,
)
from robo_infra.core.exceptions import CommunicationError


if TYPE_CHECKING:
    from robo_infra.core.bus import SerialBus


logger = logging.getLogger(__name__)


# =============================================================================
# TMC2209 Register Definitions
# =============================================================================


class TMC2209Register(IntEnum):
    """TMC2209 register addresses.

    The TMC2209 uses a 7-bit register address space.
    """

    # General Configuration
    GCONF = 0x00  # Global configuration
    GSTAT = 0x01  # Global status
    IFCNT = 0x02  # Interface transmission counter
    SLAVECONF = 0x03  # UART slave configuration
    OTP_PROG = 0x04  # OTP programming
    OTP_READ = 0x05  # OTP read
    IOIN = 0x06  # Input/output state
    FACTORY_CONF = 0x07  # Factory configuration

    # Velocity Dependent Control
    IHOLD_IRUN = 0x10  # Driver current control
    TPOWERDOWN = 0x11  # Power down delay
    TSTEP = 0x12  # Actual measured time between steps
    TPWMTHRS = 0x13  # StealthChop/SpreadCycle threshold
    TCOOLTHRS = 0x14  # CoolStep/StallGuard lower threshold
    VACTUAL = 0x22  # Actual velocity (VACTUAL mode)

    # StallGuard Control
    SGTHRS = 0x40  # StallGuard threshold
    SG_RESULT = 0x41  # StallGuard result
    COOLCONF = 0x42  # CoolStep configuration

    # Sequencer Registers
    MSCNT = 0x6A  # Microstep counter
    MSCURACT = 0x6B  # Actual microstep current

    # Chopper Control
    CHOPCONF = 0x6C  # Chopper configuration
    DRV_STATUS = 0x6F  # Driver status

    # PWM Configuration
    PWMCONF = 0x70  # PWM configuration


class GCONFBits(IntFlag):
    """GCONF register bits."""

    I_SCALE_ANALOG = 1 << 0  # Use VREF for current scale
    INTERNAL_RSENSE = 1 << 1  # Use internal sense resistors
    EN_SPREADCYCLE = 1 << 2  # Enable SpreadCycle
    SHAFT = 1 << 3  # Inverse motor direction
    INDEX_OTPW = 1 << 4  # INDEX output shows overtemp warning
    INDEX_STEP = 1 << 5  # INDEX output shows step pulse
    PDN_DISABLE = 1 << 6  # Disable PDN_UART input
    MSTEP_REG_SELECT = 1 << 7  # Use MSTEP register for microstepping
    MULTISTEP_FILT = 1 << 8  # Enable pulse filtering


class DRVStatusBits(IntFlag):
    """DRV_STATUS register bits."""

    OTPW = 1 << 0  # Overtemperature pre-warning
    OT = 1 << 1  # Overtemperature
    S2GA = 1 << 2  # Short to GND on phase A
    S2GB = 1 << 3  # Short to GND on phase B
    S2VSA = 1 << 4  # Short to VS on phase A
    S2VSB = 1 << 5  # Short to VS on phase B
    OLA = 1 << 6  # Open load on phase A
    OLB = 1 << 7  # Open load on phase B
    T120 = 1 << 8  # Temperature > 120°C
    T143 = 1 << 9  # Temperature > 143°C
    T150 = 1 << 10  # Temperature > 150°C
    T157 = 1 << 11  # Temperature > 157°C
    # Bits 16-24: CS_ACTUAL (actual current setting)
    STST = 1 << 31  # Standstill indicator


# =============================================================================
# Configuration
# =============================================================================


@dataclass
class TMC2209Config:
    """Configuration for TMC2209 driver.

    Attributes:
        address: UART address (0-3).
        run_current: Run current in Amps (0.1 to 2.8).
        hold_current: Hold current in Amps (0 to run_current).
        hold_delay: Delay before switching to hold current (0-15).
        microstepping: Microsteps per step (1, 2, 4, 8, 16, 32, 64, 128, 256).
        interpolation: Enable 256 microstep interpolation.
        stealthchop: Use StealthChop for silent operation.
        stealthchop_threshold: TSTEP threshold for StealthChop.
        stallguard_threshold: StallGuard sensitivity (0-255).
        rsense: Sense resistor value in Ohms (default 0.11).
        vsense: Use high sensitivity mode (lower current range).
    """

    address: int = 0
    run_current: float = 1.0
    hold_current: float = 0.5
    hold_delay: int = 10
    microstepping: int = 16
    interpolation: bool = True
    stealthchop: bool = True
    stealthchop_threshold: int = 0
    stallguard_threshold: int = 100
    rsense: float = 0.11
    vsense: bool = False


# =============================================================================
# TMC2209 Driver
# =============================================================================


@register_driver("tmc2209")
class TMC2209Driver(Driver):
    """Driver for TMC2209 stepper motor controller.

    The TMC2209 is a high-performance stepper motor driver with:
    - UART configuration interface
    - StealthChop for silent operation
    - SpreadCycle for high dynamics
    - StallGuard for sensorless homing
    - CoolStep for automatic current reduction
    - Up to 256 microstepping with interpolation

    Example:
        >>> driver = TMC2209Driver(uart=serial_bus, address=0)
        >>> driver.connect()
        >>> driver.set_current(1.5, 0.75)
        >>> driver.enable_stealthchop()
        >>> driver.set_microstepping(16)
    """

    def __init__(
        self,
        uart: SerialBus | None = None,
        address: int = 0,
        config: TMC2209Config | None = None,
        simulation: bool | None = None,
        name: str | None = None,
    ) -> None:
        """Initialize TMC2209 driver.

        Args:
            uart: Serial/UART bus for communication.
            address: UART address (0-3).
            config: Driver configuration.
            simulation: If True, use simulation mode. Auto-detected if None.
            name: Optional human-readable name.
        """
        if simulation is None:
            simulation = os.getenv("ROBO_SIMULATION", "").lower() in ("true", "1", "yes")

        super().__init__(
            config=DriverConfig(
                name=name or f"TMC2209@{address}",
                channels=1,
                auto_connect=False,
            )
        )

        self._uart = uart
        self._address = address
        self._tmc_config = config or TMC2209Config(address=address)
        self._simulation = simulation

        # Register cache
        self._register_cache: dict[int, int] = {}

        # Datagram counter for UART sync
        self._ifcnt: int = 0

    @property
    def address(self) -> int:
        """UART address (0-3)."""
        return self._address

    @property
    def simulation(self) -> bool:
        """Whether running in simulation mode."""
        return self._simulation

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------

    def connect(self) -> None:
        """Connect to the TMC2209 and initialize settings.

        Raises:
            CommunicationError: If connection fails.
        """
        if self._simulation:
            logger.info("TMC2209 connecting in simulation mode")
            self._state = DriverState.CONNECTED
            return

        if self._uart is None:
            raise CommunicationError("UART bus not provided")

        if not self._uart.is_open:
            self._uart.open()

        # Read IFCNT to verify communication
        try:
            self._ifcnt = self._read_register(TMC2209Register.IFCNT)
            logger.debug("TMC2209 IFCNT: %d", self._ifcnt)
        except Exception as e:
            raise CommunicationError(f"Failed to connect to TMC2209: {e}") from e

        # Apply initial configuration
        self._apply_config()

        self._state = DriverState.CONNECTED
        logger.info("TMC2209 connected at address %d", self._address)

    def disconnect(self) -> None:
        """Disconnect from the TMC2209."""
        self._state = DriverState.DISCONNECTED
        logger.info("TMC2209 disconnected")

    def _apply_config(self) -> None:
        """Apply configuration to the driver."""
        cfg = self._tmc_config

        # Configure GCONF
        gconf = GCONFBits.PDN_DISABLE | GCONFBits.MSTEP_REG_SELECT
        if not cfg.stealthchop:
            gconf |= GCONFBits.EN_SPREADCYCLE
        self._write_register(TMC2209Register.GCONF, gconf)

        # Set current
        self.set_current(cfg.run_current, cfg.hold_current, cfg.hold_delay)

        # Set microstepping
        self.set_microstepping(cfg.microstepping)

        # Set StealthChop threshold
        self._write_register(TMC2209Register.TPWMTHRS, cfg.stealthchop_threshold)

        # Set StallGuard threshold
        self._write_register(TMC2209Register.SGTHRS, cfg.stallguard_threshold)

    # -------------------------------------------------------------------------
    # Current Control
    # -------------------------------------------------------------------------

    def set_current(
        self,
        run_current: float,
        hold_current: float | None = None,
        hold_delay: int = 10,
    ) -> None:
        """Set the motor current.

        Args:
            run_current: Run current in Amps (0.1 to 2.8).
            hold_current: Hold current in Amps. Defaults to 50% of run current.
            hold_delay: Delay before switching to hold current (0-15).

        Raises:
            ValueError: If current is out of range.
        """
        if not 0.1 <= run_current <= 2.8:
            raise ValueError(f"Run current {run_current}A out of range (0.1-2.8A)")

        if hold_current is None:
            hold_current = run_current * 0.5

        if not 0 <= hold_current <= run_current:
            raise ValueError(f"Hold current {hold_current}A must be 0 to {run_current}A")

        if not 0 <= hold_delay <= 15:
            raise ValueError(f"Hold delay {hold_delay} must be 0-15")

        # Calculate IRUN and IHOLD values (0-31)
        # Current = (IRUN + 1) / 32 * V_ref / R_sense * 1.41
        # For VSENSE=0: V_ref = 0.325V, For VSENSE=1: V_ref = 0.180V
        vsense = self._tmc_config.vsense
        v_ref = 0.180 if vsense else 0.325
        rsense = self._tmc_config.rsense

        # I = (CS + 1) / 32 * V_ref / R_sense * sqrt(2)
        # CS = (I * 32 * R_sense) / (V_ref * sqrt(2)) - 1
        irun = int((run_current * 32 * rsense) / (v_ref * 1.414) - 1)
        ihold = int((hold_current * 32 * rsense) / (v_ref * 1.414) - 1)

        irun = max(0, min(31, irun))
        ihold = max(0, min(31, ihold))

        # IHOLD_IRUN register: [IHOLDDELAY: 19-16] [IRUN: 12-8] [IHOLD: 4-0]
        value = (hold_delay << 16) | (irun << 8) | ihold

        if self._simulation:
            logger.info(
                "TMC2209 set current: run=%.2fA (IRUN=%d), hold=%.2fA (IHOLD=%d)",
                run_current,
                irun,
                hold_current,
                ihold,
            )
            return

        self._write_register(TMC2209Register.IHOLD_IRUN, value)
        logger.debug("Set IHOLD_IRUN=0x%08X", value)

    def get_current(self) -> tuple[float, float]:
        """Get the current settings.

        Returns:
            Tuple of (run_current, hold_current) in Amps.
        """
        if self._simulation:
            return (self._tmc_config.run_current, self._tmc_config.hold_current)

        value = self._read_register(TMC2209Register.IHOLD_IRUN)
        irun = (value >> 8) & 0x1F
        ihold = value & 0x1F

        vsense = self._tmc_config.vsense
        v_ref = 0.180 if vsense else 0.325
        rsense = self._tmc_config.rsense

        run_current = (irun + 1) / 32 * v_ref / rsense * 1.414
        hold_current = (ihold + 1) / 32 * v_ref / rsense * 1.414

        return (run_current, hold_current)

    # -------------------------------------------------------------------------
    # Microstepping
    # -------------------------------------------------------------------------

    def set_microstepping(self, microsteps: int) -> None:
        """Set the microstepping resolution.

        Args:
            microsteps: Microsteps per full step (1, 2, 4, 8, 16, 32, 64, 128, 256).

        Raises:
            ValueError: If microsteps is not valid.
        """
        valid = [1, 2, 4, 8, 16, 32, 64, 128, 256]
        if microsteps not in valid:
            raise ValueError(f"Microsteps {microsteps} not valid. Use: {valid}")

        # MRES encoding: 256=0, 128=1, 64=2, 32=3, 16=4, 8=5, 4=6, 2=7, 1=8
        mres_map = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}
        mres = mres_map[microsteps]

        if self._simulation:
            logger.info("TMC2209 set microstepping: %d (MRES=%d)", microsteps, mres)
            return

        # Read current CHOPCONF
        chopconf = self._read_register(TMC2209Register.CHOPCONF)

        # Clear MRES bits [27:24] and set new value
        chopconf = (chopconf & ~(0x0F << 24)) | (mres << 24)

        # Enable interpolation if configured
        if self._tmc_config.interpolation:
            chopconf |= 1 << 28  # INTPOL bit
        else:
            chopconf &= ~(1 << 28)

        self._write_register(TMC2209Register.CHOPCONF, chopconf)
        logger.debug("Set CHOPCONF=0x%08X", chopconf)

    def get_microstepping(self) -> int:
        """Get the current microstepping setting.

        Returns:
            Microsteps per full step.
        """
        if self._simulation:
            return self._tmc_config.microstepping

        chopconf = self._read_register(TMC2209Register.CHOPCONF)
        mres = (chopconf >> 24) & 0x0F

        mres_to_microsteps = {0: 256, 1: 128, 2: 64, 3: 32, 4: 16, 5: 8, 6: 4, 7: 2, 8: 1}
        return mres_to_microsteps.get(mres, 256)

    # -------------------------------------------------------------------------
    # Operating Modes
    # -------------------------------------------------------------------------

    def enable_stealthchop(self) -> None:
        """Enable StealthChop for silent operation.

        StealthChop is a voltage-mode chopper that provides extremely quiet
        motor operation at the expense of reduced torque at high speeds.
        """
        if self._simulation:
            logger.info("TMC2209 enabling StealthChop")
            return

        gconf = self._read_register(TMC2209Register.GCONF)
        gconf &= ~GCONFBits.EN_SPREADCYCLE  # Disable SpreadCycle = enable StealthChop
        self._write_register(TMC2209Register.GCONF, gconf)
        logger.debug("StealthChop enabled")

    def enable_spreadcycle(self) -> None:
        """Enable SpreadCycle for high dynamics.

        SpreadCycle is a classic constant-off-time chopper that provides
        higher torque and better performance at high speeds.
        """
        if self._simulation:
            logger.info("TMC2209 enabling SpreadCycle")
            return

        gconf = self._read_register(TMC2209Register.GCONF)
        gconf |= GCONFBits.EN_SPREADCYCLE
        self._write_register(TMC2209Register.GCONF, gconf)
        logger.debug("SpreadCycle enabled")

    def is_stealthchop(self) -> bool:
        """Check if StealthChop is enabled.

        Returns:
            True if StealthChop is enabled.
        """
        if self._simulation:
            return self._tmc_config.stealthchop

        gconf = self._read_register(TMC2209Register.GCONF)
        return not bool(gconf & GCONFBits.EN_SPREADCYCLE)

    # -------------------------------------------------------------------------
    # StallGuard (Sensorless Homing)
    # -------------------------------------------------------------------------

    def set_stallguard_threshold(self, threshold: int) -> None:
        """Set the StallGuard threshold.

        Args:
            threshold: StallGuard sensitivity (0-255). Lower = more sensitive.

        Raises:
            ValueError: If threshold is out of range.
        """
        if not 0 <= threshold <= 255:
            raise ValueError(f"Threshold {threshold} out of range (0-255)")

        if self._simulation:
            logger.info("TMC2209 set StallGuard threshold: %d", threshold)
            return

        self._write_register(TMC2209Register.SGTHRS, threshold)
        logger.debug("Set SGTHRS=%d", threshold)

    def get_stallguard(self) -> int:
        """Read the StallGuard result.

        Returns:
            StallGuard value (0-510). Lower values indicate stall condition.
        """
        if self._simulation:
            return 250  # Simulated mid-range value

        return self._read_register(TMC2209Register.SG_RESULT) & 0x3FF

    def is_stalled(self, threshold: int | None = None) -> bool:
        """Check if the motor is stalled.

        Args:
            threshold: Optional threshold. Uses configured value if not specified.

        Returns:
            True if StallGuard value is below threshold.
        """
        if threshold is None:
            threshold = self._tmc_config.stallguard_threshold

        sg_value = self.get_stallguard()
        return sg_value < threshold

    # -------------------------------------------------------------------------
    # Status
    # -------------------------------------------------------------------------

    def get_status(self) -> dict:
        """Get the driver status.

        Returns:
            Dictionary with status information.
        """
        if self._simulation:
            return {
                "connected": True,
                "overtemp_warning": False,
                "overtemp": False,
                "short_a": False,
                "short_b": False,
                "open_a": False,
                "open_b": False,
                "standstill": True,
                "stallguard": 250,
                "actual_current": 16,
            }

        drv_status = self._read_register(TMC2209Register.DRV_STATUS)
        sg_result = self.get_stallguard()

        return {
            "connected": self._state == DriverState.CONNECTED,
            "overtemp_warning": bool(drv_status & DRVStatusBits.OTPW),
            "overtemp": bool(drv_status & DRVStatusBits.OT),
            "short_a": bool(drv_status & (DRVStatusBits.S2GA | DRVStatusBits.S2VSA)),
            "short_b": bool(drv_status & (DRVStatusBits.S2GB | DRVStatusBits.S2VSB)),
            "open_a": bool(drv_status & DRVStatusBits.OLA),
            "open_b": bool(drv_status & DRVStatusBits.OLB),
            "standstill": bool(drv_status & DRVStatusBits.STST),
            "stallguard": sg_result,
            "actual_current": (drv_status >> 16) & 0x1F,
        }

    def get_temperature_flags(self) -> dict:
        """Get temperature warning flags.

        Returns:
            Dictionary with temperature flags.
        """
        if self._simulation:
            return {"over_120c": False, "over_143c": False, "over_150c": False, "over_157c": False}

        drv_status = self._read_register(TMC2209Register.DRV_STATUS)

        return {
            "over_120c": bool(drv_status & DRVStatusBits.T120),
            "over_143c": bool(drv_status & DRVStatusBits.T143),
            "over_150c": bool(drv_status & DRVStatusBits.T150),
            "over_157c": bool(drv_status & DRVStatusBits.T157),
        }

    # -------------------------------------------------------------------------
    # Motor Direction
    # -------------------------------------------------------------------------

    def set_direction(self, inverted: bool) -> None:
        """Set the motor direction.

        Args:
            inverted: If True, invert the motor direction.
        """
        if self._simulation:
            logger.info("TMC2209 set direction inverted: %s", inverted)
            return

        gconf = self._read_register(TMC2209Register.GCONF)
        if inverted:
            gconf |= GCONFBits.SHAFT
        else:
            gconf &= ~GCONFBits.SHAFT
        self._write_register(TMC2209Register.GCONF, gconf)

    def get_direction(self) -> bool:
        """Get the motor direction setting.

        Returns:
            True if direction is inverted.
        """
        if self._simulation:
            return False

        gconf = self._read_register(TMC2209Register.GCONF)
        return bool(gconf & GCONFBits.SHAFT)

    # -------------------------------------------------------------------------
    # Low-Level Register Access
    # -------------------------------------------------------------------------

    def _calc_crc(self, data: bytes) -> int:
        """Calculate CRC8 for TMC2209 UART.

        Args:
            data: Data bytes.

        Returns:
            CRC8 value.
        """
        crc = 0
        for byte in data:
            for _ in range(8):
                crc = (crc << 1 ^ 7) & 255 if crc >> 7 ^ byte & 1 else crc << 1 & 255
                byte >>= 1
        return crc

    def _read_register(self, register: int) -> int:
        """Read a register from the TMC2209.

        Args:
            register: Register address.

        Returns:
            32-bit register value.

        Raises:
            CommunicationError: If read fails.
        """
        if self._simulation:
            return self._register_cache.get(register, 0)

        if self._uart is None:
            raise CommunicationError("UART bus not available")

        # Build read request: SYNC + ADDR + REG + CRC
        sync = 0x05  # Read request
        addr = self._address
        datagram = bytes([sync, addr, register])
        crc = self._calc_crc(datagram)
        datagram += bytes([crc])

        # Send request
        self._uart.reset_input_buffer()
        self._uart.write(datagram)

        # Wait for echo and response
        time.sleep(0.001)

        # Read 8 bytes (4 echo + 4 response) or 12 bytes if single-wire
        response = self._uart.read(12)

        if len(response) < 8:
            raise CommunicationError(f"TMC2209 read failed: only {len(response)} bytes received")

        # Parse response (skip echo)
        # Response format: SYNC + ADDR + REG + DATA[3:0] + CRC
        offset = 4 if len(response) >= 12 else 0
        data = response[offset + 3 : offset + 7]

        if len(data) < 4:
            raise CommunicationError("TMC2209 read failed: incomplete response")

        value = struct.unpack(">I", data)[0]
        self._register_cache[register] = value

        return value

    def _write_register(self, register: int, value: int) -> None:
        """Write a register to the TMC2209.

        Args:
            register: Register address.
            value: 32-bit value to write.

        Raises:
            CommunicationError: If write fails.
        """
        self._register_cache[register] = value

        if self._simulation:
            return

        if self._uart is None:
            raise CommunicationError("UART bus not available")

        # Build write request: SYNC + ADDR + REG|0x80 + DATA[3:0] + CRC
        sync = 0x05
        addr = self._address
        reg_write = register | 0x80  # Set write bit

        data = struct.pack(">I", value)
        datagram = bytes([sync, addr, reg_write]) + data
        crc = self._calc_crc(datagram)
        datagram += bytes([crc])

        # Send write
        self._uart.write(datagram)

        # Wait for transmission
        time.sleep(0.001)

        # Verify by reading IFCNT (should increment)
        new_ifcnt = self._read_register(TMC2209Register.IFCNT)
        if new_ifcnt == self._ifcnt:
            logger.warning("TMC2209 IFCNT did not increment - write may have failed")
        self._ifcnt = new_ifcnt

    # -------------------------------------------------------------------------
    # Driver Abstract Methods
    # -------------------------------------------------------------------------

    def _write_channel(self, channel: int, value: float) -> None:
        """Write to a channel (not used for TMC2209)."""
        pass

    def _read_channel(self, channel: int) -> float:
        """Read from a channel (returns StallGuard value normalized)."""
        return self.get_stallguard() / 510.0
