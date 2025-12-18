"""PCA9685 16-channel PWM driver.

This module provides a driver for the PCA9685 16-channel 12-bit PWM/servo driver,
commonly used for controlling servos, LEDs, and other PWM devices.

The PCA9685 uses I2C for communication and provides:
- 16 individually controllable PWM channels
- 12-bit resolution (4096 steps)
- Configurable frequency (24Hz to 1526Hz)
- Auto-increment register addressing
- Sleep mode for power saving
- All-call and sub-address support

Example:
    >>> from robo_infra.core.bus import get_i2c
    >>> from robo_infra.drivers.pca9685 import PCA9685
    >>>
    >>> # Create driver with I2C bus
    >>> bus = get_i2c(1)
    >>> driver = PCA9685(bus=bus, address=0x40)
    >>> driver.connect()
    >>>
    >>> # Set PWM frequency for servos (50Hz)
    >>> driver.set_frequency(50)
    >>>
    >>> # Set channel 0 to 50% duty cycle
    >>> driver.set_channel(0, 0.5)
    >>>
    >>> # Raw PWM control (12-bit values)
    >>> driver.set_pwm(1, on=0, off=2048)  # 50% duty
    >>>
    >>> # Control all channels at once
    >>> driver.set_all_channels(0.0)  # All off
    >>>
    >>> driver.disconnect()

Hardware Reference:
    - Default I2C address: 0x40 (configurable via A0-A5 pins)
    - I2C speed: Up to 1MHz (Fast-mode Plus)
    - Supply voltage: 2.3V to 5.5V
    - PWM outputs: 16 channels, 12-bit resolution
    - Internal 25MHz oscillator
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING

from robo_infra.core.driver import (
    Driver,
    DriverConfig,
    DriverState,
    register_driver,
)
from robo_infra.core.exceptions import CommunicationError, HardwareNotFoundError


if TYPE_CHECKING:
    from robo_infra.core.bus import I2CBus


logger = logging.getLogger(__name__)


# =============================================================================
# PCA9685 Register Definitions
# =============================================================================


class PCA9685Register(IntEnum):
    """PCA9685 register addresses.

    The PCA9685 has the following register layout:
    - Control registers: 0x00-0x01
    - LED output registers: 0x06-0x45 (4 bytes per LED)
    - ALL_LED registers: 0xFA-0xFD
    - Prescaler: 0xFE
    """

    # Control registers
    MODE1 = 0x00  # Mode register 1
    MODE2 = 0x01  # Mode register 2

    # Sub-address registers (for multi-device control)
    SUBADR1 = 0x02  # I2C sub-address 1
    SUBADR2 = 0x03  # I2C sub-address 2
    SUBADR3 = 0x04  # I2C sub-address 3
    ALLCALLADR = 0x05  # LED All Call I2C address

    # LED output registers (channels 0-15)
    # Each channel has 4 registers: ON_L, ON_H, OFF_L, OFF_H
    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09
    # ... LED1-LED14 follow the same pattern
    LED15_OFF_H = 0x45

    # All LED registers (control all channels at once)
    ALL_LED_ON_L = 0xFA
    ALL_LED_ON_H = 0xFB
    ALL_LED_OFF_L = 0xFC
    ALL_LED_OFF_H = 0xFD

    # Prescaler register
    PRE_SCALE = 0xFE  # PWM frequency prescaler


class PCA9685Mode1(IntEnum):
    """MODE1 register bit definitions."""

    ALLCALL = 0x01  # Respond to LED All Call address
    SUB3 = 0x02  # Respond to sub-address 3
    SUB2 = 0x04  # Respond to sub-address 2
    SUB1 = 0x08  # Respond to sub-address 1
    SLEEP = 0x10  # Low power mode (oscillator off)
    AI = 0x20  # Auto-increment enabled
    EXTCLK = 0x40  # Use external clock
    RESTART = 0x80  # Restart PWM channels


class PCA9685Mode2(IntEnum):
    """MODE2 register bit definitions."""

    OUTNE_0 = 0x01  # Output not enable behavior bit 0
    OUTNE_1 = 0x02  # Output not enable behavior bit 1
    OUTDRV = 0x04  # Totem pole (1) or open-drain (0)
    OCH = 0x08  # Outputs change on ACK (1) or STOP (0)
    INVRT = 0x10  # Output logic inverted


# =============================================================================
# Constants
# =============================================================================

# PCA9685 timing
PCA9685_OSCILLATOR_FREQ = 25_000_000  # 25MHz internal oscillator
PCA9685_MIN_PRESCALE = 3  # Minimum prescaler value
PCA9685_MAX_PRESCALE = 255  # Maximum prescaler value

# PWM resolution
PCA9685_RESOLUTION = 4096  # 12-bit resolution (0-4095)
PCA9685_MAX_PWM = 4095  # Maximum PWM value

# Frequency limits (with 25MHz oscillator)
PCA9685_MIN_FREQUENCY = 24  # ~24Hz at prescale=255
PCA9685_MAX_FREQUENCY = 1526  # ~1526Hz at prescale=3

# Default address
PCA9685_DEFAULT_ADDRESS = 0x40

# All-call address
PCA9685_ALLCALL_ADDRESS = 0x70

# Timing
PCA9685_RESTART_DELAY = 0.0005  # 500µs for oscillator to stabilize


# =============================================================================
# Configuration
# =============================================================================


@dataclass
class PCA9685Config:
    """Configuration for PCA9685 driver.

    Attributes:
        address: I2C device address (default 0x40).
        frequency: Initial PWM frequency in Hz (default 50Hz for servos).
        auto_increment: Enable auto-increment mode for faster writes.
        invert: Invert output logic.
        totem_pole: Use totem-pole outputs (True) or open-drain (False).
        all_call: Enable response to all-call address.
        output_change_on_ack: Change outputs on ACK (True) or STOP (False).
    """

    address: int = PCA9685_DEFAULT_ADDRESS
    frequency: int = 50
    auto_increment: bool = True
    invert: bool = False
    totem_pole: bool = True
    all_call: bool = True
    output_change_on_ack: bool = False


# =============================================================================
# PCA9685 Driver
# =============================================================================


@register_driver("pca9685")
class PCA9685(Driver):
    """Driver for PCA9685 16-channel 12-bit PWM controller.

    The PCA9685 is a popular I2C PWM driver used for controlling servos,
    LEDs, and other PWM devices. It provides 16 channels with 12-bit
    resolution and configurable frequency.

    Features:
    - 16 PWM channels with independent control
    - 12-bit resolution (4096 steps per cycle)
    - Configurable frequency from 24Hz to 1526Hz
    - Auto-increment for efficient multi-channel writes
    - Sleep mode for power saving
    - All-call address for multi-device control

    Example:
        >>> from robo_infra.core.bus import get_i2c
        >>> from robo_infra.drivers.pca9685 import PCA9685
        >>>
        >>> bus = get_i2c(1)
        >>> driver = PCA9685(bus=bus)
        >>> driver.connect()
        >>>
        >>> # Set frequency for servos
        >>> driver.set_frequency(50)
        >>>
        >>> # Set channel to 50% duty cycle
        >>> driver.set_channel(0, 0.5)
        >>>
        >>> # Full brightness LED
        >>> driver.set_channel(1, 1.0)
        >>>
        >>> driver.disconnect()
    """

    def __init__(
        self,
        bus: I2CBus | None = None,
        address: int = PCA9685_DEFAULT_ADDRESS,
        config: PCA9685Config | None = None,
        driver_config: DriverConfig | None = None,
    ) -> None:
        """Initialize PCA9685 driver.

        Args:
            bus: I2C bus for communication. If None, uses simulation mode.
            address: I2C device address (default 0x40).
            config: PCA9685-specific configuration.
            driver_config: Base driver configuration.
        """
        # Use DriverConfig if provided, otherwise create one
        if driver_config is None:
            driver_config = DriverConfig(
                name=f"PCA9685@{address:#04x}",
                channels=16,
                frequency=config.frequency if config else 50,
            )

        super().__init__(
            name=driver_config.name,
            channels=driver_config.channels,
            config=driver_config,
        )

        self._bus = bus
        self._address = config.address if config else address
        self._pca_config = config or PCA9685Config(address=address)
        self._simulation_mode = bus is None

        # Cached state
        self._is_sleeping = True  # PCA9685 starts in sleep mode
        self._current_frequency = self._pca_config.frequency

        if self._simulation_mode:
            logger.info(
                "PCA9685 at %#04x running in simulation mode",
                self._address,
            )

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def address(self) -> int:
        """I2C device address."""
        return self._address

    @property
    def is_sleeping(self) -> bool:
        """Whether the device is in sleep mode."""
        return self._is_sleeping

    @property
    def simulation_mode(self) -> bool:
        """Whether running in simulation mode (no real hardware)."""
        return self._simulation_mode

    @property
    def current_frequency(self) -> int:
        """Current PWM frequency in Hz."""
        return self._current_frequency

    # -------------------------------------------------------------------------
    # Lifecycle Methods
    # -------------------------------------------------------------------------

    def connect(self) -> None:
        """Connect to the PCA9685 and initialize it.

        Performs:
        1. Opens I2C bus if needed
        2. Resets the device
        3. Configures MODE1 and MODE2 registers
        4. Sets initial PWM frequency
        5. Wakes the device from sleep

        Raises:
            HardwareNotFoundError: If device is not found at address.
            CommunicationError: If communication fails.
        """
        if self._simulation_mode:
            self._state = DriverState.CONNECTED
            self._is_sleeping = False
            logger.debug("PCA9685 simulation connected")
            return

        try:
            # Open bus if not already open
            if self._bus is not None and not self._bus.is_open:
                self._bus.open()

            # Check device is present
            self._verify_device()

            # Reset and configure
            self._reset()
            self._configure_mode2()
            self._configure_mode1()

            # Set initial frequency
            self.set_frequency(self._pca_config.frequency)

            # Wake up
            self.wake()

            self._state = DriverState.CONNECTED
            logger.info(
                "PCA9685 at %#04x connected, frequency=%dHz",
                self._address,
                self._current_frequency,
            )

        except Exception as e:
            self._state = DriverState.ERROR
            raise CommunicationError(
                f"Failed to connect to PCA9685 at {self._address:#04x}: {e}"
            ) from e

    def disconnect(self) -> None:
        """Disconnect from the PCA9685.

        Puts device in sleep mode and closes I2C bus if we opened it.
        """
        if self._simulation_mode:
            self._state = DriverState.DISCONNECTED
            logger.debug("PCA9685 simulation disconnected")
            return

        try:
            # Put device to sleep
            self.sleep()

            self._state = DriverState.DISCONNECTED
            logger.info("PCA9685 at %#04x disconnected", self._address)

        except Exception as e:
            logger.warning("Error during PCA9685 disconnect: %s", e)
            self._state = DriverState.DISCONNECTED

    # -------------------------------------------------------------------------
    # Channel Operations
    # -------------------------------------------------------------------------

    def _write_channel(self, channel: int, value: float) -> None:
        """Write a normalized value (0-1) to a PWM channel.

        Args:
            channel: Channel number (0-15).
            value: Duty cycle from 0.0 (off) to 1.0 (full on).
        """
        # Convert normalized value to 12-bit PWM
        if value <= 0.0:
            # Fully off - use special OFF bit
            self.set_pwm(channel, on=0, off=PCA9685_RESOLUTION)
        elif value >= 1.0:
            # Fully on - use special ON bit
            self.set_pwm(channel, on=PCA9685_RESOLUTION, off=0)
        else:
            # Normal PWM
            off_time = int(value * PCA9685_MAX_PWM)
            self.set_pwm(channel, on=0, off=off_time)

    def _read_channel(self, channel: int) -> float:
        """Read the current value of a PWM channel.

        Args:
            channel: Channel number (0-15).

        Returns:
            Duty cycle from 0.0 to 1.0.
        """
        on, off = self.get_pwm(channel)

        # Check special cases
        if on >= PCA9685_RESOLUTION:
            return 1.0  # Fully on
        if off >= PCA9685_RESOLUTION:
            return 0.0  # Fully off

        # Normal PWM - calculate duty cycle
        return off / PCA9685_MAX_PWM

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        """Set raw PWM values for a channel.

        The PCA9685 uses on/off times within the 4096-step cycle:
        - on: Count (0-4095) when output turns ON
        - off: Count (0-4095) when output turns OFF

        Special values:
        - on >= 4096: Output fully ON
        - off >= 4096: Output fully OFF

        Args:
            channel: Channel number (0-15).
            on: ON time (0-4096).
            off: OFF time (0-4096).
        """
        self._validate_channel(channel)

        # Clamp values
        on = max(0, min(on, PCA9685_RESOLUTION))
        off = max(0, min(off, PCA9685_RESOLUTION))

        if self._simulation_mode:
            # Store the duty cycle value for simulation
            duty = 0.0
            if on >= PCA9685_RESOLUTION:
                duty = 1.0
            elif off >= PCA9685_RESOLUTION:
                duty = 0.0
            elif off > 0:
                duty = off / PCA9685_MAX_PWM
            self._channel_values[channel] = duty
            logger.debug(
                "PCA9685 simulation set_pwm: ch=%d on=%d off=%d",
                channel,
                on,
                off,
            )
            return

        # Calculate register address for this channel
        reg = PCA9685Register.LED0_ON_L + (channel * 4)

        # Write all 4 bytes (ON_L, ON_H, OFF_L, OFF_H)
        data = bytes(
            [
                on & 0xFF,  # ON_L
                (on >> 8) & 0x1F,  # ON_H (5 bits)
                off & 0xFF,  # OFF_L
                (off >> 8) & 0x1F,  # OFF_H (5 bits)
            ]
        )

        if self._bus is not None:
            self._bus.write_register(self._address, reg, data)

        # Update cached value
        if on >= PCA9685_RESOLUTION:
            self._channel_values[channel] = 1.0
        elif off >= PCA9685_RESOLUTION:
            self._channel_values[channel] = 0.0
        else:
            self._channel_values[channel] = off / PCA9685_MAX_PWM

        logger.debug(
            "PCA9685 set_pwm: ch=%d on=%d off=%d",
            channel,
            on,
            off,
        )

    def get_pwm(self, channel: int) -> tuple[int, int]:
        """Get raw PWM values for a channel.

        Args:
            channel: Channel number (0-15).

        Returns:
            Tuple of (on_time, off_time).
        """
        self._validate_channel(channel)

        if self._simulation_mode:
            # Reconstruct from cached duty cycle
            duty = self._channel_values.get(channel, 0.0)
            if duty >= 1.0:
                return (PCA9685_RESOLUTION, 0)
            elif duty <= 0.0:
                return (0, PCA9685_RESOLUTION)
            else:
                off = int(duty * PCA9685_MAX_PWM)
                return (0, off)

        # Read 4 bytes from channel registers
        reg = PCA9685Register.LED0_ON_L + (channel * 4)

        if self._bus is not None:
            data = self._bus.read_register(self._address, reg, 4)
            on = data[0] | ((data[1] & 0x1F) << 8)
            off = data[2] | ((data[3] & 0x1F) << 8)
            return (on, off)

        return (0, 0)

    def set_all_pwm(self, on: int, off: int) -> None:
        """Set raw PWM values for all channels at once.

        Uses the ALL_LED registers for efficient control of all channels.

        Args:
            on: ON time (0-4096).
            off: OFF time (0-4096).
        """
        on = max(0, min(on, PCA9685_RESOLUTION))
        off = max(0, min(off, PCA9685_RESOLUTION))

        if self._simulation_mode:
            duty = 0.0
            if on >= PCA9685_RESOLUTION:
                duty = 1.0
            elif off >= PCA9685_RESOLUTION:
                duty = 0.0
            elif off > 0:
                duty = off / PCA9685_MAX_PWM
            for ch in range(16):
                self._channel_values[ch] = duty
            logger.debug("PCA9685 simulation set_all_pwm: on=%d off=%d", on, off)
            return

        # Write to ALL_LED registers
        data = bytes(
            [
                on & 0xFF,
                (on >> 8) & 0x1F,
                off & 0xFF,
                (off >> 8) & 0x1F,
            ]
        )

        if self._bus is not None:
            self._bus.write_register(self._address, PCA9685Register.ALL_LED_ON_L, data)

        # Update all cached values
        for ch in range(16):
            if on >= PCA9685_RESOLUTION:
                self._channel_values[ch] = 1.0
            elif off >= PCA9685_RESOLUTION:
                self._channel_values[ch] = 0.0
            else:
                self._channel_values[ch] = off / PCA9685_MAX_PWM

        logger.debug("PCA9685 set_all_pwm: on=%d off=%d", on, off)

    # -------------------------------------------------------------------------
    # Frequency Control
    # -------------------------------------------------------------------------

    def set_frequency(self, frequency: int) -> None:
        """Set the PWM frequency.

        The prescale value is calculated from:
        prescale = round(oscillator_freq / (4096 * frequency)) - 1

        The frequency change requires putting the device in sleep mode.

        Args:
            frequency: Frequency in Hz (24-1526).

        Raises:
            ValueError: If frequency is out of range.
        """
        if not PCA9685_MIN_FREQUENCY <= frequency <= PCA9685_MAX_FREQUENCY:
            raise ValueError(
                f"Frequency must be {PCA9685_MIN_FREQUENCY}-{PCA9685_MAX_FREQUENCY}Hz, "
                f"got {frequency}"
            )

        # Calculate prescale value
        prescale = round(PCA9685_OSCILLATOR_FREQ / (PCA9685_RESOLUTION * frequency)) - 1
        prescale = max(PCA9685_MIN_PRESCALE, min(PCA9685_MAX_PRESCALE, prescale))

        # Calculate actual frequency
        actual_freq = PCA9685_OSCILLATOR_FREQ / ((prescale + 1) * PCA9685_RESOLUTION)
        self._current_frequency = int(actual_freq)

        if self._simulation_mode:
            logger.debug(
                "PCA9685 simulation set_frequency: %dHz (prescale=%d)",
                self._current_frequency,
                prescale,
            )
            self._frequency = self._current_frequency
            return

        # Prescale can only be set when device is in sleep mode
        was_sleeping = self._is_sleeping
        if not was_sleeping:
            self.sleep()

        # Set prescale register
        if self._bus is not None:
            self._bus.write_register_byte(self._address, PCA9685Register.PRE_SCALE, prescale)

        # Wake up if we weren't sleeping before
        if not was_sleeping:
            self.wake()

        self._frequency = self._current_frequency
        logger.info(
            "PCA9685 frequency set to %dHz (prescale=%d)",
            self._current_frequency,
            prescale,
        )

    def _apply_frequency(self, frequency: int) -> None:
        """Apply frequency setting (called by base Driver class).

        Args:
            frequency: Frequency in Hz.
        """
        self.set_frequency(frequency)

    # -------------------------------------------------------------------------
    # Sleep/Wake Control
    # -------------------------------------------------------------------------

    def sleep(self) -> None:
        """Put the PCA9685 into sleep mode.

        In sleep mode, the oscillator is stopped and no PWM signals
        are generated. This reduces power consumption.
        """
        if self._simulation_mode:
            self._is_sleeping = True
            logger.debug("PCA9685 simulation: sleep")
            return

        if self._bus is not None:
            # Read current MODE1
            mode1 = self._bus.read_register_byte(self._address, PCA9685Register.MODE1)
            # Set SLEEP bit
            mode1 |= PCA9685Mode1.SLEEP
            self._bus.write_register_byte(self._address, PCA9685Register.MODE1, mode1)

        self._is_sleeping = True
        logger.debug("PCA9685 sleep")

    def wake(self) -> None:
        """Wake the PCA9685 from sleep mode.

        Clears the SLEEP bit and waits for oscillator to stabilize.
        If PWM channels were active before sleep, uses RESTART to resume.
        """
        if self._simulation_mode:
            self._is_sleeping = False
            logger.debug("PCA9685 simulation: wake")
            return

        if self._bus is not None:
            # Read current MODE1
            mode1 = self._bus.read_register_byte(self._address, PCA9685Register.MODE1)

            # Clear SLEEP bit
            mode1 &= ~PCA9685Mode1.SLEEP
            self._bus.write_register_byte(self._address, PCA9685Register.MODE1, mode1)

            # Wait for oscillator to stabilize
            time.sleep(PCA9685_RESTART_DELAY)

            # If RESTART bit was set (channels were active before sleep),
            # set it again to restart PWM
            if mode1 & PCA9685Mode1.RESTART:
                mode1 |= PCA9685Mode1.RESTART
                self._bus.write_register_byte(self._address, PCA9685Register.MODE1, mode1)

        self._is_sleeping = False
        logger.debug("PCA9685 wake")

    # -------------------------------------------------------------------------
    # Configuration Methods
    # -------------------------------------------------------------------------

    def set_all_call_enabled(self, enabled: bool) -> None:
        """Enable or disable response to all-call address.

        When enabled, the device responds to commands sent to
        the all-call address (0x70 by default) in addition to
        its own address.

        Args:
            enabled: True to enable all-call response.
        """
        if self._simulation_mode:
            logger.debug("PCA9685 simulation: all_call=%s", enabled)
            return

        if self._bus is not None:
            mode1 = self._bus.read_register_byte(self._address, PCA9685Register.MODE1)

            if enabled:
                mode1 |= PCA9685Mode1.ALLCALL
            else:
                mode1 &= ~PCA9685Mode1.ALLCALL

            self._bus.write_register_byte(self._address, PCA9685Register.MODE1, mode1)

        logger.debug("PCA9685 all_call=%s", enabled)

    def set_output_inverted(self, inverted: bool) -> None:
        """Set whether outputs are inverted.

        When inverted, the output logic is flipped:
        - Normal: High = ON, Low = OFF
        - Inverted: High = OFF, Low = ON

        Args:
            inverted: True to invert outputs.
        """
        if self._simulation_mode:
            logger.debug("PCA9685 simulation: inverted=%s", inverted)
            return

        if self._bus is not None:
            mode2 = self._bus.read_register_byte(self._address, PCA9685Register.MODE2)

            if inverted:
                mode2 |= PCA9685Mode2.INVRT
            else:
                mode2 &= ~PCA9685Mode2.INVRT

            self._bus.write_register_byte(self._address, PCA9685Register.MODE2, mode2)

        logger.debug("PCA9685 inverted=%s", inverted)

    def set_totem_pole(self, totem_pole: bool) -> None:
        """Set output driver type.

        Args:
            totem_pole: True for totem-pole outputs, False for open-drain.
        """
        if self._simulation_mode:
            logger.debug("PCA9685 simulation: totem_pole=%s", totem_pole)
            return

        if self._bus is not None:
            mode2 = self._bus.read_register_byte(self._address, PCA9685Register.MODE2)

            if totem_pole:
                mode2 |= PCA9685Mode2.OUTDRV
            else:
                mode2 &= ~PCA9685Mode2.OUTDRV

            self._bus.write_register_byte(self._address, PCA9685Register.MODE2, mode2)

        logger.debug("PCA9685 totem_pole=%s", totem_pole)

    # -------------------------------------------------------------------------
    # Helper Methods
    # -------------------------------------------------------------------------

    def _verify_device(self) -> None:
        """Verify that the PCA9685 is present at the address.

        Raises:
            HardwareNotFoundError: If device is not found.
        """
        if self._bus is None:
            return

        # Scan bus for our address
        devices = self._bus.scan()
        if self._address not in devices:
            raise HardwareNotFoundError(
                f"PCA9685 not found at address {self._address:#04x}. "
                f"Found devices at: {[f'{d:#04x}' for d in devices]}"
            )

    def _reset(self) -> None:
        """Reset the PCA9685 to known state.

        Writes 0x00 to MODE1 (except ALLCALL bit if enabled).
        """
        if self._bus is None:
            return

        # Put device to sleep (also clears RESTART bit)
        mode1: int = PCA9685Mode1.SLEEP
        if self._pca_config.all_call:
            mode1 |= PCA9685Mode1.ALLCALL

        self._bus.write_register_byte(self._address, PCA9685Register.MODE1, mode1)

        # Clear all PWM outputs
        self.set_all_pwm(0, 0)

        logger.debug("PCA9685 reset")

    def _configure_mode1(self) -> None:
        """Configure MODE1 register based on config."""
        if self._bus is None:
            return

        mode1: int = PCA9685Mode1.SLEEP  # Start in sleep mode

        if self._pca_config.auto_increment:
            mode1 |= PCA9685Mode1.AI
        if self._pca_config.all_call:
            mode1 |= PCA9685Mode1.ALLCALL

        self._bus.write_register_byte(self._address, PCA9685Register.MODE1, mode1)
        self._is_sleeping = True

    def _configure_mode2(self) -> None:
        """Configure MODE2 register based on config."""
        if self._bus is None:
            return

        mode2 = 0

        if self._pca_config.totem_pole:
            mode2 |= PCA9685Mode2.OUTDRV
        if self._pca_config.invert:
            mode2 |= PCA9685Mode2.INVRT
        if self._pca_config.output_change_on_ack:
            mode2 |= PCA9685Mode2.OCH

        self._bus.write_register_byte(self._address, PCA9685Register.MODE2, mode2)

    def __repr__(self) -> str:
        """String representation."""
        mode = "simulation" if self._simulation_mode else "hardware"
        return (
            f"PCA9685(address={self._address:#04x}, "
            f"frequency={self._current_frequency}Hz, "
            f"mode={mode}, "
            f"state={self._state.value})"
        )
