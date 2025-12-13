"""Hardware driver implementations.

This package provides driver implementations for common hardware:
- SimulationDriver: Enhanced simulation driver for testing
- (Coming soon) PCA9685: 16-channel PWM driver
- (Coming soon) L298N: Dual H-bridge motor driver
- (Coming soon) GPIODriver: Direct GPIO control
"""

from robo_infra.drivers.simulation import (
    ChannelHistory,
    OperationRecord,
    OperationType,
    SimulationDriver,
)


__all__ = [
    "ChannelHistory",
    "OperationRecord",
    "OperationType",
    "SimulationDriver",
]
