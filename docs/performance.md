# Performance Targets

This document defines the performance targets and benchmarks for robo-infra components.

## Overview

robo-infra is designed for real-time robotics applications. All core operations must meet strict latency requirements to ensure reliable robot control.

## Performance Categories

###  Real-time Critical (< 1ms)

Operations that must complete within 1ms for real-time control loops:

| Operation | Target | Measured | Status |
|-----------|--------|----------|--------|
| Sensor read (simulated) | < 1ms | ~0.01ms | [OK] |
| Actuator set command | < 1ms | ~0.01ms | [OK] |
| Actuator get position | < 1ms | ~0.01ms | [OK] |
| I2C byte read/write | < 1ms | ~0.01ms | [OK] |
| SPI transfer (4 bytes) | < 1ms | ~0.01ms | [OK] |
| Buffer operations | < 1ms | ~0.01ms | [OK] |

###  Fast Operations (< 10ms)

Operations that should complete quickly but aren't in the critical path:

| Operation | Target | Measured | Status |
|-----------|--------|----------|--------|
| Controller creation | < 10ms | ~1ms | [OK] |
| Actuator creation | < 10ms | ~1ms | [OK] |
| Sensor creation | < 10ms | ~1ms | [OK] |
| I2C/SPI bus creation | < 10ms | ~1ms | [OK] |
| Trajectory point generation | < 10ms | ~0.5ms | [OK] |

###  Startup Operations (< 100ms)

Operations that occur during initialization:

| Operation | Target | Measured | Status |
|-----------|--------|----------|--------|
| Controller with 6 actuators | < 20ms | ~5ms | [OK] |
| Controller with 4 sensors | < 20ms | ~5ms | [OK] |
| Full lifecycle cycle | < 50ms | ~10ms | [OK] |

## Kinematics Performance

### Forward Kinematics

| Solver | DOF | Target | Status |
|--------|-----|--------|--------|
| TwoLinkArm | 2 | < 0.1ms | [OK] |
| ThreeLinkArm | 3 | < 0.1ms | [OK] |
| DH Chain | 3 | < 0.1ms | [OK] |
| DH Chain | 6 | < 0.5ms | [OK] |
| PUMA 560 | 6 | < 0.5ms | [OK] |
| Delta Robot | 3 | < 0.1ms | [OK] |
| SCARA | 4 | < 0.1ms | [OK] |

### Inverse Kinematics

| Solver | DOF | Target | Status |
|--------|-----|--------|--------|
| TwoLinkArm (analytical) | 2 | < 50ms | [OK] |
| ThreeLinkArm (analytical) | 3 | < 50ms | [OK] |
| CCD Solver | 3 | < 50ms | [OK] |
| Damped Least Squares | 6 | < 100ms | [OK] |
| Gradient Descent | 6 | < 100ms | [OK] |
| Delta Robot | 3 | < 50ms | [OK] |
| SCARA | 4 | < 50ms | [OK] |

### Trajectory Generation

| Profile | Points | Target | Status |
|---------|--------|--------|--------|
| Linear | 100 | < 10ms | [OK] |
| Cubic | 100 | < 10ms | [OK] |
| Quintic | 100 | < 10ms | [OK] |
| Trapezoidal | 100 | < 10ms | [OK] |
| S-Curve | 100 | < 10ms | [OK] |

## IMU Fusion

| Filter | Target Rate | Measured | Status |
|--------|-------------|----------|--------|
| Madgwick (6-DOF) | > 500 Hz | ~10000 Hz | [OK] |
| Madgwick (9-DOF) | > 500 Hz | ~8000 Hz | [OK] |
| Mahony (6-DOF) | > 500 Hz | ~10000 Hz | [OK] |

## Memory Usage

| Scenario | Threshold | Status |
|----------|-----------|--------|
| Controller + 6 actuators + 4 sensors | < 1MB | [OK] |

## Running Benchmarks

### Local Testing

```bash
# Run all benchmarks
poetry run pytest tests/benchmarks/ -v

# Run specific benchmark category
poetry run pytest tests/benchmarks/test_bench_kinematics.py -v

# Show timing details
poetry run pytest tests/benchmarks/ -v --durations=0
```

### CI Integration

Benchmarks run automatically on:
- Push to `main` branch
- Pull requests to `main`
- Changes to `src/` or `tests/benchmarks/`

View benchmark results in the GitHub Actions workflow artifacts.

## Performance Guidelines

### For Contributors

1. **Always run benchmarks** before submitting PRs that modify core modules
2. **Add benchmarks** for new real-time critical operations
3. **Document performance characteristics** in docstrings
4. **Use profiling** to identify bottlenecks before optimization

### For Users

1. **Check platform performance** - Results may vary by hardware
2. **Use simulated components** for testing to eliminate hardware variability
3. **Profile your specific use case** - Generic benchmarks may not match your workload
4. **Consider batch operations** when performance is critical

## Optimization Techniques

### Applied Optimizations

- **NumPy vectorization** for matrix operations in kinematics
- **Pre-computed lookup tables** where applicable
- **Lazy initialization** for expensive resources
- **Object pooling** for frequently allocated objects

### Measured vs Theoretical

Our benchmarks measure real-world performance including:
- Python interpreter overhead
- Memory allocation
- Context switching
- GC pauses (minimized but present)

Theoretical peak performance would be higher, but our targets are based on practical measurements.

## Baseline Metrics (2026-01-04)

The following baseline metrics were measured on Apple Silicon (M-series):

| Metric | Value | Notes |
|--------|-------|-------|
| Import time | 0.152s | Full `import robo_infra` |
| PID control loop | 1254 kHz | 0.8 us/iteration average |
| Sensor read latency | 1.0 us | SimulatedSensor.read() |
| Actuator command latency | 1.0 us | DCMotor.forward() |
| Sensor throughput | 1017 kHz | Reads per second |
| Actuator command rate | 969 kHz | Commands per second |

All core operations exceed the 1 kHz target required for real-time robotics control.
