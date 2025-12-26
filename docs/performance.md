# Performance Targets

This document defines the performance targets and benchmarks for robo-infra components.

## Overview

robo-infra is designed for real-time robotics applications. All core operations must meet strict latency requirements to ensure reliable robot control.

## Performance Categories

### 🎯 Real-time Critical (< 1ms)

Operations that must complete within 1ms for real-time control loops:

| Operation | Target | Measured | Status |
|-----------|--------|----------|--------|
| Sensor read (simulated) | < 1ms | ~0.01ms | ✅ |
| Actuator set command | < 1ms | ~0.01ms | ✅ |
| Actuator get position | < 1ms | ~0.01ms | ✅ |
| I2C byte read/write | < 1ms | ~0.01ms | ✅ |
| SPI transfer (4 bytes) | < 1ms | ~0.01ms | ✅ |
| Buffer operations | < 1ms | ~0.01ms | ✅ |

### ⚡ Fast Operations (< 10ms)

Operations that should complete quickly but aren't in the critical path:

| Operation | Target | Measured | Status |
|-----------|--------|----------|--------|
| Controller creation | < 10ms | ~1ms | ✅ |
| Actuator creation | < 10ms | ~1ms | ✅ |
| Sensor creation | < 10ms | ~1ms | ✅ |
| I2C/SPI bus creation | < 10ms | ~1ms | ✅ |
| Trajectory point generation | < 10ms | ~0.5ms | ✅ |

### 🔄 Startup Operations (< 100ms)

Operations that occur during initialization:

| Operation | Target | Measured | Status |
|-----------|--------|----------|--------|
| Controller with 6 actuators | < 20ms | ~5ms | ✅ |
| Controller with 4 sensors | < 20ms | ~5ms | ✅ |
| Full lifecycle cycle | < 50ms | ~10ms | ✅ |

## Kinematics Performance

### Forward Kinematics

| Solver | DOF | Target | Status |
|--------|-----|--------|--------|
| TwoLinkArm | 2 | < 0.1ms | ✅ |
| ThreeLinkArm | 3 | < 0.1ms | ✅ |
| DH Chain | 3 | < 0.1ms | ✅ |
| DH Chain | 6 | < 0.5ms | ✅ |
| PUMA 560 | 6 | < 0.5ms | ✅ |
| Delta Robot | 3 | < 0.1ms | ✅ |
| SCARA | 4 | < 0.1ms | ✅ |

### Inverse Kinematics

| Solver | DOF | Target | Status |
|--------|-----|--------|--------|
| TwoLinkArm (analytical) | 2 | < 50ms | ✅ |
| ThreeLinkArm (analytical) | 3 | < 50ms | ✅ |
| CCD Solver | 3 | < 50ms | ✅ |
| Damped Least Squares | 6 | < 100ms | ✅ |
| Gradient Descent | 6 | < 100ms | ✅ |
| Delta Robot | 3 | < 50ms | ✅ |
| SCARA | 4 | < 50ms | ✅ |

### Trajectory Generation

| Profile | Points | Target | Status |
|---------|--------|--------|--------|
| Linear | 100 | < 10ms | ✅ |
| Cubic | 100 | < 10ms | ✅ |
| Quintic | 100 | < 10ms | ✅ |
| Trapezoidal | 100 | < 10ms | ✅ |
| S-Curve | 100 | < 10ms | ✅ |

## IMU Fusion

| Filter | Target Rate | Measured | Status |
|--------|-------------|----------|--------|
| Madgwick (6-DOF) | > 500 Hz | ~10000 Hz | ✅ |
| Madgwick (9-DOF) | > 500 Hz | ~8000 Hz | ✅ |
| Mahony (6-DOF) | > 500 Hz | ~10000 Hz | ✅ |

## Memory Usage

| Scenario | Threshold | Status |
|----------|-----------|--------|
| Controller + 6 actuators + 4 sensors | < 1MB | ✅ |

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
