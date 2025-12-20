"""Tests for robo_infra.motion.pid module."""

from __future__ import annotations

import time

import pytest

from robo_infra.motion.pid import PID, PIDConfig


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def pid() -> PID:
    """Create a default PID controller."""
    return PID(kp=1.0, ki=0.1, kd=0.05)


@pytest.fixture
def pid_config() -> PIDConfig:
    """Create a custom PID configuration."""
    return PIDConfig(
        kp=2.0,
        ki=0.5,
        kd=0.1,
        output_min=-100.0,
        output_max=100.0,
        integral_min=-50.0,
        integral_max=50.0,
        derivative_filter=0.2,
        sample_time=0.02,
    )


# =============================================================================
# Basic Functionality Tests
# =============================================================================


class TestPIDInit:
    """Tests for PID initialization."""

    def test_pid_init_default(self) -> None:
        """Test PID initialization with default values."""
        pid = PID()
        assert pid.kp == 1.0
        assert pid.ki == 0.0
        assert pid.kd == 0.0
        assert pid.config.output_min == -1.0
        assert pid.config.output_max == 1.0
        assert pid.config.sample_time == 0.01

    def test_pid_init_with_gains(self) -> None:
        """Test PID initialization with custom gains."""
        pid = PID(kp=2.0, ki=0.5, kd=0.1)
        assert pid.kp == 2.0
        assert pid.ki == 0.5
        assert pid.kd == 0.1

    def test_pid_init_with_config(self, pid_config: PIDConfig) -> None:
        """Test PID initialization with config object."""
        pid = PID(config=pid_config)
        assert pid.kp == 2.0
        assert pid.ki == 0.5
        assert pid.kd == 0.1
        assert pid.config.output_min == -100.0
        assert pid.config.output_max == 100.0
        assert pid.config.integral_min == -50.0
        assert pid.config.integral_max == 50.0

    def test_pid_init_gains_override_config(self, pid_config: PIDConfig) -> None:
        """Test that explicit gains override config gains."""
        pid = PID(kp=5.0, ki=1.0, kd=0.5, config=pid_config)
        assert pid.kp == 5.0
        assert pid.ki == 1.0
        assert pid.kd == 0.5

    def test_pid_repr(self) -> None:
        """Test PID string representation."""
        pid = PID(kp=1.5, ki=0.2, kd=0.05)
        repr_str = repr(pid)
        assert "PID" in repr_str
        assert "1.500" in repr_str or "1.5" in repr_str


# =============================================================================
# Proportional, Integral, Derivative Term Tests
# =============================================================================


class TestPIDTerms:
    """Tests for individual PID terms."""

    def test_pid_proportional_only(self) -> None:
        """Test PID with only proportional term."""
        config = PIDConfig(output_min=-100.0, output_max=100.0)
        pid = PID(kp=2.0, ki=0.0, kd=0.0, config=config)

        # First update
        output = pid.update(setpoint=10.0, measurement=0.0)

        # P = kp * error = 2.0 * 10.0 = 20.0
        assert output == pytest.approx(20.0, rel=0.01)

    def test_pid_integral_only(self) -> None:
        """Test PID with only integral term."""
        config = PIDConfig(output_min=-100.0, output_max=100.0, sample_time=0.1)
        pid = PID(kp=0.0, ki=1.0, kd=0.0, config=config)

        # First update - integral = error * dt = 10 * 0.1 = 1.0
        output = pid.update(setpoint=10.0, measurement=0.0)

        # I = ki * integral = 1.0 * 1.0 = 1.0
        assert output == pytest.approx(1.0, rel=0.1)

    def test_pid_derivative_only(self) -> None:
        """Test PID with only derivative term."""
        config = PIDConfig(
            output_min=-1000.0,
            output_max=1000.0,
            derivative_filter=0.0,  # No filtering
            sample_time=0.1,
        )
        pid = PID(kp=0.0, ki=0.0, kd=1.0, config=config)

        # First update - establishes baseline
        pid.update(setpoint=0.0, measurement=0.0)

        # Wait a bit for time delta
        time.sleep(0.02)

        # Second update - measurement changed
        output = pid.update(setpoint=0.0, measurement=10.0)

        # Derivative = kd * (-d(measurement)/dt), should be negative since measurement increased
        assert output < 0

    def test_pid_all_terms(self) -> None:
        """Test PID with all terms active."""
        config = PIDConfig(output_min=-100.0, output_max=100.0)
        pid = PID(kp=1.0, ki=0.1, kd=0.01, config=config)

        output = pid.update(setpoint=50.0, measurement=0.0)

        # Should have contributions from all terms
        # P should be dominant with these gains
        assert 0 < output <= 100.0


# =============================================================================
# Output Limiting Tests
# =============================================================================


class TestPIDOutputLimiting:
    """Tests for PID output clamping."""

    def test_pid_output_clamped_max(self) -> None:
        """Test that output is clamped to maximum."""
        config = PIDConfig(output_min=-1.0, output_max=1.0)
        pid = PID(kp=10.0, ki=0.0, kd=0.0, config=config)

        # Large error should produce output > 1.0, but clamped
        output = pid.update(setpoint=100.0, measurement=0.0)

        assert output == 1.0

    def test_pid_output_clamped_min(self) -> None:
        """Test that output is clamped to minimum."""
        config = PIDConfig(output_min=-1.0, output_max=1.0)
        pid = PID(kp=10.0, ki=0.0, kd=0.0, config=config)

        # Large negative error should produce output < -1.0, but clamped
        output = pid.update(setpoint=-100.0, measurement=0.0)

        assert output == -1.0

    def test_pid_output_within_limits(self) -> None:
        """Test that output within limits is not clamped."""
        config = PIDConfig(output_min=-100.0, output_max=100.0)
        pid = PID(kp=1.0, ki=0.0, kd=0.0, config=config)

        output = pid.update(setpoint=50.0, measurement=0.0)

        assert output == pytest.approx(50.0, rel=0.01)


# =============================================================================
# Anti-Windup Tests
# =============================================================================


class TestPIDAntiWindup:
    """Tests for integral anti-windup."""

    def test_pid_integral_clamped(self) -> None:
        """Test that integral term is clamped."""
        config = PIDConfig(
            output_min=-100.0,
            output_max=100.0,
            integral_min=-5.0,
            integral_max=5.0,
        )
        pid = PID(kp=0.0, ki=1.0, kd=0.0, config=config)

        # Many updates with constant error should accumulate integral
        for _ in range(100):
            pid.update(setpoint=100.0, measurement=0.0)

        # Integral should be clamped
        assert pid.integral <= 5.0

    def test_pid_integral_windup_prevention(self) -> None:
        """Test conditional integration prevents windup when saturated."""
        config = PIDConfig(
            output_min=-10.0,
            output_max=10.0,
            integral_min=-100.0,
            integral_max=100.0,
        )
        pid = PID(kp=1.0, ki=1.0, kd=0.0, config=config)

        # Create saturation condition
        for _ in range(50):
            output = pid.update(setpoint=100.0, measurement=0.0)

        # Output should be saturated
        assert output == 10.0

        # Integral should not have grown excessively due to conditional integration
        assert pid.integral < 20.0


# =============================================================================
# Derivative Filtering Tests
# =============================================================================


class TestPIDDerivativeFiltering:
    """Tests for derivative term filtering."""

    def test_pid_derivative_on_measurement(self) -> None:
        """Test that derivative is computed on measurement, not error."""
        config = PIDConfig(
            output_min=-100.0,
            output_max=100.0,
            derivative_filter=0.0,
        )
        pid = PID(kp=0.0, ki=0.0, kd=1.0, config=config)

        # First call - baseline
        pid.update(setpoint=0.0, measurement=50.0)
        time.sleep(0.02)

        # Change setpoint dramatically, but measurement is stable
        pid.update(setpoint=100.0, measurement=50.0)

        # Since measurement didn't change much, derivative should be small
        # This tests that derivative is on measurement, not on error
        assert abs(pid.last_derivative) < 100

    def test_pid_derivative_filter(self) -> None:
        """Test low-pass filter on derivative."""
        # High filtering
        config_filtered = PIDConfig(
            output_min=-1000.0,
            output_max=1000.0,
            derivative_filter=0.9,  # Heavy filtering
        )
        pid_filtered = PID(kp=0.0, ki=0.0, kd=1.0, config=config_filtered)

        # No filtering
        config_unfiltered = PIDConfig(
            output_min=-1000.0,
            output_max=1000.0,
            derivative_filter=0.0,  # No filtering
        )
        pid_unfiltered = PID(kp=0.0, ki=0.0, kd=1.0, config=config_unfiltered)

        # Baseline
        pid_filtered.update(setpoint=0.0, measurement=0.0)
        pid_unfiltered.update(setpoint=0.0, measurement=0.0)
        time.sleep(0.02)

        # Step change
        pid_filtered.update(setpoint=0.0, measurement=100.0)
        pid_unfiltered.update(setpoint=0.0, measurement=100.0)

        # Filtered derivative should be smaller in magnitude
        assert abs(pid_filtered.last_derivative) < abs(pid_unfiltered.last_derivative)


# =============================================================================
# Control Method Tests
# =============================================================================


class TestPIDControl:
    """Tests for PID control methods."""

    def test_pid_reset(self) -> None:
        """Test PID reset clears state."""
        pid = PID(kp=1.0, ki=0.5, kd=0.1)

        # Accumulate some state
        for _ in range(10):
            pid.update(setpoint=100.0, measurement=50.0)

        assert pid.integral != 0.0
        assert pid.last_error != 0.0

        # Reset
        pid.reset()

        assert pid.integral == 0.0
        assert pid.last_error == 0.0
        assert pid.last_derivative == 0.0

    def test_pid_set_gains(self) -> None:
        """Test dynamic gain adjustment."""
        pid = PID(kp=1.0, ki=0.1, kd=0.01)

        pid.set_gains(kp=2.0, ki=0.2, kd=0.02)

        assert pid.kp == 2.0
        assert pid.ki == 0.2
        assert pid.kd == 0.02

    def test_pid_set_output_limits(self) -> None:
        """Test dynamic output limit adjustment."""
        pid = PID(kp=1.0, ki=0.0, kd=0.0)

        pid.set_output_limits(-50.0, 50.0)

        assert pid.config.output_min == -50.0
        assert pid.config.output_max == 50.0

    def test_pid_set_output_limits_invalid(self) -> None:
        """Test that invalid output limits raise error."""
        pid = PID(kp=1.0, ki=0.0, kd=0.0)

        with pytest.raises(ValueError, match="must be greater"):
            pid.set_output_limits(50.0, -50.0)

    def test_pid_set_sample_time(self) -> None:
        """Test dynamic sample time adjustment."""
        pid = PID(kp=1.0, ki=0.0, kd=0.0)

        pid.set_sample_time(0.05)

        assert pid.config.sample_time == 0.05

    def test_pid_set_sample_time_invalid(self) -> None:
        """Test that invalid sample time raises error."""
        pid = PID(kp=1.0, ki=0.0, kd=0.0)

        with pytest.raises(ValueError, match="must be positive"):
            pid.set_sample_time(-0.01)


# =============================================================================
# Step Response Tests
# =============================================================================


class TestPIDStepResponse:
    """Tests for PID step response behavior."""

    def test_pid_step_response_converges(self) -> None:
        """Test that PID converges toward setpoint."""
        config = PIDConfig(
            output_min=-10.0,
            output_max=10.0,
            sample_time=0.01,
        )
        pid = PID(kp=0.5, ki=0.1, kd=0.05, config=config)

        # Simulate a simple system: output directly affects measurement
        measurement = 0.0
        setpoint = 100.0

        for _ in range(200):  # More iterations for convergence
            output = pid.update(setpoint=setpoint, measurement=measurement)
            # Simple integrating system
            measurement += output * 0.1
            time.sleep(0.001)

        # Should have moved significantly toward setpoint
        assert measurement > 30.0  # More lenient threshold

    def test_pid_tracks_changing_setpoint(self) -> None:
        """Test that PID can track changing setpoint."""
        config = PIDConfig(output_min=-100.0, output_max=100.0)
        pid = PID(kp=1.0, ki=0.1, kd=0.0, config=config)

        # First setpoint
        output1 = pid.update(setpoint=50.0, measurement=0.0)

        # Change setpoint
        output2 = pid.update(setpoint=100.0, measurement=25.0)

        # Output should reflect new error
        assert output2 > output1 * 0.5


# =============================================================================
# Config Validation Tests
# =============================================================================


class TestPIDConfigValidation:
    """Tests for PIDConfig validation."""

    def test_config_defaults(self) -> None:
        """Test PIDConfig default values."""
        config = PIDConfig()
        assert config.kp == 1.0
        assert config.ki == 0.0
        assert config.kd == 0.0
        assert config.output_min == -1.0
        assert config.output_max == 1.0
        assert config.integral_min == -10.0
        assert config.integral_max == 10.0
        assert config.derivative_filter == 0.1
        assert config.sample_time == 0.01

    def test_config_output_max_greater_than_min(self) -> None:
        """Test that output_max must be greater than output_min."""
        with pytest.raises(ValueError, match="output_max.*must be greater"):
            PIDConfig(output_min=10.0, output_max=-10.0)

    def test_config_integral_max_greater_than_min(self) -> None:
        """Test that integral_max must be greater than integral_min."""
        with pytest.raises(ValueError, match="integral_max.*must be greater"):
            PIDConfig(integral_min=10.0, integral_max=-10.0)

    def test_config_derivative_filter_range(self) -> None:
        """Test derivative_filter must be between 0 and 1."""
        with pytest.raises(ValueError):
            PIDConfig(derivative_filter=-0.1)

        with pytest.raises(ValueError):
            PIDConfig(derivative_filter=1.5)

    def test_config_sample_time_positive(self) -> None:
        """Test sample_time must be positive."""
        with pytest.raises(ValueError):
            PIDConfig(sample_time=0.0)

        with pytest.raises(ValueError):
            PIDConfig(sample_time=-0.01)


__all__ = [
    "TestPIDAntiWindup",
    "TestPIDConfigValidation",
    "TestPIDControl",
    "TestPIDDerivativeFiltering",
    "TestPIDInit",
    "TestPIDOutputLimiting",
    "TestPIDStepResponse",
    "TestPIDTerms",
]
