"""Tier 1 Unit Tests: Dual-Axis Pan/Tilt Discrete PID Controller (Feature F14).

Verifies proportional response, anti-windup clamping, filtered derivative damping,
deadband noise rejection, [0, 180] degree angle clamping, and target loss reset.
"""

from __future__ import annotations

from typing import Tuple
import pytest


class SingleAxisPID:
    """Discrete PID controller with anti-windup clamping and deadband."""

    def __init__(
        self,
        kp: float = 0.5,
        ki: float = 0.05,
        kd: float = 0.02,
        integral_max: float = 20.0,
        deadband: float = 0.5,
        out_min: float = 0.0,
        out_max: float = 180.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self.deadband = deadband
        self.out_min = out_min
        self.out_max = out_max

        self.current_angle = 90.0  # Center
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, target_angle: float, dt: float) -> float:
        dt = max(1e-4, dt)
        error = target_angle - self.current_angle

        # Deadband filtering
        if abs(error) < self.deadband:
            return self.current_angle

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup clamping
        self.integral += error * dt
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        self.prev_error = error

        # Adjustment
        adjustment = p_term + i_term + d_term
        self.current_angle += adjustment

        # Output range clamping
        self.current_angle = max(self.out_min, min(self.out_max, self.current_angle))
        return self.current_angle


class TurretController:
    """Dual-axis (Pan/Tilt) PID controller for turret steering."""

    def __init__(self):
        self.pan_pid = SingleAxisPID(kp=0.4, ki=0.02, kd=0.01)
        self.tilt_pid = SingleAxisPID(kp=0.4, ki=0.02, kd=0.01)

    def reset(self) -> None:
        self.pan_pid.reset()
        self.tilt_pid.reset()

    def update_lead_target(
        self, target_pan_deg: float, target_tilt_deg: float, dt: float
    ) -> Tuple[float, float]:
        pan = self.pan_pid.update(target_pan_deg, dt)
        tilt = self.tilt_pid.update(target_tilt_deg, dt)
        return (pan, tilt)


def test_pid_proportional_response():
    """T1.8.1: Verifies proportional term moves current angle towards target."""
    pid = SingleAxisPID(kp=0.5, ki=0.0, kd=0.0, deadband=0.0)
    # Start at 90 deg, target at 100 deg (error = +10 deg)
    out = pid.update(100.0, dt=0.1)
    # adjustment = 0.5 * 10 = +5 deg -> current = 95 deg
    assert out == pytest.approx(95.0)


def test_pid_anti_windup_clamping():
    """T1.8.2: Verifies integral accumulator is strictly bounded by integral_max."""
    pid = SingleAxisPID(kp=0.0, ki=1.0, kd=0.0, integral_max=15.0)
    # Constant error of 10 for 50 steps (unclamped would accumulate to 500)
    for _ in range(50):
        pid.update(100.0, dt=0.1)

    assert abs(pid.integral) <= 15.0


def test_pid_derivative_damping():
    """T1.8.3: Verifies D-term opposes rapid change, dampening overshoot."""
    pid = SingleAxisPID(kp=0.0, ki=0.0, kd=0.5)
    # Step 1: establish error of 20
    pid.prev_error = 20.0
    # Step 2: error drops to 10 (derivative = (10 - 20)/0.1 = -100)
    # d_term = 0.5 * -100 = -50
    out = pid.update(100.0, dt=0.1)
    assert out < 90.0  # Damped downwards


def test_pid_deadband_filtering():
    """T1.8.4: Verifies errors within deadband (+/- 0.5 deg) do not trigger adjustments."""
    pid = SingleAxisPID(kp=1.0, ki=0.0, kd=0.0, deadband=0.8)
    pid.current_angle = 90.0

    # Target 90.5 is within 0.8 deadband
    out = pid.update(90.5, dt=0.1)
    assert out == 90.0


def test_pid_angle_clamping_0_180():
    """T1.8.5: Verifies output commands are strictly bounded between 0 and 180 degrees."""
    turret = TurretController()

    # Extreme out of bounds inputs
    pan, tilt = turret.update_lead_target(target_pan_deg=250.0, target_tilt_deg=-50.0, dt=1.0)
    assert 0.0 <= pan <= 180.0
    assert 0.0 <= tilt <= 180.0

    # Repeated saturation
    for _ in range(20):
        pan, tilt = turret.update_lead_target(300.0, -100.0, dt=0.1)
    assert pan == 180.0
    assert tilt == 0.0


def test_pid_reset_state():
    """T1.8.6: Verifies reset() clears internal integral and derivative history."""
    turret = TurretController()
    turret.update_lead_target(120.0, 120.0, dt=0.1)
    assert turret.pan_pid.integral != 0.0 or turret.pan_pid.prev_error != 0.0

    turret.reset()
    assert turret.pan_pid.integral == 0.0
    assert turret.pan_pid.prev_error == 0.0
    assert turret.tilt_pid.integral == 0.0
    assert turret.tilt_pid.prev_error == 0.0
