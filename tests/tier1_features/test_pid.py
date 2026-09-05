"""
Tier 1 Feature Tests: F14 - Dual-Axis Discrete PID Turret Controller.
"""

from __future__ import annotations

import pytest

from drone_turret.control.pid import (
    DEFAULT_DEADBAND_DEG,
    DiscretePID,
    TurretController,
)


def test_pid_deadband_filtering():
    """Verifies that error within deadband (±0.3°) results in zero corrective action."""
    pid = DiscretePID(kp=0.5, ki=0.0, kd=0.0, deadband=0.3, initial_angle=90.0)
    
    # Target 90.2° -> error 0.2° <= 0.3° -> filtered error = 0.0 -> no movement
    out1 = pid.update(target_deg=90.2, dt=0.033)
    assert out1 == 90.0
    assert pid.telemetry.filtered_error_deg == 0.0

    # Target 90.5° -> error 0.5° > 0.3° -> filtered error = 0.5 -> movement occurs
    out2 = pid.update(target_deg=90.5, dt=0.033)
    assert out2 > 90.0
    assert pid.telemetry.filtered_error_deg == 0.5


def test_pid_anti_windup_integral_clamping():
    """Verifies that integral accumulator is strictly clamped to i_max."""
    i_max = 4.0
    pid = DiscretePID(kp=0.0, ki=0.1, kd=0.0, i_max=i_max, deadband=0.0, initial_angle=90.0)
    
    # Apply constant large error over many long time steps
    for _ in range(50):
        pid.update(target_deg=150.0, dt=1.0)
    
    assert pid.telemetry.integral_accum == i_max
    assert pid.telemetry.i_term == pytest.approx(0.1 * i_max, abs=1e-4)


def test_pid_slew_rate_limiting():
    """Verifies that maximum angular change per update step is capped to max_step (10.0°)."""
    max_step = 10.0
    pid = DiscretePID(kp=1.0, ki=0.0, kd=0.0, max_step=max_step, initial_angle=90.0)
    
    # Request huge step: 90° -> 160° (error = 70°)
    out = pid.update(target_deg=160.0, dt=0.033)
    
    # Should only advance by max_step
    assert out == 90.0 + max_step
    assert pid.telemetry.step_output_deg == max_step


def test_pid_absolute_servo_angle_clamping():
    """Verifies output angles are strictly bounded to physical [0.0°, 180.0°]."""
    pid = DiscretePID(kp=1.0, max_step=50.0, initial_angle=170.0)
    
    # Overshoot past 180°
    out_high = pid.update(target_deg=220.0, dt=0.033)
    assert out_high == 180.0

    # Undershoot below 0°
    pid.reset(initial_angle=10.0)
    out_low = pid.update(target_deg=-50.0, dt=0.033)
    assert out_low == 0.0


def test_turret_controller_lead_point_aiming_and_telemetry():
    """Verifies dual-axis TurretController coordinates pan/tilt to lead target with telemetry."""
    controller = TurretController(
        pan_kp=0.2,
        tilt_kp=0.2,
        deadband_deg=0.3,
        max_step_deg=10.0,
        home_pan_deg=90.0,
        home_tilt_deg=90.0,
    )
    
    # Step towards lead target: Pan 110.0°, Tilt 75.0°
    cmd_pan, cmd_tilt = controller.update_lead_target(
        target_pan_deg=110.0,
        target_tilt_deg=75.0,
        dt=0.033,
    )
    
    assert cmd_pan > 90.0
    assert cmd_tilt < 90.0
    
    telemetry = controller.get_telemetry()
    assert "pan" in telemetry and "tilt" in telemetry
    assert telemetry["lead_angles"]["pan"] == 110.0
    assert telemetry["lead_angles"]["tilt"] == 75.0

    # Verify home resets back to 90°, 90°
    controller.reset()
    assert controller.current_angles == (90.0, 90.0)
