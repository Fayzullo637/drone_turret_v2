"""Tier 2 Boundary Tests: BallisticCalculator Edge Cases & Limits.

Tests edge conditions:
- Out of maximum range targets (> 75m)
- Min engagement range (< 1m)
- Escaping / supersonic receding targets
- Near-vertical zenith target handling
- Extreme servo clamping [0, 180] deg
- Coasting target predictions
"""

import math
import numpy as np
import pytest

from drone_turret.ballistics import (
    BallisticCalculator,
    BallisticConfig,
    TargetState,
)


def test_out_of_range_target_rejection():
    """T2.B.1: Target beyond maximum range is flagged unreachable."""
    calc = BallisticCalculator()
    target = TargetState(pos_3d=(0.0, 0.0, 150.0), vel_3d=(0.0, 0.0, 0.0))
    sol = calc.solve_intercept(target)

    assert sol.reachable is False
    assert sol.lead_pos_3d == (0.0, 0.0, 150.0)
    assert sol.drop_m == 0.0


def test_zero_and_below_min_range_target():
    """T2.B.2: Target at 0 distance does not crash and is handled safely."""
    calc = BallisticCalculator()
    target = TargetState(pos_3d=(0.0, 0.0, 0.2), vel_3d=(0.0, 0.0, 0.0))
    sol = calc.solve_intercept(target)

    assert sol.reachable is False


def test_fast_escaping_target():
    """T2.B.3: Fast receding target (vz=120 m/s > v0=80 m/s) is unreachable."""
    calc = BallisticCalculator()
    target = TargetState(pos_3d=(0.0, 0.0, 40.0), vel_3d=(0.0, 0.0, 120.0), speed_kmh=432.0)
    sol = calc.solve_intercept(target)

    assert sol.reachable is False


def test_near_zenith_singularity_handling():
    """T2.B.4: Target directly overhead does not produce NaN angles."""
    calc = BallisticCalculator()
    target = TargetState(pos_3d=(0.0, 40.0, 0.1), vel_3d=(0.0, 0.0, 0.0))
    sol = calc.solve_intercept(target)

    assert not math.isnan(sol.aim_pan_deg)
    assert not math.isnan(sol.aim_tilt_deg)
    assert 0.0 <= sol.aim_pan_deg <= 180.0
    assert 0.0 <= sol.aim_tilt_deg <= 180.0
    assert sol.aim_tilt_deg > 120.0  # High elevation


def test_servo_angle_clamping():
    """T2.B.5: Extreme aim points clamp to configured servo limits [0, 180] deg."""
    calc = BallisticCalculator()

    # Extreme right target (+X = 100m, Z = 1m -> azimuth > 89 deg -> pan > 179 deg)
    pan, tilt = calc.compute_aim_angles((100.0, 0.0, 1.0))
    assert 0.0 <= pan <= 180.0
    assert pan == pytest.approx(180.0, abs=1.0)

    # Extreme left target (-X = 100m, Z = 1m -> azimuth < -89 deg -> pan < 1 deg)
    pan_l, _ = calc.compute_aim_angles((-100.0, 0.0, 1.0))
    assert 0.0 <= pan_l <= 180.0
    assert pan_l == pytest.approx(0.0, abs=1.0)


def test_coasting_target_state_propagation():
    """T2.B.6: Target in coasting state extrapolates position smoothly with acceleration."""
    calc = BallisticCalculator()
    target = TargetState(
        pos_3d=(0.0, 5.0, 30.0),
        vel_3d=(10.0, 0.0, 0.0),
        acc_3d=(2.0, 0.0, 0.0),
        is_coasting=True,
        coast_frames=5,
    )
    sol = calc.solve_intercept(target)

    assert sol.reachable is True
    assert sol.t_intercept > 0.0
    # Pos extrapolated: x = 0 + 10*t + 0.5*2*t^2 = 10t + t^2
    t = sol.t_intercept
    expected_x = 10.0 * t + 1.0 * (t**2)
    assert sol.lead_pos_3d[0] == pytest.approx(expected_x, rel=1e-3)
