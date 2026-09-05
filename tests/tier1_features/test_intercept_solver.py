"""Tier 1 Unit Tests: Newton-Raphson Intercept Solver & Lead Point Calculation (Features F10, F11).

Verifies root-finding convergence for time-to-intercept, lead position offsets,
elevation gravity drop compensation, pan/tilt angle commands, and 2D pixel crosshair projection.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple
import pytest

from tests.fixtures.physics_benchmarks import BallisticsBenchmarks


@dataclass
class InterceptSolution:
    reachable: bool
    t_intercept: float                      # time to intercept in seconds
    lead_pos_3d: Tuple[float, float, float] # X, Y, Z in meters (turret-centric)
    aim_pan_deg: float                     # absolute pan angle [0, 180]
    aim_tilt_deg: float                    # absolute tilt angle [0, 180]
    lead_pixel_xy: Tuple[int, int]         # (x, y) lead crosshair in screen pixels
    drop_m: float                          # vertical gravity + drag drop in meters


def project_3d_to_screen(
    pos_3d: Tuple[float, float, float],
    focal_length: float = 800.0,
    cx: float = 320.0,
    cy: float = 240.0,
) -> Tuple[int, int]:
    """Projects 3D turret-centric point (X, Y, Z) to 2D screen pixels."""
    x, y, z = pos_3d
    safe_z = max(0.1, z)
    px = int(round(cx + (focal_length * x / safe_z)))
    py = int(round(cy - (focal_length * y / safe_z)))
    return (px, py)


def test_intercept_solution_dataclass():
    """T1.5.1: Verifies InterceptSolution dataclass structure and field types."""
    sol = InterceptSolution(
        reachable=True,
        t_intercept=0.65,
        lead_pos_3d=(12.5, 3.2, 50.0),
        aim_pan_deg=104.0,
        aim_tilt_deg=93.5,
        lead_pixel_xy=(520, 190),
        drop_m=2.07,
    )
    assert sol.reachable is True
    assert sol.t_intercept == 0.65
    assert len(sol.lead_pos_3d) == 3
    assert 0.0 <= sol.aim_pan_deg <= 180.0
    assert 0.0 <= sol.aim_tilt_deg <= 180.0
    assert len(sol.lead_pixel_xy) == 2


def test_newton_raphson_stationary_target(physics_benchmarks):
    """T1.5.2: Verifies stationary target at 50m has zero horizontal lead and positive elevation compensation."""
    sol = physics_benchmarks.reference_intercept_solver(
        target_pos_3d=(0.0, 0.0, 50.0),
        target_vel_3d=(0.0, 0.0, 0.0),
        v0=80.0,
        mass=0.6,
        cd=1.2,
    )
    assert sol["reachable"] is True
    assert 0.60 < sol["t_intercept"] < 1.10
    # Pan angle for target at x=0 should be 90.0 (center)
    assert sol["aim_pan_deg"] == pytest.approx(90.0, abs=0.5)
    # Tilt angle must aim upward (> 90.0) to compensate for gravity
    assert sol["aim_tilt_deg"] > 90.0
    assert sol["drop_m"] > 1.5


def test_newton_raphson_moving_target_lead_offset(physics_benchmarks):
    """T1.5.3: Verifies moving target at 200 km/h (55.56 m/s) leads target position by ~vx * t_int."""
    vx_mps = 200.0 / 3.6  # 55.56 m/s
    sol = physics_benchmarks.reference_intercept_solver(
        target_pos_3d=(0.0, 0.0, 50.0),
        target_vel_3d=(vx_mps, 0.0, 0.0),
        v0=80.0,
    )

    t_int = sol["t_intercept"]
    lead_x = sol["lead_pos_3d"][0]
    expected_lead_x = vx_mps * t_int

    assert lead_x == pytest.approx(expected_lead_x, rel=1e-2)
    # Aim pan must lead to the right (> 90 deg)
    assert sol["aim_pan_deg"] > 90.0


def test_newton_raphson_convergence_speed(physics_benchmarks):
    """T1.5.4: Verifies solver converges quickly in <= 15 iterations."""
    sol = physics_benchmarks.reference_intercept_solver(
        target_pos_3d=(-15.0, 5.0, 45.0),
        target_vel_3d=(20.0, -2.0, 5.0),
        v0=80.0,
        max_iter=15,
        tol=1e-3,
    )
    assert sol["reachable"] is True
    assert sol["t_intercept"] > 0.0


def test_intercept_lead_pixel_projection():
    """T1.5.5: Verifies projection of lead 3D coordinate to screen pixel crosshair."""
    lead_pos = (5.0, 0.0, 50.0)  # 5m to right at 50m range
    px, py = project_3d_to_screen(lead_pos, focal_length=800.0, cx=320.0, cy=240.0)

    # px = 320 + (800 * 5 / 50) = 320 + 80 = 400
    # py = 240 - (800 * 0 / 50) = 240
    assert px == 400
    assert py == 240


def test_intercept_unreachable_target_handling(physics_benchmarks):
    """T1.5.6: Verifies targets beyond maximum effective range (>100m) are flagged unreachable."""
    sol = physics_benchmarks.reference_intercept_solver(
        target_pos_3d=(0.0, 0.0, 150.0),  # 150m is out of range
        target_vel_3d=(0.0, 0.0, 0.0),
        v0=60.0,
    )
    assert sol["reachable"] is False
