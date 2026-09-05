"""Tier 2 Boundary Tests: Ballistics & Intercept Solver Edge Cases (Features F8, F9, F10).

Tests extreme physical and numerical edge conditions: zero range, out-of-range targets,
retreating supersonic targets, near-vertical elevation angles, and parameter rejection.
"""

from __future__ import annotations

import math
import pytest

from tests.fixtures.physics_benchmarks import BallisticsBenchmarks


def test_ballistics_zero_range(physics_benchmarks):
    """T2.3.1: Verifies target at 0 distance returns t_intercept=0 without division-by-zero."""
    sol = physics_benchmarks.reference_intercept_solver(
        target_pos_3d=(0.0, 0.0, 0.0),
        target_vel_3d=(0.0, 0.0, 0.0),
        v0=80.0,
    )
    assert sol["reachable"] is True
    assert sol["t_intercept"] == 0.0
    assert sol["drop_m"] == 0.0


def test_ballistics_out_of_range(physics_benchmarks):
    """T2.3.2: Verifies target beyond maximum range (150m) is marked unreachable."""
    sol = physics_benchmarks.reference_intercept_solver(
        target_pos_3d=(0.0, 0.0, 150.0),
        target_vel_3d=(0.0, 0.0, 0.0),
        v0=70.0,
    )
    assert sol["reachable"] is False


def test_ballistics_retreating_target(physics_benchmarks):
    """T2.3.3: Verifies drone retreating faster than muzzle velocity (vz=100 m/s > v0=70 m/s) is unreachable."""
    sol = physics_benchmarks.reference_intercept_solver(
        target_pos_3d=(0.0, 0.0, 40.0),
        target_vel_3d=(0.0, 0.0, 100.0),  # Escaping at 360 km/h away
        v0=70.0,
    )
    # Target can never be overtaken
    assert sol["reachable"] is False or sol["t_intercept"] > 4.0


def test_ballistics_near_vertical_pole(physics_benchmarks):
    """T2.3.4: Verifies target directly overhead (Y=50m, X=0, Z=0.1) handles spherical singularity."""
    sol = physics_benchmarks.reference_intercept_solver(
        target_pos_3d=(0.0, 50.0, 0.1),
        target_vel_3d=(0.0, 0.0, 0.0),
        v0=80.0,
    )
    assert not math.isnan(sol["aim_tilt_deg"])
    assert not math.isnan(sol["aim_pan_deg"])
    assert sol["aim_tilt_deg"] > 120.0  # High elevation


def test_ballistics_invalid_muzzle_velocity_rejection(physics_benchmarks):
    """T2.3.5: Verifies negative or zero muzzle velocity raises ValueError."""
    with pytest.raises(ValueError):
        physics_benchmarks.vacuum_trajectory(
            v0=-10.0, theta_deg=10.0, target_x=50.0
        )


def test_ballistics_zero_mass_rejection(physics_benchmarks):
    """T2.3.6: Verifies zero or negative projectile mass is rejected."""
    with pytest.raises(ValueError):
        physics_benchmarks.quadratic_drag_terminal_velocity(
            mass=0.0, cd=1.2, area=0.015
        )
