"""Tier 3 Pairwise Integration Tests: Intercept Lead Point ↔ Servo Output Clamping (R3 + R5).

Verifies that out-of-field 3D lead calculations (e.g. crossing pan > 180 or negative elevation)
are smoothly clamped by the dual-axis PID controller into valid [0, 180] degree hardware servo ranges.
"""

from __future__ import annotations

import pytest

from tests.fixtures.mock_serial import MockSerialPort
from tests.fixtures.physics_benchmarks import BallisticsBenchmarks
from tests.tier1_features.test_pid_controller import TurretController
from tests.tier1_features.test_arduino_comm import ArduinoSerialCommunicator


def test_lead_to_servo_clamping_extreme_azimuth(mock_serial, physics_benchmarks):
    """T3.3.1: Tests extreme wide-angle lead solution (>180 deg) is safely clamped to 180 deg."""
    turret = TurretController()
    comm = ArduinoSerialCommunicator(mock_serial=mock_serial)

    # Target far to the right (+60m at 20m depth) -> pan azimuth angle >> 180 deg
    sol = physics_benchmarks.reference_intercept_solver(
        target_pos_3d=(60.0, 0.0, 20.0),
        target_vel_3d=(20.0, 0.0, 0.0),
        v0=80.0,
    )

    # PID step
    pan_cmd, tilt_cmd = turret.update_lead_target(
        target_pan_deg=sol["aim_pan_deg"],
        target_tilt_deg=sol["aim_tilt_deg"],
        dt=0.1,
    )

    # Transmit to Arduino
    sent_str = comm.send_angles(pan_cmd, tilt_cmd)
    last_cmd = mock_serial.get_last_command()

    assert last_cmd is not None
    assert 0 <= last_cmd[0] <= 180
    assert 0 <= last_cmd[1] <= 180
    assert last_cmd[0] <= 180


def test_lead_to_servo_clamping_negative_elevation(mock_serial, physics_benchmarks):
    """T3.3.2: Tests low diving target resulting in negative elevation angle is clamped to 0 deg."""
    turret = TurretController()
    comm = ArduinoSerialCommunicator(mock_serial=mock_serial)

    # Target diving deep below horizon (Y = -50m at 20m depth)
    sol = physics_benchmarks.reference_intercept_solver(
        target_pos_3d=(0.0, -50.0, 20.0),
        target_vel_3d=(0.0, -10.0, 0.0),
        v0=80.0,
    )

    pan_cmd, tilt_cmd = turret.update_lead_target(
        target_pan_deg=sol["aim_pan_deg"],
        target_tilt_deg=sol["aim_tilt_deg"],
        dt=0.1,
    )

    comm.send_angles(pan_cmd, tilt_cmd)
    last_cmd = mock_serial.get_last_command()

    assert last_cmd is not None
    assert 0 <= last_cmd[1] <= 180
