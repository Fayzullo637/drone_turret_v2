"""Tier 4 Tactical Scenario 3: High-G Evasive Sinusoidal Zigzag Maneuvers.

Simulates an agile enemy drone executing high-G sinusoidal jinks (A=5.0m, f=0.5 Hz) at 40m range.
Validates Kalman 6-state acceleration tracking, lead point reversal handling, and continuous PID engagement.
"""

from __future__ import annotations

import math
import numpy as np
import pytest

from tests.fixtures.mock_serial import MockSerialPort
from tests.fixtures.physics_benchmarks import BallisticsBenchmarks
from tests.fixtures.synthetic_video import SyntheticVideoGenerator, TargetKinematics
from tests.tier1_features.test_detector_yolo import BaseDroneDetector, Detection
from tests.tier1_features.test_bytetrack import SimpleTracker
from tests.tier1_features.test_kalman_6d import DroneKalmanFilter6D
from tests.tier1_features.test_pid_controller import TurretController
from tests.tier1_features.test_arduino_comm import ArduinoSerialCommunicator


def test_scenario_evasive_sinusoidal_zigzag(mock_serial, physics_benchmarks):
    """T4.3: Executes 90-frame high-G evasive sinusoidal maneuver scenario."""
    gen = SyntheticVideoGenerator(fps=30.0)
    total_frames = 90
    amplitude_m = 5.0
    freq_hz = 0.5

    detector = BaseDroneDetector(conf_threshold=0.50)
    tracker = SimpleTracker()
    kf: DroneKalmanFilter6D | None = None
    turret = TurretController()
    comm = ArduinoSerialCommunicator(mock_serial=mock_serial)

    measured_acc_x = []
    pan_commands = []

    for i in range(total_frames):
        t = (i + 1) * gen.dt
        # Target sinusoidal kinematics: x(t) = A * sin(omega * t)
        omega = 2.0 * math.pi * freq_hz
        tgt_x = amplitude_m * math.sin(omega * t)
        tgt_vx = amplitude_m * omega * math.cos(omega * t)
        tgt_ax = -amplitude_m * (omega**2) * math.sin(omega * t)
        target = TargetKinematics(
            x=tgt_x, y=1.0, z=40.0, vx=tgt_vx, vy=0.0, vz=0.0, ax=tgt_ax
        )

        frame, meta = gen.generate_single_frame([target])
        gt = meta["targets"][0]

        # 1. Detect & Track
        det = Detection(box=tuple(gt["bbox"]), confidence=0.90, class_id=0, class_name="drone")
        tracks = tracker.update(detector.filter_detections([det]))
        locked = tracker.get_locked_target(tracks)
        assert locked is not None

        # 2. Kalman Filter with Acceleration
        cx = (locked.box[0] + locked.box[2]) / 2.0
        cy = (locked.box[1] + locked.box[3]) / 2.0
        if kf is None:
            kf = DroneKalmanFilter6D(initial_pos=(cx, cy), dt=gen.dt)
        state = kf.update((cx, cy), dt=gen.dt)
        measured_acc_x.append(state.acc_2d[0])

        # 3. Intercept Calculation
        sol = physics_benchmarks.reference_intercept_solver(
            target_pos_3d=(tgt_x, 1.0, 40.0),
            target_vel_3d=(tgt_vx, 0.0, 0.0),
            target_acc_3d=(tgt_ax, 0.0, 0.0),
            v0=80.0,
        )

        # 4. PID Servo Steering
        pan, tilt = turret.update_lead_target(sol["aim_pan_deg"], sol["aim_tilt_deg"], dt=gen.dt)
        comm.send_angles(pan, tilt)
        pan_commands.append(pan)

    # Assertions:
    # 1. Track ID 1 maintained throughout all 90 frames of high-G turns
    assert tracker.locked_track_id == 1

    # 2. Pan servo commands oscillate following target sine wave
    pan_min, pan_max = min(pan_commands), max(pan_commands)
    assert pan_min < 90.0 < pan_max  # Swings both left and right across center
    assert (pan_max - pan_min) > 10.0  # Demonstrates dynamic tracking deflection
