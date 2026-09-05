"""Tier 4 Tactical Scenario 4: Foliage & Building Visual Occlusion Recovery (15 Frames).

Simulates a drone flying behind foliage/structures with 15 frames of total visual occlusion.
Validates Kalman coasting trajectory extrapolation, track retention, and seamless re-acquisition on exit.
"""

from __future__ import annotations

import math
import pytest

from tests.fixtures.mock_serial import MockSerialPort
from tests.fixtures.physics_benchmarks import BallisticsBenchmarks
from tests.fixtures.synthetic_video import SyntheticVideoGenerator
from tests.tier1_features.test_detector_yolo import BaseDroneDetector, Detection
from tests.tier1_features.test_bytetrack import SimpleTracker
from tests.tier1_features.test_kalman_6d import DroneKalmanFilter6D
from tests.tier1_features.test_pid_controller import TurretController
from tests.tier1_features.test_arduino_comm import ArduinoSerialCommunicator


def test_scenario_foliage_occlusion_recovery(mock_serial, physics_benchmarks):
    """T4.4: Executes 70-frame occlusion scenario with 15 frames of total visual loss (frames 25-40)."""
    gen, targets, occl_spans = SyntheticVideoGenerator.create_occlusion_scenario(
        speed_kmh=100.0,
        distance_m=45.0,
        occlusion_frames=(25, 40),
        total_frames=70,
    )

    detector = BaseDroneDetector(conf_threshold=0.50)
    tracker = SimpleTracker(max_age=25)  # Can survive up to 25 inactive frames
    kf: DroneKalmanFilter6D | None = None
    turret = TurretController()
    comm = ArduinoSerialCommunicator(mock_serial=mock_serial)

    coasting_flags = []
    track_ids_after_recovery = []

    for frame, meta in gen.stream_scenario(targets, total_frames=70, occlusion_spans=occl_spans):
        gt = meta["targets"][0]
        frame_id = meta["frame_id"]

        # 1. Detection
        if gt["visible"]:
            det = Detection(box=tuple(gt["bbox"]), confidence=0.88, class_id=0, class_name="drone")
            filtered = detector.filter_detections([det])
        else:
            filtered = []

        # 2. ByteTrack Update
        tracks = tracker.update(filtered)

        # 3. Kalman Update
        if tracks:
            locked = tracker.get_locked_target(tracks)
            if locked is not None:
                cx = (locked.box[0] + locked.box[2]) / 2.0
                cy = (locked.box[1] + locked.box[3]) / 2.0
                meas = (cx, cy)
                if frame_id > 40:
                    track_ids_after_recovery.append(locked.track_id)
            else:
                meas = None
        else:
            meas = None

        if kf is None and meas is not None:
            kf = DroneKalmanFilter6D(initial_pos=meas, dt=gen.dt)

        if kf is not None:
            state = kf.update(meas, dt=gen.dt)
            coasting_flags.append(state.is_coasting)

            # 4. Ballistics & PID
            pred_x = state.pos_2d[0]
            sol = physics_benchmarks.reference_intercept_solver(
                target_pos_3d=((pred_x - gen.cx) * 45.0 / gen.focal_length, 2.0, 45.0),
                target_vel_3d=(state.vel_2d[0] * 45.0 / gen.focal_length, 0.0, 0.0),
                v0=80.0,
            )
            pan, tilt = turret.update_lead_target(sol["aim_pan_deg"], sol["aim_tilt_deg"], dt=gen.dt)
            comm.send_angles(pan, tilt)

    # Assertions:
    # 1. During occlusion (frames 25 to 40), Kalman performed at least 15 coasting steps
    assert sum(coasting_flags) >= 15

    # 2. Re-acquisition established single persistent track ID after recovery
    assert len(track_ids_after_recovery) > 0
    assert len(set(track_ids_after_recovery)) == 1
