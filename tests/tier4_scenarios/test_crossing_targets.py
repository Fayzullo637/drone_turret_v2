"""Tier 4 Tactical Scenario 5: Dual-Drone Trajectory Crossing & Lock Retention.

Simulates two hostile drones crossing paths with overlapping bounding boxes in the field of view.
Validates that the tracking engine maintains continuous lock on the primary target without identity swapping.
"""

from __future__ import annotations

import pytest

from tests.fixtures.mock_serial import MockSerialPort
from tests.fixtures.physics_benchmarks import BallisticsBenchmarks
from tests.fixtures.synthetic_video import SyntheticVideoGenerator
from tests.tier1_features.test_detector_yolo import BaseDroneDetector, Detection
from tests.tier1_features.test_bytetrack import SimpleTracker
from tests.tier1_features.test_kalman_6d import DroneKalmanFilter6D
from tests.tier1_features.test_pid_controller import TurretController
from tests.tier1_features.test_arduino_comm import ArduinoSerialCommunicator


def test_scenario_crossing_targets(mock_serial, physics_benchmarks):
    """T4.5: Executes 60-frame dual-drone crossing scenario with overlapping bounding boxes."""
    gen, targets = SyntheticVideoGenerator.create_crossing_scenario(
        distance_m=50.0, total_frames=60
    )

    detector = BaseDroneDetector(conf_threshold=0.50)
    tracker = SimpleTracker()
    kf: DroneKalmanFilter6D | None = None
    turret = TurretController()
    comm = ArduinoSerialCommunicator(mock_serial=mock_serial)

    locked_target_ids = []

    for frame, meta in gen.stream_scenario(targets, total_frames=60):
        # Generate detections for both drones
        raw_dets = []
        for gt in meta["targets"]:
            if gt["visible"]:
                raw_dets.append(
                    Detection(
                        box=tuple(gt["bbox"]),
                        confidence=0.90,
                        class_id=0,
                        class_name="drone",
                    )
                )

        filtered = detector.filter_detections(raw_dets)
        tracks = tracker.update(filtered)

        # Lock on primary target (ID 1)
        if tracker.locked_track_id is None and tracks:
            tracker.lock_target(tracks[0].track_id)

        locked = tracker.get_locked_target(tracks)
        assert locked is not None
        locked_target_ids.append(locked.track_id)

        # Kalman & Steering on locked target
        cx = (locked.box[0] + locked.box[2]) / 2.0
        cy = (locked.box[1] + locked.box[3]) / 2.0
        if kf is None:
            kf = DroneKalmanFilter6D(initial_pos=(cx, cy), dt=gen.dt)
        state = kf.update((cx, cy), dt=gen.dt)

        sol = physics_benchmarks.reference_intercept_solver(
            target_pos_3d=((cx - gen.cx) * 50.0 / gen.focal_length, 0.0, 50.0),
            target_vel_3d=(state.vel_2d[0] * 50.0 / gen.focal_length, 0.0, 0.0),
            v0=80.0,
        )
        pan, tilt = turret.update_lead_target(sol["aim_pan_deg"], sol["aim_tilt_deg"], dt=gen.dt)
        comm.send_angles(pan, tilt)

    # Assertions:
    # 1. Locked target ID remained strictly ID 1 across the entire 60-frame run
    assert len(locked_target_ids) == 60
    assert all(tid == 1 for tid in locked_target_ids)
    assert len(set(locked_target_ids)) == 1
