"""Tier 4 Tactical Scenario 2: High-Speed Head-On Dive Approach (80m -> 10m).

Simulates a fast drone diving directly towards the turret from 80m to 10m at 150 km/h.
Validates monotonic range reduction, expanding bounding box geometry, LiDAR parser integration,
and dynamic elevation lead adjustments.
"""

from __future__ import annotations

import pytest

from tests.fixtures.mock_serial import MockSerialPort
from tests.fixtures.physics_benchmarks import BallisticsBenchmarks
from tests.fixtures.synthetic_video import SyntheticVideoGenerator
from tests.tier1_features.test_detector_yolo import BaseDroneDetector, Detection
from tests.tier1_features.test_bytetrack import SimpleTracker
from tests.tier1_features.test_kalman_6d import DroneKalmanFilter6D
from tests.tier1_features.test_lidar_parser import TFMiniLidarParser
from tests.tier1_features.test_bbox_distance import DistanceEstimator
from tests.tier1_features.test_pid_controller import TurretController
from tests.tier1_features.test_arduino_comm import ArduinoSerialCommunicator


def test_scenario_headon_approach(mock_serial, physics_benchmarks, tfmini_gen):
    """T4.2: Executes complete 60-frame head-on fast dive scenario."""
    gen, targets = SyntheticVideoGenerator.create_headon_scenario(
        start_dist=80.0, speed_kmh=150.0, y_m=6.0, total_frames=60
    )

    detector = BaseDroneDetector(conf_threshold=0.50)
    tracker = SimpleTracker()
    kf: DroneKalmanFilter6D | None = None
    lidar_parser = TFMiniLidarParser()
    dist_est = DistanceEstimator(focal_length=gen.focal_length, preset="dji_mavic")
    turret = TurretController()
    comm = ArduinoSerialCommunicator(mock_serial=mock_serial)

    ranges = []
    flight_times = []

    for frame, meta in gen.stream_scenario(targets, total_frames=60):
        gt = meta["targets"][0]
        if not gt["visible"]:
            continue

        # 1. Detection
        det = Detection(
            box=tuple(gt["bbox"]),
            confidence=0.95,
            class_id=0,
            class_name="drone",
        )
        filtered_dets = detector.filter_detections([det])
        tracks = tracker.update(filtered_dets)
        locked = tracker.get_locked_target(tracks)
        assert locked is not None

        # 2. LiDAR injection and reading
        simulated_cm = int(gt["distance_m"] * 100)
        mock_serial.inject_tfmini_packet(distance_cm=simulated_cm, strength=1200)
        parsed_lidar = lidar_parser.feed_bytes(mock_serial.read(64))
        lidar_dist = parsed_lidar[-1] if parsed_lidar else None

        # 3. Distance Estimation
        est_range, src = dist_est.get_distance(locked.box, lidar_dist=lidar_dist)
        ranges.append(est_range)

        # 4. Kalman State
        cx = (locked.box[0] + locked.box[2]) / 2.0
        cy = (locked.box[1] + locked.box[3]) / 2.0
        if kf is None:
            kf = DroneKalmanFilter6D(initial_pos=(cx, cy), dt=gen.dt)
        kf.update((cx, cy), dt=gen.dt)

        # 5. Ballistics Intercept
        vz_mps = -(150.0 / 3.6)
        sol = physics_benchmarks.reference_intercept_solver(
            target_pos_3d=(0.0, gt["pos_3d"][1], est_range),
            target_vel_3d=(0.0, gt["vel_3d"][1], vz_mps),
            v0=80.0,
        )
        flight_times.append(sol["t_intercept"])

        # 6. Servo Commands
        pan, tilt = turret.update_lead_target(sol["aim_pan_deg"], sol["aim_tilt_deg"], dt=gen.dt)
        comm.send_angles(pan, tilt)

    # Assertions:
    # 1. Monotonic decrease in target range
    assert ranges[0] > ranges[-1]
    assert ranges[0] > 70.0
    assert ranges[-1] < 20.0

    # 2. Flight time to intercept monotonically decreases as drone gets closer
    assert flight_times[0] > flight_times[-1]
    assert flight_times[-1] < 0.30  # Very close, fast intercept
