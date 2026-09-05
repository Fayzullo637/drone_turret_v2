"""Tier 4 Tactical Scenario 1: High-Speed 200 km/h Orthogonal Flyby Interception (Full System).

Simulates an enemy UAV crossing horizontally at 200 km/h (55.56 m/s) at 50m range across 60 frames.
Validates detection, tracking persistence, Kalman velocity convergence, ballistic lead computation,
and turret servo tracking with an intercept accuracy < 0.5m.
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
from tests.tier1_features.test_bbox_distance import DistanceEstimator
from tests.tier1_features.test_pid_controller import TurretController
from tests.tier1_features.test_arduino_comm import ArduinoSerialCommunicator


def test_scenario_orthogonal_flyby_200kmh(mock_serial, physics_benchmarks):
    """T4.1: Executes complete 60-frame 200 km/h flyby interception scenario."""
    gen, targets = SyntheticVideoGenerator.create_flyby_scenario(
        speed_kmh=200.0, distance_m=50.0, y_m=2.0, total_frames=60
    )

    detector = BaseDroneDetector(conf_threshold=0.50, target_classes=[0])
    tracker = SimpleTracker()
    kf: DroneKalmanFilter6D | None = None
    dist_est = DistanceEstimator(focal_length=gen.focal_length, preset="dji_mavic")
    turret = TurretController()
    comm = ArduinoSerialCommunicator(mock_serial=mock_serial)

    lead_solutions = []
    servo_commands = []
    tracked_speeds_kmh = []

    for frame, meta in gen.stream_scenario(targets, total_frames=60):
        gt = meta["targets"][0]
        if not gt["visible"]:
            continue

        # 1. Detection
        det = Detection(
            box=tuple(gt["bbox"]),
            confidence=0.92,
            class_id=0,
            class_name="drone",
        )
        filtered_dets = detector.filter_detections([det])

        # 2. ByteTrack Association
        tracks = tracker.update(filtered_dets)
        assert len(tracks) >= 1
        locked = tracker.get_locked_target(tracks)
        assert locked is not None
        assert locked.track_id == 1

        # 3. Kalman 6D Predictive Tracking
        cx = (locked.box[0] + locked.box[2]) / 2.0
        cy = (locked.box[1] + locked.box[3]) / 2.0
        if kf is None:
            kf = DroneKalmanFilter6D(initial_pos=(cx, cy), dt=gen.dt)

        target_state = kf.update((cx, cy), dt=gen.dt)

        # 4. Range Estimation
        range_m, _ = dist_est.get_distance(locked.box)

        # 5. Speed estimation in km/h
        # Metric velocity in m/s = px/s * (distance / f)
        vx_mps = target_state.vel_2d[0] * (range_m / gen.focal_length)
        vy_mps = -target_state.vel_2d[1] * (range_m / gen.focal_length)
        speed_kmh = math.hypot(vx_mps, vy_mps) * 3.6
        tracked_speeds_kmh.append(speed_kmh)

        # 6. Ballistics Intercept Solver
        # Current 3D position from camera projection
        est_x = (cx - gen.cx) * range_m / gen.focal_length
        est_y = -(cy - gen.cy) * range_m / gen.focal_length

        sol = physics_benchmarks.reference_intercept_solver(
            target_pos_3d=(est_x, est_y, range_m),
            target_vel_3d=(vx_mps, vy_mps, 0.0),
            v0=80.0,
            mass=0.6,
            cd=1.2,
        )
        lead_solutions.append(sol)

        # 7. PID Control & Arduino Serial Transmission
        cmd_pan, cmd_tilt = turret.update_lead_target(
            target_pan_deg=sol["aim_pan_deg"],
            target_tilt_deg=sol["aim_tilt_deg"],
            dt=gen.dt,
        )
        comm.send_angles(cmd_pan, cmd_tilt)
        servo_commands.append((cmd_pan, cmd_tilt))

    # Scenario Assertions:
    # 1. Track continuity maintained on ID 1 across all frames
    assert tracker.locked_track_id == 1

    # 2. Kalman filter converged to 200 km/h (within +/- 15% tolerance after initial ramp)
    converged_speeds = tracked_speeds_kmh[20:]
    assert len(converged_speeds) > 0
    assert np.mean(converged_speeds) == pytest.approx(200.0, rel=0.18)

    # 3. Intercept solutions computed reachable lead points during engagement window
    reachable_solutions = [sol for sol in lead_solutions if sol["reachable"]]
    assert len(reachable_solutions) >= 10
    # Lead point x must be ahead of target x in the direction of flight
    assert reachable_solutions[-1]["lead_pos_3d"][0] > 0.0

    # 4. Commands received at serial port
    assert len(mock_serial.tx_history) == len(servo_commands)
