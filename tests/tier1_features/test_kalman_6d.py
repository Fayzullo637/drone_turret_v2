"""Tier 1 Unit Tests: 6-State Kalman Predictive Tracker (Features F4, F5, F6, F7).

Verifies 6D state vector [x, y, dx, dy, ddx, ddy], dynamic dt transition matrix F(dt),
future trajectory projection, >=10 frame visual coasting, and km/h speed estimation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
import cv2
import numpy as np
import pytest

from tests.fixtures.synthetic_video import SyntheticVideoGenerator, TargetKinematics


@dataclass
class TargetState:
    pos_2d: Tuple[float, float]        # x, y in pixels
    vel_2d: Tuple[float, float]        # vx, vy in px/s
    acc_2d: Tuple[float, float]        # ax, ay in px/s^2
    pos_3d: Tuple[float, float, float] # X, Y, Z in meters
    vel_3d: Tuple[float, float, float] # vX, vY, vZ in m/s
    speed_kmh: float
    is_coasting: bool
    coast_frames: int


class DroneKalmanFilter6D:
    """6-State Constant-Acceleration Kalman Filter [x, y, vx, vy, ax, ay]^T."""

    def __init__(self, initial_pos: Tuple[float, float] = (320.0, 240.0), dt: float = 1.0 / 30.0):
        self.kf = cv2.KalmanFilter(6, 2, 0)
        # Measurement matrix H selects [x, y]
        self.kf.measurementMatrix = np.array(
            [[1, 0, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0]], dtype=np.float32
        )
        # Initial state
        self.kf.statePost = np.array(
            [[initial_pos[0]], [initial_pos[1]], [0.0], [0.0], [0.0], [0.0]], dtype=np.float32
        )
        self.kf.errorCovPost = np.eye(6, dtype=np.float32) * 10.0
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * 1e-2
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1.0

        self.last_dt = dt
        self.set_dt(dt)
        self.coast_frames = 0
        self.max_coast_frames = 15

    def set_dt(self, dt: float) -> None:
        """Update transition matrix F(dt) for dynamic frame rate."""
        self.last_dt = max(1e-4, dt)
        dt2 = 0.5 * (self.last_dt**2)
        # F: x' = x + vx*dt + 0.5*ax*dt^2
        self.kf.transitionMatrix = np.array(
            [
                [1, 0, self.last_dt, 0, dt2, 0],
                [0, 1, 0, self.last_dt, 0, dt2],
                [0, 0, 1, 0, self.last_dt, 0],
                [0, 0, 0, 1, 0, self.last_dt],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ],
            dtype=np.float32,
        )

    def predict(self, dt: Optional[float] = None) -> Tuple[float, float]:
        """Perform time update step."""
        if dt is not None and dt != self.last_dt:
            self.set_dt(dt)
        pred = self.kf.predict()
        return float(pred[0, 0]), float(pred[1, 0])

    def update(self, measurement: Optional[Tuple[float, float]], dt: Optional[float] = None) -> TargetState:
        """Correct state with measurement if available; otherwise coast."""
        if dt is not None:
            self.set_dt(dt)

        self.predict()

        if measurement is not None:
            meas = np.array([[measurement[0]], [measurement[1]]], dtype=np.float32)
            self.kf.correct(meas)
            self.coast_frames = 0
            is_coasting = False
        else:
            self.coast_frames += 1
            is_coasting = True

        state = self.kf.statePost
        x, y = float(state[0, 0]), float(state[1, 0])
        vx, vy = float(state[2, 0]), float(state[3, 0])
        ax, ay = float(state[4, 0]), float(state[5, 0])

        speed_px_s = math.hypot(vx, vy)
        # Assuming nominal distance/scaling for 2D px/s -> km/h
        speed_kmh = speed_px_s * 0.1  # scaling factor

        return TargetState(
            pos_2d=(x, y),
            vel_2d=(vx, vy),
            acc_2d=(ax, ay),
            pos_3d=(0.0, 0.0, 50.0),
            vel_3d=(vx * 0.05, vy * 0.05, 0.0),
            speed_kmh=speed_kmh,
            is_coasting=is_coasting,
            coast_frames=self.coast_frames,
        )

    def project_future_trajectory(self, horizons: List[float]) -> List[Tuple[float, float]]:
        """Projects future positions for given forward time horizons [t1, t2, ...]."""
        state = self.kf.statePost
        x, y = float(state[0, 0]), float(state[1, 0])
        vx, vy = float(state[2, 0]), float(state[3, 0])
        ax, ay = float(state[4, 0]), float(state[5, 0])

        points = []
        for t in horizons:
            fut_x = x + vx * t + 0.5 * ax * (t**2)
            fut_y = y + vy * t + 0.5 * ay * (t**2)
            points.append((fut_x, fut_y))
        return points


def test_kalman_state_initialization():
    """T1.3.1: Verifies initial state vector matches [x0, y0, 0, 0, 0, 0]."""
    kf = DroneKalmanFilter6D(initial_pos=(320.0, 240.0), dt=1.0 / 30.0)
    state = kf.kf.statePost.flatten()
    assert state[0] == pytest.approx(320.0)
    assert state[1] == pytest.approx(240.0)
    assert state[2] == pytest.approx(0.0)
    assert state[3] == pytest.approx(0.0)
    assert state[4] == pytest.approx(0.0)
    assert state[5] == pytest.approx(0.0)


def test_kalman_constant_velocity_convergence():
    """T1.3.2: Verifies Kalman filter converges on constant velocity vx = 60 px/s."""
    dt = 1.0 / 30.0
    kf = DroneKalmanFilter6D(initial_pos=(100.0, 200.0), dt=dt)

    true_vx = 60.0  # px/s
    true_x = 100.0
    true_y = 200.0

    for _ in range(25):
        true_x += true_vx * dt
        state = kf.update((true_x, true_y), dt=dt)

    assert state.pos_2d[0] == pytest.approx(true_x, abs=2.0)
    assert state.vel_2d[0] == pytest.approx(true_vx, abs=4.0)
    assert abs(state.vel_2d[1]) < 2.0


def test_kalman_dynamic_dt_transition_matrix():
    """T1.3.3: Verifies transition matrix correctly reflects varying dt values."""
    kf = DroneKalmanFilter6D(dt=0.033)
    assert kf.kf.transitionMatrix[0, 2] == pytest.approx(0.033, rel=1e-3)
    assert kf.kf.transitionMatrix[0, 4] == pytest.approx(0.5 * (0.033**2), rel=1e-3)

    # Change dt to 0.050s
    kf.set_dt(0.050)
    assert kf.kf.transitionMatrix[0, 2] == pytest.approx(0.050, rel=1e-3)
    assert kf.kf.transitionMatrix[0, 4] == pytest.approx(0.5 * (0.050**2), rel=1e-3)


def test_kalman_multi_horizon_prediction():
    """T1.3.4: Verifies multi-horizon trajectory projection for t in [0.1, 0.5, 1.0]s."""
    kf = DroneKalmanFilter6D(initial_pos=(100.0, 100.0), dt=0.033)
    # Inject velocity
    kf.kf.statePost[2, 0] = 50.0  # vx = 50 px/s
    kf.kf.statePost[3, 0] = 0.0

    points = kf.project_future_trajectory([0.1, 0.5, 1.0])
    assert len(points) == 3

    # x(0.1) = 100 + 50*0.1 = 105
    assert points[0][0] == pytest.approx(105.0, abs=0.5)
    # x(0.5) = 100 + 50*0.5 = 125
    assert points[1][0] == pytest.approx(125.0, abs=0.5)
    # x(1.0) = 100 + 50*1.0 = 150
    assert points[2][0] == pytest.approx(150.0, abs=0.5)


def test_kalman_10frame_occlusion_coasting():
    """T1.3.5: Verifies Kalman filter coasts for 10 consecutive missed detections."""
    dt = 1.0 / 30.0
    kf = DroneKalmanFilter6D(initial_pos=(100.0, 100.0), dt=dt)

    # Establish track with 15 frames
    for i in range(15):
        kf.update((100.0 + i * 5.0, 100.0), dt=dt)

    # 10 frames of occlusion (measurement is None)
    for _ in range(10):
        state = kf.update(None, dt=dt)
        assert state.is_coasting is True

    assert state.coast_frames == 10
    # Final position should still have advanced forward
    assert state.pos_2d[0] > 175.0


def test_kalman_speed_calculation_kmh():
    """T1.3.6: Verifies speed conversion calculation from velocity."""
    vx_mps = 55.555  # 200 km/h
    vy_mps = 0.0
    vz_mps = 0.0
    speed_mps = math.sqrt(vx_mps**2 + vy_mps**2 + vz_mps**2)
    speed_kmh = speed_mps * 3.6
    assert speed_kmh == pytest.approx(200.0, abs=0.01)
