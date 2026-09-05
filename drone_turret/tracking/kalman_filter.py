"""Constant Acceleration (CA) 6-State Kalman Predictive Tracker.

Milestone M2 Implementation for drone_turret_v2:
- 6-State Constant Acceleration (CA) model using cv2.KalmanFilter
  State vector: [x, y, dx, dy, ddx, ddy]^T (position, velocity, acceleration in pixels)
  Measurement vector: [z_x, z_y]^T (target centroid in pixels)
- Dynamic dt handling per frame for F(dt) and Q(dt)
- Forward multi-horizon trajectory projection (0.1 - 2.0s)
- Occlusion & detection loss coasting (>= 10-15 frames before LOST)
- 3D kinematics and speed calculation in km/h via camera pinhole geometry
- Comprehensive state dataclasses and multi-target tracking manager
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple, Union

import cv2
import numpy as np


class FilterStatus(str, Enum):
    """Lifecycle status of the Kalman tracking filter."""
    UNINITIALIZED = "UNINITIALIZED"
    TRACKING = "TRACKING"
    COASTING = "COASTING"
    LOST = "LOST"


@dataclass
class KalmanConfig:
    """Configuration parameters for the 6-state Kalman Predictive Tracker."""
    process_noise_scale: float = 10.0      # q maneuver variance parameter (px^2/s^5)
    measurement_noise_std: float = 3.0     # sigma measurement noise in pixels (R = sigma^2)
    initial_cov_pos: float = 10.0          # Initial position variance
    initial_cov_vel: float = 1000.0        # Initial velocity variance
    initial_cov_acc: float = 10000.0       # Initial acceleration variance
    max_coast_frames: int = 15             # Max consecutive missed frames before declaring LOST
    nominal_dt: float = 1.0 / 30.0         # Fallback frame interval (30 FPS)
    min_dt: float = 1e-4                   # Minimum clamped dt in seconds
    max_dt: float = 1.0                    # Maximum clamped dt in seconds
    camera_hfov: float = 70.0              # Camera horizontal field of view in degrees
    camera_width: int = 640                # Default image width in pixels
    camera_height: int = 480               # Default image height in pixels
    default_distance: float = 30.0         # Default range in meters when unmeasured
    known_drone_size: float = 0.35         # Known physical size in meters (e.g. DJI Mavic)
    innovation_gate_sigma: float = 0.0     # Innovation gating threshold in sigmas (0.0 = disabled)


# Type alias for configuration
FilterConfig = KalmanConfig


@dataclass
class TargetState:
    """Complete 2D/3D kinematic state of a tracked target.
    
    Adheres strictly to the PROJECT.md Tracking <-> Ballistics interface contract.
    """
    pos_2d: Tuple[float, float]        # x, y in pixels
    vel_2d: Tuple[float, float]        # vx, vy in px/s
    acc_2d: Tuple[float, float]        # ax, ay in px/s^2
    pos_3d: Tuple[float, float, float] # X, Y, Z in meters (turret-centric)
    vel_3d: Tuple[float, float, float] # vX, vY, vZ in m/s
    speed_kmh: float                   # Speed magnitude in km/h
    is_coasting: bool                  # True if currently predicting without detection
    coast_frames: int                  # Number of consecutive coasting frames
    acc_3d: Optional[Tuple[float, float, float]] = None # aX, aY, aZ in m/s^2
    
    # Extended telemetry and status metadata
    status: FilterStatus = FilterStatus.UNINITIALIZED
    trajectory_2d: List[Tuple[float, float]] = field(default_factory=list)
    trajectory_3d: List[Tuple[float, float, float]] = field(default_factory=list)
    confidence: float = 1.0
    timestamp: float = 0.0
    track_id: Optional[int] = None
    bbox: Optional[Tuple[int, int, int, int]] = None
    pos_uncertainty_px: float = 0.0
    vel_uncertainty_px: float = 0.0

    @property
    def x(self) -> float:
        """2D pixel X position."""
        return self.pos_2d[0]

    @property
    def y(self) -> float:
        """2D pixel Y position."""
        return self.pos_2d[1]

    @property
    def vx(self) -> float:
        """2D pixel X velocity (px/s)."""
        return self.vel_2d[0]

    @property
    def vy(self) -> float:
        """2D pixel Y velocity (px/s)."""
        return self.vel_2d[1]

    @property
    def ax(self) -> float:
        """2D pixel X acceleration (px/s^2)."""
        return self.acc_2d[0]

    @property
    def ay(self) -> float:
        """2D pixel Y acceleration (px/s^2)."""
        return self.acc_2d[1]

    @property
    def X(self) -> float:
        """3D world X position in meters (turret-centric, +X right)."""
        return self.pos_3d[0]

    @property
    def Y(self) -> float:
        """3D world Y position in meters (turret-centric, +Y up)."""
        return self.pos_3d[1]

    @property
    def Z(self) -> float:
        """3D world Z position / depth in meters (turret-centric, +Z downrange)."""
        return self.pos_3d[2]

    @property
    def vX(self) -> float:
        """3D world X velocity in m/s."""
        return self.vel_3d[0]

    @property
    def vY(self) -> float:
        """3D world Y velocity in m/s."""
        return self.vel_3d[1]

    @property
    def vZ(self) -> float:
        """3D world Z velocity in m/s."""
        return self.vel_3d[2]

    @property
    def speed_mps(self) -> float:
        """3D velocity magnitude in meters per second."""
        return self.speed_kmh / 3.6

    @property
    def speed_px_s(self) -> float:
        """2D pixel speed magnitude in pixels per second."""
        return math.sqrt(self.vel_2d[0]**2 + self.vel_2d[1]**2)

    @property
    def is_tracking(self) -> bool:
        """True if filter is actively tracking (direct measurement or valid coasting)."""
        return self.status in (FilterStatus.TRACKING, FilterStatus.COASTING)

    @property
    def is_lost(self) -> bool:
        """True if target is declared lost or uninitialized."""
        return self.status in (FilterStatus.LOST, FilterStatus.UNINITIALIZED)


def focal_length_from_hfov(width_px: float, hfov_deg: float) -> float:
    """Computes camera focal length in pixels from image width and horizontal FOV."""
    hfov_rad = math.radians(hfov_deg)
    return float(width_px / (2.0 * math.tan(hfov_rad / 2.0)))


def compute_transition_matrix(dt: float) -> np.ndarray:
    """Constructs 6x6 Constant Acceleration (CA) state transition matrix F(dt).
    
    State vector: [x, y, dx, dy, ddx, ddy]^T
    Equations:
      x(t+dt)  = x + dx*dt + 0.5*ddx*dt^2
      y(t+dt)  = y + dy*dt + 0.5*ddy*dt^2
      dx(t+dt) = dx + ddx*dt
      dy(t+dt) = dy + ddy*dt
      ddx(t+dt)= ddx
      ddy(t+dt)= ddy
    """
    F = np.eye(6, dtype=np.float32)
    dt = float(dt)
    dt2_half = 0.5 * dt * dt
    
    # x channel
    F[0, 2] = dt
    F[0, 4] = dt2_half
    F[2, 4] = dt

    # y channel
    F[1, 3] = dt
    F[1, 5] = dt2_half
    F[3, 5] = dt

    return F


def compute_process_noise(dt: float, q: float = 10.0) -> np.ndarray:
    """Constructs 6x6 Continuous White Noise Acceleration (CWNA) covariance matrix Q(dt).
    
    Parameters:
      dt: Frame interval in seconds.
      q: Continuous process noise intensity (px^2/s^5).
    """
    Q = np.zeros((6, 6), dtype=np.float32)
    dt = float(dt)
    dt2 = dt * dt
    dt3 = dt2 * dt
    dt4 = dt3 * dt
    dt5 = dt4 * dt

    q00 = q * (dt5 / 20.0)
    q02 = q * (dt4 / 8.0)
    q04 = q * (dt3 / 6.0)
    q22 = q * (dt3 / 3.0)
    q24 = q * (dt2 / 2.0)
    q44 = q * dt

    for offset in (0, 1):
        # Position-Position
        Q[0 + offset, 0 + offset] = q00
        # Position-Velocity
        Q[0 + offset, 2 + offset] = q02
        Q[2 + offset, 0 + offset] = q02
        # Position-Acceleration
        Q[0 + offset, 4 + offset] = q04
        Q[4 + offset, 0 + offset] = q04
        # Velocity-Velocity
        Q[2 + offset, 2 + offset] = q22
        # Velocity-Acceleration
        Q[2 + offset, 4 + offset] = q24
        Q[4 + offset, 2 + offset] = q24
        # Acceleration-Acceleration
        Q[4 + offset, 4 + offset] = q44

    return Q


def pixel_to_camera_3d(
    px: float,
    py: float,
    distance_m: float,
    width_px: float = 640.0,
    height_px: float = 480.0,
    focal_length_px: Optional[float] = None,
    hfov_deg: float = 70.0,
) -> Tuple[float, float, float]:
    """Converts 2D pixel coordinates and distance to 3D turret-centric Cartesian coordinates (X, Y, Z).
    
    Convention:
    - +X: Right (meters)
    - +Y: Up (meters)
    - +Z: Range / Forward downrange (meters)
    """
    if focal_length_px is None or focal_length_px <= 0:
        focal_length_px = focal_length_from_hfov(width_px, hfov_deg)

    cx = width_px / 2.0
    cy = height_px / 2.0
    safe_z = max(0.01, float(distance_m))

    # Pinhole projection inversion
    X = float((px - cx) * safe_z / focal_length_px)
    Y = float(-(py - cy) * safe_z / focal_length_px)  # Invert Y so up is positive
    Z = float(safe_z)
    return (X, Y, Z)


def velocity_to_speed_kmh(vx: float, vy: float, vz: float = 0.0) -> float:
    """Converts 3D velocity vector (m/s) into scalar speed in km/h."""
    v_mag = math.sqrt(vx**2 + vy**2 + vz**2)
    return float(v_mag * 3.6)


class KalmanPredictiveTracker:
    """High-performance 6-state Constant Acceleration (CA) Kalman Filter.
    
    Maintains continuous state estimation [x, y, dx, dy, ddx, ddy] with dynamic dt,
    occlusion coasting, forward trajectory prediction, and 3D velocity/speed reconstruction.
    """

    def __init__(self, config: Optional[KalmanConfig] = None):
        self.config = config or KalmanConfig()
        
        # Instantiate OpenCV Kalman Filter (6 dynamic params, 2 measurement params, 0 control)
        self.kf = cv2.KalmanFilter(6, 2, 0)
        
        # Measurement matrix H (2x6): extracts (x, y) from [x, y, dx, dy, ddx, ddy]
        H = np.zeros((2, 6), dtype=np.float32)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        self.kf.measurementMatrix = H
        
        # Measurement noise covariance R (2x2)
        r_var = float(self.config.measurement_noise_std ** 2)
        self.kf.measurementNoiseCov = (np.eye(2, dtype=np.float32) * r_var)
        
        # Internal state tracking
        self.status = FilterStatus.UNINITIALIZED
        self.coast_frames = 0
        self.last_timestamp: Optional[float] = None
        self.last_distance: float = self.config.default_distance
        self.last_distance_time: Optional[float] = None
        self.distance_rate: float = 0.0
        self.track_id: Optional[int] = None
        self.last_bbox: Optional[Tuple[int, int, int, int]] = None
        self.confidence: float = 1.0
        self.total_updates: int = 0

        self.reset()

    def reset(self) -> None:
        """Resets the Kalman Filter state, error covariance, and counters to uninitialized."""
        self.status = FilterStatus.UNINITIALIZED
        self.coast_frames = 0
        self.last_timestamp = None
        self.last_distance = self.config.default_distance
        self.last_distance_time = None
        self.distance_rate = 0.0
        self.track_id = None
        self.last_bbox = None
        self.confidence = 1.0
        self.total_updates = 0

        # Reset states to zeros
        self.kf.statePre = np.zeros((6, 1), dtype=np.float32)
        self.kf.statePost = np.zeros((6, 1), dtype=np.float32)

        # Initialize initial error covariance P0
        P0 = np.diag([
            self.config.initial_cov_pos,
            self.config.initial_cov_pos,
            self.config.initial_cov_vel,
            self.config.initial_cov_vel,
            self.config.initial_cov_acc,
            self.config.initial_cov_acc,
        ]).astype(np.float32)
        self.kf.errorCovPre = P0.copy()
        self.kf.errorCovPost = P0.copy()

    def initialize(
        self,
        initial_pos: Tuple[float, float],
        initial_vel: Tuple[float, float] = (0.0, 0.0),
        initial_acc: Tuple[float, float] = (0.0, 0.0),
        timestamp: Optional[float] = None,
        distance: Optional[float] = None,
        track_id: Optional[int] = None,
        bbox: Optional[Tuple[int, int, int, int]] = None,
    ) -> TargetState:
        """Explicitly initializes the filter state with given position, velocity, and acceleration."""
        self.reset()
        x0, y0 = float(initial_pos[0]), float(initial_pos[1])
        vx0, vy0 = float(initial_vel[0]), float(initial_vel[1])
        ax0, ay0 = float(initial_acc[0]), float(initial_acc[1])

        state = np.array([[x0], [y0], [vx0], [vy0], [ax0], [ay0]], dtype=np.float32)
        self.kf.statePre = state.copy()
        self.kf.statePost = state.copy()

        self.status = FilterStatus.TRACKING
        self.coast_frames = 0
        self.last_timestamp = timestamp
        if distance is not None and distance > 0:
            self.last_distance = float(distance)
        self.last_distance_time = timestamp
        self.track_id = track_id
        self.last_bbox = bbox
        self.total_updates = 1

        return self.get_state()

    def _sanitize_dt(self, dt: Optional[float], timestamp: Optional[float]) -> float:
        """Determines and sanitizes the time elapsed delta dt in seconds."""
        if dt is not None and dt > 0:
            calc_dt = float(dt)
        elif timestamp is not None and self.last_timestamp is not None:
            calc_dt = float(timestamp - self.last_timestamp)
        else:
            calc_dt = self.config.nominal_dt

        # Clamp dt to avoid numerical singularities or huge jumps during pauses
        return max(self.config.min_dt, min(self.config.max_dt, calc_dt))

    def _update_matrices_for_dt(self, dt: float) -> None:
        """Recalculates and updates transition matrix F(dt) and process noise covariance Q(dt)."""
        F = compute_transition_matrix(dt)
        Q = compute_process_noise(dt, q=self.config.process_noise_scale)
        self.kf.transitionMatrix = F
        self.kf.processNoiseCov = Q

    def predict(self, dt: Optional[float] = None, timestamp: Optional[float] = None) -> TargetState:
        """Performs a Kalman prediction step forward in time by dt seconds."""
        if self.status == FilterStatus.UNINITIALIZED:
            return self.get_state()

        step_dt = self._sanitize_dt(dt, timestamp)
        self._update_matrices_for_dt(step_dt)

        self.kf.predict()

        if timestamp is not None:
            self.last_timestamp = timestamp
        elif self.last_timestamp is not None:
            self.last_timestamp += step_dt

        return self.get_state()

    def correct(
        self,
        measurement: Tuple[float, float],
        distance: Optional[float] = None,
        bbox: Optional[Tuple[int, int, int, int]] = None,
        confidence: float = 1.0,
    ) -> TargetState:
        """Performs a Kalman measurement correction step with observed (z_x, z_y)."""
        zx, zy = float(measurement[0]), float(measurement[1])
        meas_arr = np.array([[zx], [zy]], dtype=np.float32)

        # Innovation gating / outlier rejection if enabled
        if self.config.innovation_gate_sigma > 0 and self.status == FilterStatus.TRACKING:
            pred_x = float(self.kf.statePre[0, 0])
            pred_y = float(self.kf.statePre[1, 0])
            res_x = zx - pred_x
            res_y = zy - pred_y
            
            # Innovation covariance S = H P H^T + R
            P = self.kf.errorCovPre
            var_x = P[0, 0] + self.config.measurement_noise_std**2
            var_y = P[1, 1] + self.config.measurement_noise_std**2
            std_dist = math.sqrt((res_x**2 / max(1e-4, var_x)) + (res_y**2 / max(1e-4, var_y)))
            
            if std_dist > self.config.innovation_gate_sigma:
                # Outlier detected - treat as coasting step
                self.coast_frames += 1
                self.status = FilterStatus.COASTING if self.coast_frames <= self.config.max_coast_frames else FilterStatus.LOST
                return self.get_state()

        self.kf.correct(meas_arr)
        self.coast_frames = 0
        self.status = FilterStatus.TRACKING
        self.confidence = float(confidence)
        if bbox is not None:
            self.last_bbox = bbox

        if distance is not None and distance > 0:
            self._update_distance(distance)

        self.total_updates += 1
        return self.get_state()

    def _update_distance(self, distance: float) -> None:
        """Updates range distance and computes distance rate (range velocity)."""
        now = self.last_timestamp
        if self.last_distance_time is not None and now is not None and (now - self.last_distance_time) > 1e-4:
            dt_dist = now - self.last_distance_time
            # Exponentially smoothed distance rate
            raw_vz = (distance - self.last_distance) / dt_dist
            self.distance_rate = 0.7 * self.distance_rate + 0.3 * raw_vz
        self.last_distance = float(distance)
        self.last_distance_time = now

    def update(
        self,
        measurement: Optional[Tuple[float, float]] = None,
        dt: Optional[float] = None,
        distance: Optional[float] = None,
        frame_shape: Optional[Tuple[int, int]] = None,
        timestamp: Optional[float] = None,
        bbox: Optional[Tuple[int, int, int, int]] = None,
        track_id: Optional[int] = None,
        confidence: float = 1.0,
    ) -> TargetState:
        """Primary cycle update: handles detection arrival, occlusion coasting, and state lifecycle.
        
        Parameters:
          measurement: Observed (x, y) centroid in pixels, or None if detection missed.
          dt: Elapsed time since last update (seconds). If None, calculated from timestamp.
          distance: Measured distance in meters (from LiDAR or optical estimator).
          frame_shape: (height, width) of camera frame.
          timestamp: Monotonic epoch timestamp of the frame.
          bbox: (x1, y1, x2, y2) bounding box in pixels.
          track_id: Identifier of the tracked target.
          confidence: Detection confidence score [0.0, 1.0].
        """
        if track_id is not None:
            self.track_id = track_id

        # Passive distance estimation from bounding box if explicit distance omitted
        if distance is None and bbox is not None:
            w_px = max(1, bbox[2] - bbox[0])
            frame_w = frame_shape[1] if frame_shape is not None else self.config.camera_width
            f_px = focal_length_from_hfov(frame_w, self.config.camera_hfov)
            distance = (self.config.known_drone_size * f_px) / w_px

        # Case 1: Active Measurement Observed
        if measurement is not None:
            # If not initialized or previously lost, bootstrap state
            if self.status in (FilterStatus.UNINITIALIZED, FilterStatus.LOST):
                return self.initialize(
                    initial_pos=measurement,
                    timestamp=timestamp,
                    distance=distance,
                    track_id=track_id,
                    bbox=bbox,
                )
            
            # Active filter: Predict then Correct
            self.predict(dt=dt, timestamp=timestamp)
            return self.correct(measurement=measurement, distance=distance, bbox=bbox, confidence=confidence)

        # Case 2: Missed Detection (Occlusion / Coasting)
        if self.status in (FilterStatus.TRACKING, FilterStatus.COASTING):
            self.predict(dt=dt, timestamp=timestamp)
            self.coast_frames += 1
            if self.coast_frames <= self.config.max_coast_frames:
                self.status = FilterStatus.COASTING
            else:
                self.status = FilterStatus.LOST
            return self.get_state(frame_shape=frame_shape)

        # Case 3: Already Lost / Uninitialized with no measurement
        return self.get_state(frame_shape=frame_shape)

    def update_from_detection(
        self,
        detection: Any,
        dt: Optional[float] = None,
        distance: Optional[float] = None,
        frame_shape: Optional[Tuple[int, int]] = None,
        timestamp: Optional[float] = None,
    ) -> TargetState:
        """Convenience method accepting Detection object or None."""
        if detection is None:
            return self.update(
                measurement=None,
                dt=dt,
                distance=distance,
                frame_shape=frame_shape,
                timestamp=timestamp,
            )

        # Extract box, confidence, track_id
        bbox = getattr(detection, "box", None)
        conf = getattr(detection, "confidence", 1.0)
        tid = getattr(detection, "track_id", self.track_id)

        if bbox is not None and len(bbox) == 4:
            x1, y1, x2, y2 = bbox
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            meas = (cx, cy)
        else:
            meas = None

        return self.update(
            measurement=meas,
            dt=dt,
            distance=distance,
            frame_shape=frame_shape,
            timestamp=timestamp,
            bbox=bbox,
            track_id=tid,
            confidence=conf,
        )

    def predict_at_time(self, t_future: float) -> Tuple[float, float]:
        """Predicts 2D pixel position (x, y) at t_future seconds ahead using Constant Acceleration."""
        t = max(0.0, float(t_future))
        st = self.kf.statePost
        x = float(st[0, 0])
        y = float(st[1, 0])
        vx = float(st[2, 0])
        vy = float(st[3, 0])
        ax = float(st[4, 0])
        ay = float(st[5, 0])

        pred_x = x + (vx * t) + (0.5 * ax * t * t)
        pred_y = y + (vy * t) + (0.5 * ay * t * t)
        return (pred_x, pred_y)

    def predict_state_at_time(
        self,
        t_future: float,
        frame_shape: Optional[Tuple[int, int]] = None,
    ) -> TargetState:
        """Predicts full kinematic TargetState at future time t_future seconds ahead."""
        t = max(0.0, float(t_future))
        st = self.kf.statePost
        x = float(st[0, 0]) + float(st[2, 0]) * t + 0.5 * float(st[4, 0]) * t * t
        y = float(st[1, 0]) + float(st[3, 0]) * t + 0.5 * float(st[5, 0]) * t * t
        vx = float(st[2, 0]) + float(st[4, 0]) * t
        vy = float(st[3, 0]) + float(st[5, 0]) * t
        ax = float(st[4, 0])
        ay = float(st[5, 0])

        # Distance prediction
        pred_dist = max(0.1, self.last_distance + self.distance_rate * t)

        width_px = frame_shape[1] if frame_shape else self.config.camera_width
        height_px = frame_shape[0] if frame_shape else self.config.camera_height
        f_px = focal_length_from_hfov(width_px, self.config.camera_hfov)

        pos_3d = pixel_to_camera_3d(x, y, pred_dist, width_px, height_px, f_px, self.config.camera_hfov)
        vX = (vx * pred_dist) / f_px
        vY = -(vy * pred_dist) / f_px
        vZ = self.distance_rate
        vel_3d = (vX, vY, vZ)
        speed_kmh = velocity_to_speed_kmh(vX, vY, vZ)

        return TargetState(
            pos_2d=(x, y),
            vel_2d=(vx, vy),
            acc_2d=(ax, ay),
            pos_3d=pos_3d,
            vel_3d=vel_3d,
            speed_kmh=speed_kmh,
            is_coasting=self.coast_frames > 0,
            coast_frames=self.coast_frames,
            status=self.status,
            confidence=self.confidence,
            timestamp=(self.last_timestamp or 0.0) + t,
            track_id=self.track_id,
            bbox=self.last_bbox,
        )

    def predict_trajectory(
        self,
        dt_horizon: float = 1.0,
        num_steps: int = 20,
    ) -> List[Tuple[float, float]]:
        """Computes sampled 2D forward trajectory points [(x_0, y_0), ..., (x_N, y_N)].
        
        Parameters:
          dt_horizon: Prediction time span in seconds (e.g. 0.1 - 2.0s).
          num_steps: Number of discrete points along the trajectory curve.
        """
        if self.status == FilterStatus.UNINITIALIZED or num_steps < 1:
            return []

        dt_horizon = max(0.01, float(dt_horizon))
        step_size = dt_horizon / float(num_steps)
        
        trajectory: List[Tuple[float, float]] = []
        for i in range(num_steps + 1):
            t_eval = i * step_size
            px, py = self.predict_at_time(t_eval)
            trajectory.append((float(px), float(py)))

        return trajectory

    def predict_trajectory_3d(
        self,
        dt_horizon: float = 1.0,
        num_steps: int = 20,
        frame_shape: Optional[Tuple[int, int]] = None,
    ) -> List[Tuple[float, float, float]]:
        """Computes sampled 3D Cartesian forward trajectory points [(X_0, Y_0, Z_0), ..., (X_N, Y_N, Z_N)]."""
        if self.status == FilterStatus.UNINITIALIZED or num_steps < 1:
            return []

        dt_horizon = max(0.01, float(dt_horizon))
        step_size = dt_horizon / float(num_steps)

        traj_3d: List[Tuple[float, float, float]] = []
        for i in range(num_steps + 1):
            t_eval = i * step_size
            st = self.predict_state_at_time(t_eval, frame_shape=frame_shape)
            traj_3d.append(st.pos_3d)

        return traj_3d

    def get_state(self, frame_shape: Optional[Tuple[int, int]] = None) -> TargetState:
        """Constructs and returns current TargetState snapshot."""
        st = self.kf.statePost
        x = float(st[0, 0])
        y = float(st[1, 0])
        vx = float(st[2, 0])
        vy = float(st[3, 0])
        ax = float(st[4, 0])
        ay = float(st[5, 0])

        width_px = frame_shape[1] if frame_shape else self.config.camera_width
        height_px = frame_shape[0] if frame_shape else self.config.camera_height
        f_px = focal_length_from_hfov(width_px, self.config.camera_hfov)

        # 3D Position & Velocity
        dist = max(0.1, self.last_distance)
        pos_3d = pixel_to_camera_3d(x, y, dist, width_px, height_px, f_px, self.config.camera_hfov)
        vX = (vx * dist) / f_px
        vY = -(vy * dist) / f_px
        vZ = self.distance_rate
        vel_3d = (vX, vY, vZ)
        speed_kmh = velocity_to_speed_kmh(vX, vY, vZ)

        # Uncertainties from covariance
        P = self.kf.errorCovPost
        pos_unc = float(math.sqrt(max(0.0, P[0, 0] + P[1, 1])))
        vel_unc = float(math.sqrt(max(0.0, P[2, 2] + P[3, 3])))

        # Forward Trajectory Prediction (0.0 to 1.0s)
        traj_2d = self.predict_trajectory(dt_horizon=1.0, num_steps=20) if self.status != FilterStatus.UNINITIALIZED else []
        traj_3d = self.predict_trajectory_3d(dt_horizon=1.0, num_steps=20, frame_shape=frame_shape) if self.status != FilterStatus.UNINITIALIZED else []

        return TargetState(
            pos_2d=(x, y),
            vel_2d=(vx, vy),
            acc_2d=(ax, ay),
            pos_3d=pos_3d,
            vel_3d=vel_3d,
            speed_kmh=speed_kmh,
            is_coasting=self.coast_frames > 0 and self.status == FilterStatus.COASTING,
            coast_frames=self.coast_frames,
            status=self.status,
            trajectory_2d=traj_2d,
            trajectory_3d=traj_3d,
            confidence=self.confidence,
            timestamp=self.last_timestamp or 0.0,
            track_id=self.track_id,
            bbox=self.last_bbox,
            pos_uncertainty_px=pos_unc,
            vel_uncertainty_px=vel_unc,
        )

    @property
    def state_vector(self) -> np.ndarray:
        """Current 6-element state vector [x, y, dx, dy, ddx, ddy] as 1D array."""
        return self.kf.statePost.ravel().copy()

    @property
    def covariance_matrix(self) -> np.ndarray:
        """Current 6x6 state error covariance matrix P."""
        return self.kf.errorCovPost.copy()

    @property
    def is_tracking(self) -> bool:
        """True if filter is actively tracking (TRACKING or COASTING)."""
        return self.status in (FilterStatus.TRACKING, FilterStatus.COASTING)

    @property
    def is_coasting(self) -> bool:
        """True if filter is currently coasting across missed detections."""
        return self.status == FilterStatus.COASTING

    @property
    def is_lost(self) -> bool:
        """True if target is declared lost or uninitialized."""
        return self.status in (FilterStatus.LOST, FilterStatus.UNINITIALIZED)


class MultiTargetKalmanTracker:
    """Manages an ensemble of KalmanPredictiveTrackers indexed by track ID.
    
    Provides multi-target association, automatic lifecycle management, and single-target locking.
    """

    def __init__(self, config: Optional[KalmanConfig] = None):
        self.config = config or KalmanConfig()
        self.trackers: Dict[int, KalmanPredictiveTracker] = {}
        self.locked_track_id: Optional[int] = None
        self.frame_count: int = 0

    def get_tracker(self, track_id: int) -> KalmanPredictiveTracker:
        """Retrieves or creates a KalmanPredictiveTracker for the given track ID."""
        if track_id not in self.trackers:
            tracker = KalmanPredictiveTracker(self.config)
            tracker.track_id = track_id
            self.trackers[track_id] = tracker
        return self.trackers[track_id]

    def update_tracks(
        self,
        detections: List[Any],
        dt: Optional[float] = None,
        distances: Optional[Dict[int, float]] = None,
        frame_shape: Optional[Tuple[int, int]] = None,
        timestamp: Optional[float] = None,
    ) -> Dict[int, TargetState]:
        """Updates all active tracks with new detections, handles coasting for unobserved tracks."""
        self.frame_count += 1
        distances = distances or {}
        observed_ids = set()
        results: Dict[int, TargetState] = {}

        # 1. Update tracks with matching detections
        for det in detections:
            tid = getattr(det, "track_id", None)
            if tid is None:
                continue
            observed_ids.add(tid)
            tracker = self.get_tracker(tid)
            dist = distances.get(tid, None)
            state = tracker.update_from_detection(
                detection=det,
                dt=dt,
                distance=dist,
                frame_shape=frame_shape,
                timestamp=timestamp,
            )
            results[tid] = state

        # 2. Coast unobserved active tracks
        unobserved_ids = set(self.trackers.keys()) - observed_ids
        for tid in unobserved_ids:
            tracker = self.trackers[tid]
            if tracker.is_tracking:
                state = tracker.update(
                    measurement=None,
                    dt=dt,
                    frame_shape=frame_shape,
                    timestamp=timestamp,
                )
                results[tid] = state

        # 3. Prune lost tracks
        self.prune_lost_tracks()

        return results

    def prune_lost_tracks(self) -> None:
        """Removes trackers that have entered LOST status, preserving locked target if needed."""
        dead_ids = [
            tid for tid, tracker in self.trackers.items()
            if tracker.is_lost and tid != self.locked_track_id
        ]
        for tid in dead_ids:
            del self.trackers[tid]

    def lock_track(self, track_id: Optional[int]) -> None:
        """Sets the priority locked target ID for single-target engagement."""
        self.locked_track_id = track_id

    def get_locked_target_state(self, frame_shape: Optional[Tuple[int, int]] = None) -> Optional[TargetState]:
        """Returns TargetState of the currently locked target, or None if not locked/found."""
        if self.locked_track_id is not None and self.locked_track_id in self.trackers:
            tracker = self.trackers[self.locked_track_id]
            if tracker.is_tracking:
                return tracker.get_state(frame_shape=frame_shape)

        # Fallback: if no locked ID, pick the oldest active tracking target
        for tid, tracker in self.trackers.items():
            if tracker.is_tracking:
                return tracker.get_state(frame_shape=frame_shape)

        return None

    def reset(self) -> None:
        """Resets all managed trackers and clear lock."""
        self.trackers.clear()
        self.locked_track_id = None
        self.frame_count = 0
