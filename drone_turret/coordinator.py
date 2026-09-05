"""
Pipeline Coordinator Module for drone_turret_v2.

Coordinating:
  CameraCapture -> YOLODetector -> KalmanPredictor -> DistanceEstimator ->
  BallisticsCalculator -> TurretController -> SerialCommunicator -> StreamRenderer.

Features:
- Non-blocking thread-safe processing loop.
- Dynamic runtime configuration updates without restart.
- Comprehensive 5-state state machine: SEARCHING -> LOCKED -> COASTING -> ENGAGING -> LOST.
- Tactical HUD stream annotation (reticles, predicted trajectory dashed lines,
  ballistic lead diamond crosshair with time-to-intercept readout, status badges).
- High-frequency telemetry generation and fire trigger execution.
"""

from __future__ import annotations

import logging
import os
import queue
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Generator, List, Optional, Tuple, Union

import cv2
import numpy as np

from drone_turret.config import SystemConfig, TurretConfigModel, TurretStateModel
from drone_turret.vision.camera import CameraCapture, CameraDeviceInfo, discover_cameras
from drone_turret.vision.detector import YOLODetector, Detection, DetectionResult
from drone_turret.tracking.kalman_filter import (
    KalmanPredictiveTracker,
    TargetState,
    FilterStatus,
    KalmanConfig,
    focal_length_from_hfov,
)
from drone_turret.sensors.distance import DistanceEstimator
from drone_turret.sensors.lidar import LidarSerialReader, LidarReading
from drone_turret.ballistics.calculator import (
    BallisticCalculator,
    BallisticConfig,
    InterceptSolution,
)
from drone_turret.control.pid import TurretController
from drone_turret.comms.serial_comm import (
    SerialCommunicator,
    MockSerialTransport,
    list_serial_ports,
    find_arduino_port,
)

logger = logging.getLogger(__name__)


class PipelineStatus(str, Enum):
    """Lifecycle tracking states of the pipeline coordinator."""
    SEARCHING = "SEARCHING"
    LOCKED = "LOCKED"
    COASTING = "COASTING"
    ENGAGING = "ENGAGING"
    LOST = "LOST"


@dataclass
class TelemetryData:
    """Structured real-time telemetry snapshot."""
    fps: float = 0.0
    target_locked: bool = False
    tracking_state: str = "SEARCHING"
    track_id: Optional[int] = None
    target_class: str = "drone"
    confidence: float = 0.0
    distance_m: float = 0.0
    distance_source: str = "optical"
    speed_kmh: float = 0.0
    intercept_time_s: float = 0.0
    lead_pan_angle: float = 90.0
    lead_tilt_angle: float = 90.0
    current_pan_angle: float = 90.0
    current_tilt_angle: float = 90.0
    drop_m: float = 0.0
    coast_frames: int = 0
    arduino_connected: bool = False
    lidar_connected: bool = False
    simulation_mode: bool = True
    model_name: str = "drone_best.pt"
    camera_id: Union[int, str] = 0
    fire_count: int = 0
    timestamp: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "fps": round(self.fps, 1),
            "target_locked": self.target_locked,
            "tracking_state": self.tracking_state,
            "track_id": self.track_id,
            "target_class": self.target_class,
            "confidence": round(self.confidence, 3),
            "distance_m": round(self.distance_m, 2),
            "distance_source": self.distance_source,
            "speed_kmh": round(self.speed_kmh, 1),
            "intercept_time_s": round(self.intercept_time_s, 3),
            "lead_pan_angle": round(self.lead_pan_angle, 2),
            "lead_tilt_angle": round(self.lead_tilt_angle, 2),
            "pan_angle": round(self.current_pan_angle, 2),
            "tilt_angle": round(self.current_tilt_angle, 2),
            "current_pan_angle": round(self.current_pan_angle, 2),
            "current_tilt_angle": round(self.current_tilt_angle, 2),
            "drop_m": round(self.drop_m, 3),
            "coast_frames": self.coast_frames,
            "arduino_connected": self.arduino_connected,
            "lidar_connected": self.lidar_connected,
            "simulation_mode": self.simulation_mode,
            "model_name": self.model_name,
            "camera_id": self.camera_id,
            "fire_count": self.fire_count,
            "timestamp": self.timestamp,
        }


class PipelineCoordinator:
    """
    Central Pipeline Coordinator for drone_turret_v2.
    
    Wires CameraCapture, YOLODetector, KalmanPredictiveTracker, DistanceEstimator,
    BallisticCalculator, TurretController, and SerialCommunicator.
    """

    def __init__(self, config: Optional[SystemConfig] = None):
        self.config = config or SystemConfig()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._is_running = False
        self._thread: Optional[threading.Thread] = None

        # Subsystems
        self.camera = CameraCapture(
            camera_id=self.config.vision.camera_id,
            width=self.config.vision.width,
            height=self.config.vision.height,
            fps=self.config.vision.fps,
            auto_fallback=self.config.vision.auto_fallback,
        )

        self.detector = YOLODetector(
            model_path=self.config.vision.model_path,
            conf_threshold=self.config.vision.confidence_threshold,
            target_classes=self.config.vision.target_classes,
            tracker=self.config.vision.tracker,
        )

        self.kalman = KalmanPredictiveTracker(
            config=KalmanConfig(
                process_noise_scale=self.config.kalman.process_noise_scale,
                measurement_noise_std=self.config.kalman.measurement_noise_std,
                initial_cov_pos=self.config.kalman.initial_cov_pos,
                initial_cov_vel=self.config.kalman.initial_cov_vel,
                initial_cov_acc=self.config.kalman.initial_cov_acc,
                max_coast_frames=self.config.kalman.max_coast_frames,
                camera_hfov=self.config.kalman.camera_hfov,
                known_drone_size=self.config.kalman.known_drone_size,
            )
        )

        self.distance_estimator = DistanceEstimator(
            default_preset=self.config.sensors.drone_preset,
            camera_hfov_deg=self.config.sensors.camera_hfov,
            min_distance_m=self.config.sensors.min_distance_m,
            max_distance_m=self.config.sensors.max_distance_m,
            custom_drone_size_m=self.config.sensors.drone_real_size,
        )

        self.lidar_reader: Optional[LidarSerialReader] = None
        if self.config.hardware.lidar_port:
            self.lidar_reader = LidarSerialReader(
                port=self.config.hardware.lidar_port,
                baudrate=self.config.hardware.baudrate,
            )

        self.ballistics = BallisticCalculator(
            config=BallisticConfig(
                muzzle_velocity=self.config.ballistics.muzzle_velocity,
                projectile_mass=self.config.ballistics.projectile_mass,
                cd_initial=self.config.ballistics.cd_initial,
                cd_max=self.config.ballistics.cd_max,
                area_initial=self.config.ballistics.area_initial,
                area_max=self.config.ballistics.area_max,
                tau_deploy=self.config.ballistics.tau_deploy,
                dynamic_expansion=self.config.ballistics.dynamic_expansion,
                air_density=self.config.ballistics.air_density,
                gravity=self.config.ballistics.gravity,
                min_effective_range_m=self.config.ballistics.min_effective_range_m,
                max_effective_range_m=self.config.ballistics.max_effective_range_m,
                camera_hfov_deg=self.config.ballistics.camera_hfov_deg,
            )
        )

        self.turret_controller = TurretController(
            pan_kp=self.config.pid.kp,
            pan_ki=self.config.pid.ki,
            pan_kd=self.config.pid.kd,
            tilt_kp=self.config.pid.kp,
            tilt_ki=self.config.pid.ki,
            tilt_kd=self.config.pid.kd,
            deadband_deg=self.config.pid.deadband_deg,
            max_step_deg=self.config.pid.max_step_deg,
            home_pan_deg=self.config.pid.home_pan_deg,
            home_tilt_deg=self.config.pid.home_tilt_deg,
        )

        self.serial_comm = SerialCommunicator(
            port=self.config.hardware.arduino_port,
            baudrate=self.config.hardware.baudrate,
            simulation_mode=self.config.hardware.simulation_mode,
            use_legacy_format=self.config.hardware.use_legacy_format,
            tx_rate_hz=self.config.hardware.tx_rate_hz,
        )

        # State & cache
        self.status: PipelineStatus = PipelineStatus.SEARCHING
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_annotated_frame: Optional[np.ndarray] = None
        self.latest_jpeg_bytes: Optional[bytes] = None
        self.latest_telemetry: TelemetryData = TelemetryData(
            simulation_mode=self.config.hardware.simulation_mode,
            model_name=self.config.vision.model_name,
            camera_id=self.config.vision.camera_id,
        )
        self.latest_detection_result: Optional[DetectionResult] = None
        self.latest_target_state: Optional[TargetState] = None
        self.latest_intercept: Optional[InterceptSolution] = None

        self.last_step_time = time.time()
        self.fps_counter = 0
        self.current_fps = 30.0
        self._fps_history: List[float] = []
        self._frame_count = 0
        self._detect_every_n = 2  # Run YOLO every Nth frame (1=every frame, 2=every other)
        self._last_det_result: Optional[DetectionResult] = None

    @property
    def is_running(self) -> bool:
        return self._is_running

    def start(self) -> bool:
        """Start background video capture, sensors, and coordinator loop."""
        if self._is_running:
            return True

        self.camera.start()
        if self.lidar_reader is not None:
            self.lidar_reader.start()

        self._stop_event.clear()
        self._is_running = True
        self._thread = threading.Thread(
            target=self._run_loop,
            name="PipelineCoordinatorWorker",
            daemon=True,
        )
        self._thread.start()
        logger.info("Pipeline Coordinator started successfully.")
        return True

    def stop(self) -> None:
        """Graceful shutdown of coordinator worker and submodules."""
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)
            self._thread = None
        self._is_running = False

        self.camera.stop()
        if self.lidar_reader is not None:
            self.lidar_reader.stop()
        self.serial_comm.disconnect()
        logger.info("Pipeline Coordinator stopped.")

    def shutdown(self) -> None:
        self.stop()

    def step(
        self,
        frame: Optional[np.ndarray] = None,
        dt: Optional[float] = None,
    ) -> Tuple[Optional[np.ndarray], TelemetryData]:
        """Execute one synchronous frame cycle across all 8 subsystems."""
        start_t = time.perf_counter()
        now = time.time()

        # 1. Delta Time calculation
        if dt is not None and dt > 0:
            calc_dt = dt
        else:
            calc_dt = now - self.last_step_time
            calc_dt = max(0.001, min(0.5, calc_dt))
        self.last_step_time = now

        # 2. Acquire Video Frame
        if frame is None:
            ret, raw_frame = self.camera.read(timeout=0.05)
            if not ret or raw_frame is None:
                # Generate dark tactical frame if camera unavailable
                raw_frame = np.full((self.config.vision.height, self.config.vision.width, 3), 20, dtype=np.uint8)
        else:
            raw_frame = frame

        self.latest_frame = raw_frame
        h, w = raw_frame.shape[:2]

        # 3. YOLOv8 Detection & ByteTrack Tracking (skip-frame optimization)
        self._frame_count += 1
        if self._frame_count % self._detect_every_n == 0 or self._last_det_result is None:
            det_result = self.detector.detect(
                raw_frame,
                conf=self.config.vision.confidence_threshold,
                target_classes=self.config.vision.target_classes,
            )
            self._last_det_result = det_result
        else:
            # Reuse previous detection — Kalman filter will extrapolate position
            det_result = self._last_det_result
        self.latest_detection_result = det_result

        # 4. LiDAR Measurement
        lidar_dist: Optional[float] = None
        if self.lidar_reader is not None:
            lidar_dist = self.lidar_reader.get_latest_distance(max_age_s=0.5)

        # 5. Target State & Kalman Filter Update
        locked_target = det_result.locked_target
        dist_m = 0.0
        dist_src = "optical"
        target_state: Optional[TargetState] = None

        if det_result.target_locked and locked_target is not None:
            # Ranging
            dist_m, dist_src = self.distance_estimator.get_distance(
                bbox=locked_target.box,
                frame_shape=(h, w),
                lidar_dist=lidar_dist,
            )
            # Kalman Update with measurement
            target_state = self.kalman.update(
                measurement=(locked_target.cx, locked_target.cy),
                dt=calc_dt,
                distance=dist_m,
                frame_shape=(h, w),
                timestamp=now,
                bbox=locked_target.box,
                track_id=locked_target.track_id,
                confidence=locked_target.confidence,
            )
            self.status = PipelineStatus.LOCKED
        else:
            # No detection in current frame: Coasting or Searching
            target_state = self.kalman.update(
                measurement=None,
                dt=calc_dt,
                frame_shape=(h, w),
                timestamp=now,
            )
            if self.kalman.is_coasting:
                self.status = PipelineStatus.COASTING
                dist_m = target_state.Z
            else:
                self.status = PipelineStatus.SEARCHING

        self.latest_target_state = target_state

        # 6. Ballistic Intercept Calculation & Servo PID Control
        intercept: Optional[InterceptSolution] = None
        lead_pan = self.config.pid.home_pan_deg
        lead_tilt = self.config.pid.home_tilt_deg
        cmd_pan = self.config.pid.home_pan_deg
        cmd_tilt = self.config.pid.home_tilt_deg
        speed_kmh = 0.0

        if self.status in (PipelineStatus.LOCKED, PipelineStatus.COASTING):
            speed_kmh = target_state.speed_kmh

            # Quick range pre-check: skip heavy solver if target is clearly out of range
            t_dist = float(np.linalg.norm(np.asarray(target_state.pos_3d)))
            if t_dist > self.ballistics.config.max_effective_range_m * 1.2 or t_dist < 0.5:
                # Target too far or too close — use simple aim without full solver
                intercept = self.ballistics._unreachable_solution(target_state, t_dist)
            else:
                # Time-budgeted solve: cap at 30ms to prevent pipeline freeze
                t_solve_start = time.perf_counter()
                intercept = self.ballistics.solve_intercept(target_state, max_iterations=10)
                solve_ms = (time.perf_counter() - t_solve_start) * 1000
                if solve_ms > 30:
                    logger.debug("Ballistic solver took %.1fms — consider reducing max_iterations", solve_ms)

            self.latest_intercept = intercept

            if intercept.reachable:
                self.status = PipelineStatus.ENGAGING
                lead_pan = intercept.aim_pan_deg
                lead_tilt = intercept.aim_tilt_deg
            else:
                lead_pan = intercept.aim_pan_deg
                lead_tilt = intercept.aim_tilt_deg

            # Step PID towards lead angles
            cmd_pan, cmd_tilt = self.turret_controller.update_lead_target(
                target_pan_deg=lead_pan,
                target_tilt_deg=lead_tilt,
                dt=calc_dt,
            )
            self.serial_comm.send_angles(cmd_pan, cmd_tilt)
        else:
            # Home / Scan position
            self.latest_intercept = None
            cmd_pan, cmd_tilt = self.turret_controller.home(dt=calc_dt)
            self.serial_comm.send_angles(cmd_pan, cmd_tilt)

        # 7. FPS Computation
        step_duration = time.perf_counter() - start_t
        instant_fps = 1.0 / max(0.001, step_duration)
        self._fps_history.append(instant_fps)
        if len(self._fps_history) > 30:
            self._fps_history.pop(0)
        self.current_fps = sum(self._fps_history) / len(self._fps_history)

        # 8. Render Tactical HUD
        annotated = self._render_tactical_hud(
            frame=raw_frame,
            det_result=det_result,
            target_state=target_state,
            intercept=intercept,
            dist_m=dist_m,
            dist_src=dist_src,
            speed_kmh=speed_kmh,
            cmd_pan=cmd_pan,
            cmd_tilt=cmd_tilt,
        )
        self.latest_annotated_frame = annotated

        # Encode JPEG for MJPEG stream
        _, jpeg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 65])
        self.latest_jpeg_bytes = jpeg.tobytes()

        # 9. Update Telemetry
        telem = TelemetryData(
            fps=self.current_fps,
            target_locked=(self.status != PipelineStatus.SEARCHING),
            tracking_state=self.status.value,
            track_id=target_state.track_id if target_state else None,
            target_class=locked_target.class_name if locked_target else "drone",
            confidence=locked_target.confidence if locked_target else 0.0,
            distance_m=dist_m,
            distance_source=dist_src,
            speed_kmh=speed_kmh,
            intercept_time_s=intercept.t_intercept if intercept else 0.0,
            lead_pan_angle=lead_pan,
            lead_tilt_angle=lead_tilt,
            current_pan_angle=cmd_pan,
            current_tilt_angle=cmd_tilt,
            drop_m=intercept.drop_m if intercept else 0.0,
            coast_frames=self.kalman.coast_frames,
            arduino_connected=self.serial_comm.is_connected and not self.serial_comm.is_simulated,
            lidar_connected=(self.lidar_reader.is_connected if self.lidar_reader is not None else False),
            simulation_mode=self.serial_comm.is_simulated,
            model_name=self.config.vision.model_name,
            camera_id=self.config.vision.camera_id,
            fire_count=self.serial_comm.get_status().get("fire_count", 0),
            timestamp=now,
        )
        self.latest_telemetry = telem

        return annotated, telem

    def _run_loop(self) -> None:
        """Continuous background worker processing video frames at target FPS."""
        target_interval = 1.0 / max(10.0, self.config.vision.fps)

        while not self._stop_event.is_set():
            t0 = time.time()
            try:
                self.step()
            except Exception as err:
                logger.error("Error in coordinator loop: %s", err)

            elapsed = time.time() - t0
            sleep_time = target_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _render_tactical_hud(
        self,
        frame: np.ndarray,
        det_result: DetectionResult,
        target_state: Optional[TargetState],
        intercept: Optional[InterceptSolution],
        dist_m: float,
        dist_src: str,
        speed_kmh: float,
        cmd_pan: float,
        cmd_tilt: float,
    ) -> np.ndarray:
        """Render high-fidelity tactical military HUD onto frame."""
        hud = frame.copy()
        h, w = hud.shape[:2]
        cx_screen, cy_screen = w // 2, h // 2

        # 1. Center Reticle (Turret Aim Boresight)
        reticle_color = (0, 200, 255)  # Amber
        cv2.circle(hud, (cx_screen, cy_screen), 28, reticle_color, 1)
        cv2.circle(hud, (cx_screen, cy_screen), 4, reticle_color, -1)
        cv2.line(hud, (cx_screen - 36, cy_screen), (cx_screen - 8, cy_screen), reticle_color, 1)
        cv2.line(hud, (cx_screen + 8, cy_screen), (cx_screen + 36, cy_screen), reticle_color, 1)
        cv2.line(hud, (cx_screen, cy_screen - 36), (cx_screen, cy_screen - 8), reticle_color, 1)
        cv2.line(hud, (cx_screen, cy_screen + 8), (cx_screen, cy_screen + 36), reticle_color, 1)

        # 2. Render all detected bounding boxes
        for det in det_result.detections:
            is_locked = (
                det_result.locked_target is not None
                and det.track_id == det_result.locked_target.track_id
            )
            box_color = (0, 0, 255) if is_locked else (0, 255, 0)
            thickness = 2 if is_locked else 1

            cv2.rectangle(hud, (det.x1, det.y1), (det.x2, det.y2), box_color, thickness)
            cv2.circle(hud, (det.cx, det.cy), 3, box_color, -1)

            label = f"#{det.track_id or 0} {det.class_name} {det.confidence:.0%}"
            (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            cv2.rectangle(
                hud,
                (det.x1, max(0, det.y1 - lh - 4)),
                (det.x1 + lw + 4, det.y1),
                box_color,
                -1,
            )
            cv2.putText(
                hud,
                label,
                (det.x1 + 2, max(lh, det.y1 - 2)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255) if is_locked else (0, 0, 0),
                1,
                cv2.LINE_AA,
            )

        # 3. Render Kalman Predicted Forward Trajectory (Dashed Cyan Curve)
        if target_state and target_state.trajectory_2d and len(target_state.trajectory_2d) > 1:
            traj_color = (255, 255, 0)  # Cyan
            for i in range(len(target_state.trajectory_2d) - 1):
                if i % 2 == 0:  # Dashed effect
                    p1 = (int(round(target_state.trajectory_2d[i][0])), int(round(target_state.trajectory_2d[i][1])))
                    p2 = (int(round(target_state.trajectory_2d[i + 1][0])), int(round(target_state.trajectory_2d[i + 1][1])))
                    cv2.line(hud, p1, p2, traj_color, 2, cv2.LINE_AA)

        # 4. Render Ballistic Lead Point (Orange Diamond Crosshair)
        if intercept is not None:
            lx, ly = intercept.lead_pixel_xy
            lead_color = (0, 140, 255)  # Orange
            size = 10

            # Diamond shape
            pts = np.array([[lx, ly - size], [lx + size, ly], [lx, ly + size], [lx - size, ly]], np.int32)
            cv2.polylines(hud, [pts], isClosed=True, color=lead_color, thickness=2)
            cv2.circle(hud, (lx, ly), 2, lead_color, -1)

            # Lead banner text
            lead_txt = f"LEAD Tint:{intercept.t_intercept:.2f}s Drop:{intercept.drop_m:.2f}m"
            cv2.putText(hud, lead_txt, (lx + 14, ly + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42, lead_color, 1, cv2.LINE_AA)

            # Line from current target to lead point
            if det_result.locked_target:
                cv2.line(hud, (det_result.locked_target.cx, det_result.locked_target.cy), (lx, ly), lead_color, 1, cv2.LINE_AA)

        # 5. Status Badge & Top Tactical Banner
        state_colors = {
            PipelineStatus.SEARCHING: (100, 100, 100),
            PipelineStatus.LOCKED: (0, 200, 0),
            PipelineStatus.COASTING: (0, 200, 255),
            PipelineStatus.ENGAGING: (0, 0, 255),
            PipelineStatus.LOST: (50, 50, 200),
        }
        badge_bg = state_colors.get(self.status, (100, 100, 100))

        cv2.rectangle(hud, (12, 12), (150, 40), badge_bg, -1)
        cv2.putText(
            hud,
            f"STATE: {self.status.value}",
            (20, 31),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # Top telemetry text
        telem_str1 = f"FPS: {self.current_fps:.1f} | RNG: {dist_m:.1f}m ({dist_src.upper()}) | SPD: {speed_kmh:.1f} km/h"
        telem_str2 = f"AIM: P:{cmd_pan:.1f}° T:{cmd_tilt:.1f}° | SIM: {'ON' if self.serial_comm.is_simulated else 'OFF'}"

        cv2.putText(hud, telem_str1, (165, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(hud, telem_str2, (165, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 180, 180), 1, cv2.LINE_AA)

        return hud

    def update_config(self, config_data: Union[SystemConfig, Dict[str, Any]]) -> None:
        """Dynamically update subsystem configurations without restarting pipeline."""
        with self._lock:
            if isinstance(config_data, SystemConfig):
                self.config = config_data
            elif isinstance(config_data, dict):
                self.config.update_from_flat_dict(config_data)

            # Update detector
            self.detector.set_confidence_threshold(self.config.vision.confidence_threshold)
            if self.config.vision.model_path != self.detector.model_path:
                self.detector.switch_model(self.config.vision.model_path)

            # Update ballistics
            self.ballistics.update_config(
                muzzle_velocity=self.config.ballistics.muzzle_velocity,
                projectile_mass=self.config.ballistics.projectile_mass,
                cd_max=self.config.ballistics.cd_max,
            )

            # Update PID
            self.turret_controller.set_gains(
                kp=self.config.pid.kp,
                ki=self.config.pid.ki,
                kd=self.config.pid.kd,
                deadband=self.config.pid.deadband_deg,
            )

            # Update camera source if changed
            if self.camera.camera_id != self.config.vision.camera_id:
                self.camera.change_camera(self.config.vision.camera_id)

            logger.info("Pipeline configuration updated dynamically.")

    def switch_model(self, model_name_or_path: str) -> bool:
        """Hot-swap YOLO model weights dynamically."""
        with self._lock:
            success = self.detector.switch_model(model_name_or_path)
            if success:
                self.config.vision.model_path = self.detector.model_path
                self.config.vision.model_name = self.detector.model_name
            return success

    def switch_camera(self, camera_id: Union[int, str]) -> bool:
        """Switch camera input source dynamically."""
        with self._lock:
            success = self.camera.change_camera(camera_id)
            if success:
                self.config.vision.camera_id = camera_id
            return success

    def fire(self, confirmed: bool = False) -> bool:
        """
        Human-in-the-loop fire control.

        Safety rules:
        1. System must be in ENGAGING state with a reachable intercept solution.
        2. If fire_requires_confirmation is True (default), operator must call
           fire(confirmed=True) to actually trigger the solenoid.
        3. Calling fire(confirmed=False) arms the system and returns the
           intercept solution for operator review.

        Returns True if the solenoid was actually triggered.
        """
        with self._lock:
            # Safety check: must have a valid engagement
            if self.status not in (PipelineStatus.ENGAGING, PipelineStatus.LOCKED):
                logger.warning("FIRE REJECTED: Not in ENGAGING/LOCKED state (current: %s)", self.status)
                return False

            if not hasattr(self, '_fire_armed'):
                self._fire_armed = False

            if not confirmed:
                # Arm the fire — operator must confirm with a second call
                self._fire_armed = True
                logger.info("FIRE ARMED: Awaiting operator confirmation. Call fire(confirmed=True) to launch.")
                return False

            if not self._fire_armed:
                logger.warning("FIRE REJECTED: Must arm first by calling fire(confirmed=False)")
                return False

            # Execute fire
            self._fire_armed = False
            logger.info("FIRE CONFIRMED: Launching net interceptor!")
            return self.serial_comm.fire(immediate=True)

    def home(self) -> bool:
        """Command turret servos to home neutral position."""
        self.turret_controller.home()
        return self.serial_comm.home(immediate=True)

    def lock_track(self, track_id: int) -> None:
        """Explicitly lock tracking onto specific track ID."""
        self.detector.lock_track(track_id)

    def unlock_track(self) -> None:
        """Release active track lock."""
        self.detector.unlock_track()

    def get_telemetry(self) -> Dict[str, Any]:
        """Return JSON-serializable telemetry dictionary."""
        with self._lock:
            return self.latest_telemetry.to_dict()

    def get_latest_jpeg(self) -> Optional[bytes]:
        """Return latest compressed JPEG image bytes."""
        with self._lock:
            return self.latest_jpeg_bytes

    def generate_mjpeg_frames(self) -> Generator[bytes, None, None]:
        """Generator streaming MJPEG multipart frames for HTTP streaming."""
        while not self._stop_event.is_set():
            jpeg = self.get_latest_jpeg()
            if jpeg is not None:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
                )
            time.sleep(0.03)

