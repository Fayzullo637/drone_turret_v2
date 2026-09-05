"""Configuration Models and Presets for drone_turret_v2.

Features:
- Pydantic v2 data validation and serialization.
- Subsystem configurations: Vision, Kalman, Ballistics, Sensors, PID, Hardware, Server.
- Aggregate SystemConfig with flat-dictionary mutation and preset management.
- REST API compatibility models (TurretConfigModel, TurretStateModel).
"""

from __future__ import annotations

import json
import logging
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

from pydantic import BaseModel, Field, model_validator

logger = logging.getLogger(__name__)


class VisionConfig(BaseModel):
    """Vision and object detection / tracking configuration."""
    camera_id: Union[int, str] = Field(default=0, description="Camera index or 'synthetic'/'sim'")
    model_path: str = Field(default="models/drone_best.pt", description="Path or filename of YOLO weights")
    model_name: str = Field(default="drone_best.pt", description="Name of active YOLO model")
    confidence_threshold: float = Field(default=0.45, ge=0.0, le=1.0, description="Detection confidence threshold [0.0, 1.0]")
    confidence: float = Field(default=0.45, ge=0.0, le=1.0, description="Alias for confidence_threshold")
    target_classes: Optional[List[int]] = Field(default=None, description="Allowed class IDs (None for all)")
    target_class: int = Field(default=0, ge=0, description="Primary target class ID")
    tracker: str = Field(default="bytetrack.yaml", description="Multi-object tracker configuration yaml")
    width: int = Field(default=640, gt=0, description="Capture frame width in pixels")
    height: int = Field(default=480, gt=0, description="Capture frame height in pixels")
    fps: float = Field(default=30.0, gt=0.0, le=240.0, description="Target video frame rate")
    auto_lock: bool = Field(default=True, description="Automatically lock onto closest/largest target")
    auto_fallback: bool = Field(default=True, description="Fallback to synthetic stream if camera disconnects")

    @model_validator(mode="after")
    def sync_confidence(self) -> VisionConfig:
        if self.confidence != self.confidence_threshold:
            self.confidence_threshold = self.confidence
        return self


class KalmanConfigModel(BaseModel):
    """6-State Constant Acceleration Kalman predictive tracking configuration."""
    process_noise_scale: float = Field(default=10.0, gt=0.0, description="Maneuver variance parameter q (px^2/s^5)")
    measurement_noise_std: float = Field(default=3.0, gt=0.0, description="Measurement noise standard deviation sigma (pixels)")
    initial_cov_pos: float = Field(default=10.0, gt=0.0, description="Initial position error variance")
    initial_cov_vel: float = Field(default=1000.0, gt=0.0, description="Initial velocity error variance")
    initial_cov_acc: float = Field(default=10000.0, gt=0.0, description="Initial acceleration error variance")
    max_coast_frames: int = Field(default=15, ge=1, le=100, description="Maximum missed frames before declaring target LOST")
    nominal_dt: float = Field(default=1.0 / 30.0, gt=0.0, description="Default frame delta time (seconds)")
    min_dt: float = Field(default=1e-4, gt=0.0, description="Minimum clamped delta time (seconds)")
    max_dt: float = Field(default=1.0, gt=0.0, description="Maximum clamped delta time (seconds)")
    camera_hfov: float = Field(default=70.0, gt=0.0, lt=180.0, description="Camera horizontal FOV in degrees")
    camera_width: int = Field(default=640, gt=0, description="Camera sensor width in pixels")
    camera_height: int = Field(default=480, gt=0, description="Camera sensor height in pixels")
    default_distance: float = Field(default=30.0, gt=0.0, description="Fallback distance in meters")
    known_drone_size: float = Field(default=0.35, gt=0.0, description="Known drone physical dimension in meters")
    innovation_gate_sigma: float = Field(default=0.0, ge=0.0, description="Outlier gating threshold in sigmas (0 to disable)")


# Alias for backward compatibility
KalmanConfig = KalmanConfigModel


class BallisticsConfigModel(BaseModel):
    """Aerodynamic variable-drag RK4 and Newton-Raphson intercept solver configuration."""
    muzzle_velocity: float = Field(default=80.0, gt=0.0, le=300.0, description="Net launcher muzzle velocity v0 (m/s)")
    projectile_mass: float = Field(default=0.35, gt=0.0, le=5.0, description="Projectile + net mass (kg)")
    net_mass: float = Field(default=0.35, gt=0.0, le=5.0, description="Alias for projectile_mass")
    cd_initial: float = Field(default=0.45, ge=0.1, le=2.5, description="Initial canister drag coefficient")
    cd_max: float = Field(default=1.35, ge=0.5, le=2.5, description="Fully deployed net drag coefficient")
    net_cd: float = Field(default=1.35, ge=0.5, le=2.5, description="Alias for cd_max")
    area_initial: float = Field(default=0.0015, gt=0.0, description="Initial canister frontal area (m^2)")
    area_max: float = Field(default=0.0080, gt=0.0, description="Deployed net frontal area (m^2)")
    tau_deploy: float = Field(default=0.15, gt=0.0, description="Net expansion time constant (seconds)")
    dynamic_expansion: bool = Field(default=True, description="Enable time-dependent Cd(t) and A(t) expansion")
    air_density: float = Field(default=1.225, ge=0.0, description="Air density rho (kg/m^3)")
    gravity: float = Field(default=9.81, ge=0.0, description="Gravitational acceleration g (m/s^2)")
    wind_speed_ms: float = Field(default=0.0, ge=0.0, le=30.0, description="Wind speed in m/s (0 = no wind)")
    wind_direction_deg: float = Field(default=0.0, ge=0.0, lt=360.0, description="Wind direction in degrees (0=from front, 90=from right, 180=from behind)")
    dt_step: float = Field(default=0.005, gt=0.0, le=0.05, description="RK4 numerical integration time step h (seconds)")
    max_flight_time: float = Field(default=3.0, gt=0.1, le=10.0, description="Maximum simulated projectile flight time (seconds)")
    min_effective_range_m: float = Field(default=1.0, ge=0.0, description="Minimum engagement distance in meters")
    max_effective_range_m: float = Field(default=75.0, gt=1.0, description="Maximum engagement distance in meters")
    pan_center_deg: float = Field(default=90.0, ge=0.0, le=180.0, description="Boresight neutral pan angle")
    tilt_center_deg: float = Field(default=90.0, ge=0.0, le=180.0, description="Level neutral tilt angle")
    pan_min_deg: float = Field(default=0.0, ge=0.0, le=180.0, description="Minimum pan angle limit")
    pan_max_deg: float = Field(default=180.0, ge=0.0, le=180.0, description="Maximum pan angle limit")
    tilt_min_deg: float = Field(default=0.0, ge=0.0, le=180.0, description="Minimum tilt angle limit")
    tilt_max_deg: float = Field(default=180.0, ge=0.0, le=180.0, description="Maximum tilt angle limit")
    camera_hfov_deg: float = Field(default=70.0, gt=0.0, lt=180.0, description="Camera horizontal FOV (degrees)")
    camera_frame_size: Tuple[int, int] = Field(default=(640, 480), description="(width, height) in pixels")

    @model_validator(mode="after")
    def sync_aliases(self) -> BallisticsConfigModel:
        if self.net_mass != self.projectile_mass:
            self.projectile_mass = self.net_mass
        if self.net_cd != self.cd_max:
            self.cd_max = self.net_cd
        return self


# Alias for backward compatibility
BallisticsConfig = BallisticsConfigModel


class SensorConfig(BaseModel):
    """Distance sensors (LiDAR & passive optical ranging) configuration."""
    drone_preset: str = Field(default="DJI Mavic 3", description="Preset drone physical dimension")
    drone_real_size: float = Field(default=0.35, gt=0.0, description="Custom drone physical dimension in meters")
    camera_hfov: float = Field(default=70.0, gt=0.0, lt=180.0, description="Camera horizontal FOV in degrees")
    min_distance_m: float = Field(default=0.1, gt=0.0, description="Minimum valid range in meters")
    max_distance_m: float = Field(default=50.0, gt=1.0, description="Maximum valid range in meters")
    lidar_min_strength: int = Field(default=100, ge=0, description="Minimum LiDAR signal strength threshold")


class PIDConfig(BaseModel):
    """Dual-axis discrete PID controller configuration."""
    kp: float = Field(default=0.12, ge=0.0, le=2.0, description="Proportional gain Kp")
    ki: float = Field(default=0.005, ge=0.0, le=1.0, description="Integral gain Ki")
    kd: float = Field(default=0.03, ge=0.0, le=1.0, description="Derivative gain Kd")
    deadband_deg: float = Field(default=0.3, ge=0.0, le=5.0, description="Deadband threshold in degrees")
    max_step_deg: float = Field(default=10.0, gt=0.0, le=45.0, description="Slew rate limit in degrees per frame")
    i_max: float = Field(default=5.0, gt=0.0, description="Anti-windup integral clamping ceiling")
    derivative_alpha: float = Field(default=0.7, ge=0.0, le=1.0, description="Derivative low-pass filter alpha")
    home_pan_deg: float = Field(default=90.0, ge=0.0, le=180.0, description="Home neutral pan angle")
    home_tilt_deg: float = Field(default=90.0, ge=0.0, le=180.0, description="Home neutral tilt angle")


class HardwareConfig(BaseModel):
    """Serial communication and hardware interface configuration."""
    arduino_port: Optional[str] = Field(default=None, description="Arduino serial COM port (None to auto-detect)")
    lidar_port: Optional[str] = Field(default=None, description="LiDAR serial COM port (None to auto-detect)")
    baudrate: int = Field(default=115200, gt=0, description="Serial communication baud rate")
    simulation_mode: bool = Field(default=True, description="Run in virtual software simulation mode")
    tx_rate_hz: float = Field(default=50.0, gt=0.0, le=200.0, description="Maximum serial command transmission rate")
    use_legacy_format: bool = Field(default=False, description="Use legacy comma-separated ASCII format '<pan>,<tilt>\\n'")


class ServerConfig(BaseModel):
    """FastAPI backend and streaming web server configuration."""
    host: str = Field(default="0.0.0.0", description="Host IP address to bind server")
    port: int = Field(default=8000, ge=1, le=65535, description="TCP port to listen on")
    reload: bool = Field(default=False, description="Enable auto-reload on code change")
    debug: bool = Field(default=False, description="Enable debug logging")


class TurretConfigModel(BaseModel):
    """REST API configuration payload model matching test fixture schema."""
    confidence: Optional[float] = Field(default=0.50, ge=0.0, le=1.0)
    target_class: Optional[int] = Field(default=0, ge=0)
    muzzle_velocity: Optional[float] = Field(default=80.0, gt=0.0, le=300.0)
    net_mass: Optional[float] = Field(default=0.60, gt=0.0, le=5.0)
    net_cd: Optional[float] = Field(default=1.20, ge=0.5, le=2.5)
    model_name: Optional[str] = Field(default="yolov8n.pt")
    camera_id: Optional[Union[int, str]] = None
    simulation_mode: Optional[bool] = None
    arduino_port: Optional[str] = None
    lidar_port: Optional[str] = None
    drone_real_size: Optional[float] = None
    camera_hfov: Optional[float] = None
    pid_kp: Optional[float] = None
    pid_ki: Optional[float] = None
    pid_kd: Optional[float] = None


class TurretStateModel(BaseModel):
    """REST API telemetry and status output model."""
    fps: float = 30.0
    pan_angle: float = 90.0
    tilt_angle: float = 90.0
    lead_pan_angle: float = 90.0
    lead_tilt_angle: float = 90.0
    distance_m: float = 25.0
    distance_source: str = "optical"
    speed_kmh: float = 120.0
    target_locked: bool = True
    tracking_state: str = "TRACKING"
    track_id: Optional[int] = None
    target_class: str = "drone"
    confidence: float = 0.0
    intercept_time_s: float = 0.0
    drop_m: float = 0.0
    coast_frames: int = 0
    arduino_connected: bool = False
    lidar_connected: bool = False
    simulation_mode: bool = True
    model_name: str = "drone_best.pt"


class SystemConfig(BaseModel):
    """Root aggregate system configuration combining all subsystem parameters."""
    vision: VisionConfig = Field(default_factory=VisionConfig)
    kalman: KalmanConfigModel = Field(default_factory=KalmanConfigModel)
    ballistics: BallisticsConfigModel = Field(default_factory=BallisticsConfigModel)
    sensors: SensorConfig = Field(default_factory=SensorConfig)
    pid: PIDConfig = Field(default_factory=PIDConfig)
    hardware: HardwareConfig = Field(default_factory=HardwareConfig)
    server: ServerConfig = Field(default_factory=ServerConfig)

    def to_dict(self) -> Dict[str, Any]:
        return self.model_dump()

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> SystemConfig:
        return cls.model_validate(data)

    def to_flat_dict(self) -> Dict[str, Any]:
        return {
            "camera_id": self.vision.camera_id,
            "model_name": self.vision.model_name,
            "model_path": self.vision.model_path,
            "confidence": self.vision.confidence,
            "confidence_threshold": self.vision.confidence_threshold,
            "target_class": self.vision.target_class,
            "target_classes": self.vision.target_classes,
            "muzzle_velocity": self.ballistics.muzzle_velocity,
            "net_mass": self.ballistics.projectile_mass,
            "projectile_mass": self.ballistics.projectile_mass,
            "net_cd": self.ballistics.cd_max,
            "cd_max": self.ballistics.cd_max,
            "drone_preset": self.sensors.drone_preset,
            "drone_real_size": self.sensors.drone_real_size,
            "camera_hfov": self.sensors.camera_hfov,
            "pid_kp": self.pid.kp,
            "pid_ki": self.pid.ki,
            "pid_kd": self.pid.kd,
            "deadband_deg": self.pid.deadband_deg,
            "arduino_port": self.hardware.arduino_port,
            "lidar_port": self.hardware.lidar_port,
            "simulation_mode": self.hardware.simulation_mode,
            "host": self.server.host,
            "port": self.server.port,
            "debug": self.server.debug,
        }

    def update_from_flat_dict(self, data: Dict[str, Any]) -> None:
        for key, value in data.items():
            if value is None:
                continue

            # Vision
            if key in ("confidence", "confidence_threshold"):
                val = max(0.0, min(1.0, float(value)))
                self.vision.confidence = val
                self.vision.confidence_threshold = val
            elif key == "model_name":
                self.vision.model_name = str(value)
                self.vision.model_path = str(value)
            elif key == "model_path":
                self.vision.model_path = str(value)
                self.vision.model_name = os.path.basename(str(value))
            elif key == "target_class":
                self.vision.target_class = max(0, int(value))
            elif key == "target_classes":
                self.vision.target_classes = [int(x) for x in value] if value is not None else None
            elif key == "camera_id":
                self.vision.camera_id = value

            # Ballistics
            elif key == "muzzle_velocity":
                val = max(0.1, min(300.0, float(value)))
                self.ballistics.muzzle_velocity = val
            elif key in ("net_mass", "projectile_mass"):
                val = max(0.01, min(5.0, float(value)))
                self.ballistics.projectile_mass = val
                self.ballistics.net_mass = val
            elif key in ("net_cd", "cd_max"):
                val = max(0.5, min(2.5, float(value)))
                self.ballistics.cd_max = val
                self.ballistics.net_cd = val

            # Sensors
            elif key == "drone_preset":
                self.sensors.drone_preset = str(value)
            elif key in ("drone_real_size", "known_drone_size"):
                val = float(value)
                self.sensors.drone_real_size = val
                self.kalman.known_drone_size = val
            elif key in ("camera_hfov", "camera_hfov_deg"):
                val = float(value)
                self.sensors.camera_hfov = val
                self.kalman.camera_hfov = val
                self.ballistics.camera_hfov_deg = val

            # PID
            elif key in ("pid_kp", "kp"):
                self.pid.kp = float(value)
            elif key in ("pid_ki", "ki"):
                self.pid.ki = float(value)
            elif key in ("pid_kd", "kd"):
                self.pid.kd = float(value)
            elif key in ("deadband_deg", "deadband"):
                self.pid.deadband_deg = float(value)

            # Hardware
            elif key == "arduino_port":
                self.hardware.arduino_port = str(value) if value else None
            elif key == "lidar_port":
                self.hardware.lidar_port = str(value) if value else None
            elif key == "simulation_mode":
                self.hardware.simulation_mode = bool(value)

            # Server
            elif key == "host":
                self.server.host = str(value)
            elif key == "port":
                self.server.port = int(value)
            elif key == "debug":
                self.server.debug = bool(value)

    def save_json(self, path: Union[str, Path]) -> None:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)
        logger.info("Configuration saved to %s", p)

    @classmethod
    def load_json(cls, path: Union[str, Path]) -> SystemConfig:
        p = Path(path)
        if not p.exists():
            raise FileNotFoundError(f"Configuration file not found: {p}")
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls.from_dict(data)

    @classmethod
    def get_preset(cls, name: str = "default") -> SystemConfig:
        cfg = cls()
        preset = name.lower().strip()
        if preset in ("sim", "simulation", "virtual"):
            cfg.hardware.simulation_mode = True
            cfg.vision.camera_id = "synthetic"
            cfg.vision.auto_fallback = True
        elif preset in ("high_speed", "combat", "field"):
            cfg.vision.confidence_threshold = 0.40
            cfg.vision.confidence = 0.40
            cfg.kalman.process_noise_scale = 25.0
            cfg.ballistics.muzzle_velocity = 95.0
            cfg.ballistics.cd_max = 1.35
            cfg.pid.max_step_deg = 15.0
        elif preset in ("close_range", "indoor"):
            cfg.ballistics.muzzle_velocity = 60.0
            cfg.ballistics.max_effective_range_m = 25.0
            cfg.sensors.drone_preset = "FPV Quad"
            cfg.sensors.drone_real_size = 0.22
        return cfg