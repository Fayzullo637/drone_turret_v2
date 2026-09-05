"""
drone_turret: AI Interceptor Turret with Ballistic Net Launcher.

High-throughput, cyber-physical AI guidance architecture for anti-drone turret interception.
"""

from __future__ import annotations

from drone_turret.config import (
    SystemConfig,
    VisionConfig,
    KalmanConfigModel,
    KalmanConfig,
    BallisticsConfigModel,
    BallisticsConfig,
    SensorConfig,
    PIDConfig,
    HardwareConfig,
    ServerConfig,
    TurretConfigModel,
    TurretStateModel,
)
from drone_turret.coordinator import (
    PipelineCoordinator,
    PipelineStatus,
    TelemetryData,
)
from drone_turret.vision.camera import (
    CameraCapture,
    CameraDeviceInfo,
    discover_cameras,
)
from drone_turret.vision.detector import (
    YOLODetector,
    YOLOTrackerDetector,
    Detection,
    DetectionResult,
)
from drone_turret.tracking.kalman_filter import (
    KalmanPredictiveTracker,
    MultiTargetKalmanTracker,
    TargetState,
    FilterStatus,
)
from drone_turret.sensors.distance import DistanceEstimator
from drone_turret.sensors.lidar import (
    LidarParser,
    LidarSerialReader,
    LidarReading,
)
from drone_turret.ballistics.calculator import (
    BallisticCalculator,
    BallisticConfig,
    InterceptSolution,
)
from drone_turret.control.pid import (
    TurretController,
    DiscretePID,
)
from drone_turret.comms.serial_comm import (
    SerialCommunicator,
    MockSerialTransport,
    ArduinoController,
    list_serial_ports,
    find_arduino_port,
)

__version__ = "2.0.0"
__author__ = "drone_turret_v2 Engineering Team"

__all__ = [
    "__version__",
    "__author__",
    "PipelineCoordinator",
    "PipelineStatus",
    "TelemetryData",
    "SystemConfig",
    "VisionConfig",
    "KalmanConfigModel",
    "KalmanConfig",
    "BallisticsConfigModel",
    "BallisticsConfig",
    "SensorConfig",
    "PIDConfig",
    "HardwareConfig",
    "ServerConfig",
    "TurretConfigModel",
    "TurretStateModel",
    "CameraCapture",
    "CameraDeviceInfo",
    "discover_cameras",
    "YOLODetector",
    "YOLOTrackerDetector",
    "Detection",
    "DetectionResult",
    "KalmanPredictiveTracker",
    "MultiTargetKalmanTracker",
    "TargetState",
    "FilterStatus",
    "BallisticCalculator",
    "BallisticConfig",
    "InterceptSolution",
    "DistanceEstimator",
    "LidarParser",
    "LidarSerialReader",
    "LidarReading",
    "TurretController",
    "DiscretePID",
    "SerialCommunicator",
    "MockSerialTransport",
    "ArduinoController",
    "list_serial_ports",
    "find_arduino_port",
]

