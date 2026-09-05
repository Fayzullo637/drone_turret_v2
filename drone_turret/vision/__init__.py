"""Vision and Tracking Package for drone_turret_v2.

Exports:
- CameraCapture: Multi-threaded camera frame capture with synthetic fallback.
- CameraDeviceInfo: Dataclass for video capture device metadata.
- SyntheticFrameSource: Generator for synthetic simulation frames.
- discover_cameras: Auto-discovery for available hardware/virtual cameras.
- Detection: Bounding box, confidence, class, and track dataclass.
- DetectionResult: Container for inference output, tracking, and target lock.
- YOLODetector: YOLOv8 inference wrapper with ByteTrack tracking and hot-swapping.
- YOLOTrackerDetector: Alias for YOLODetector.
"""

from drone_turret.vision.camera import (
    CameraCapture,
    CameraDeviceInfo,
    SyntheticFrameSource,
    discover_cameras,
)
from drone_turret.vision.detector import (
    Detection,
    DetectionResult,
    YOLODetector,
    YOLOTrackerDetector,
)

__all__ = [
    "CameraCapture",
    "CameraDeviceInfo",
    "SyntheticFrameSource",
    "discover_cameras",
    "Detection",
    "DetectionResult",
    "YOLODetector",
    "YOLOTrackerDetector",
]
