import sys
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import time
import numpy as np
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
)


def test_discover_cameras():
    print("[TEST 1] Camera discovery...")
    devices = discover_cameras(max_devices=3)
    assert isinstance(devices, list)
    assert len(devices) >= 1
    # Check structure
    for d in devices:
        assert "id" in d
        assert "name" in d
        assert "width" in d
        assert "height" in d
        assert "fps" in d
        assert "is_available" in d
    assert any(d["id"] == "synthetic" for d in devices)
    print("  -> Passed (found %d devices)" % len(devices))


def test_synthetic_frame_source():
    print("[TEST 2] Synthetic frame source...")
    src = SyntheticFrameSource(width=320, height=240, fps=60.0, render_target=True)
    ret, frame = src.read()
    assert ret is True
    assert frame.shape == (240, 320, 3)
    assert frame.dtype == np.uint8
    print("  -> Passed")


def test_camera_threaded_capture():
    print("[TEST 3] Threaded CameraCapture...")
    cam = CameraCapture(camera_id="synthetic", width=640, height=480, fps=30.0)
    started = cam.start()
    assert started is True
    assert cam.is_running is True
    assert cam.is_opened is True

    # Read some frames
    time.sleep(0.3)
    ret, frame = cam.read(timeout=1.0)
    assert ret is True
    assert frame is not None
    assert frame.shape == (480, 640, 3)

    telemetry = cam.get_telemetry()
    assert telemetry["frame_count"] >= 1
    assert telemetry["width"] == 640
    assert telemetry["height"] == 480

    # Test dynamic camera switch
    switched = cam.change_camera("synthetic", width=320, height=240)
    assert switched is True
    time.sleep(0.1)
    ret2, frame2 = cam.read(timeout=1.0)
    assert ret2 is True
    assert frame2.shape == (240, 320, 3)

    cam.release()
    assert cam.is_running is False
    print("  -> Passed")


def test_detection_dataclass():
    print("[TEST 4] Detection & DetectionResult Dataclasses...")
    d1 = Detection(
        box=(50, 60, 150, 180),
        confidence=0.925,
        class_id=0,
        class_name="shahed_136",
        track_id=12,
    )
    assert d1.x1 == 50
    assert d1.y1 == 60
    assert d1.x2 == 150
    assert d1.y2 == 180
    assert d1.width == 100
    assert d1.height == 120
    assert d1.area == 12000
    assert d1.cx == 100
    assert d1.cy == 120
    assert d1.center == (100, 120)

    d_dict = d1.to_dict()
    assert d_dict["class_name"] == "shahed_136"
    assert d_dict["track_id"] == 12

    res = DetectionResult(
        detections=[d1],
        locked_target=d1,
        target_locked=True,
        locked_track_id=12,
        inference_time_ms=15.4,
        model_name="drone_best.pt",
        frame_shape=(480, 640),
    )
    assert res.to_dict()["target_locked"] is True
    assert res.to_dict()["locked_track_id"] == 12
    print("  -> Passed")


def test_yolo_detector_and_locking():
    print("[TEST 5] YOLODetector inference, hot-swap & lock retention...")
    detector = YOLODetector(model_path="models/drone_best.pt", conf_threshold=0.25)
    assert len(detector.class_names) == 5
    assert detector.class_names[0] == "shahed_136"
    assert detector.class_names[3] == "dji_mavic"

    # Test empty / None frame handling
    empty_res = detector.detect(None)
    assert len(empty_res.detections) == 0
    assert empty_res.target_locked is False

    # Test lock / unlock controls
    detector.lock_track(99)
    assert detector.locked_track_id == 99
    detector.unlock_track()
    assert detector.locked_track_id is None

    # Test filter mutations
    detector.set_target_classes([3])
    assert detector.target_classes == [3]
    detector.set_confidence_threshold(0.75)
    assert detector.conf_threshold == 0.75

    # Test hot-swap to yolov8n.pt
    assert detector.switch_model("models/yolov8n.pt") is True
    assert len(detector.class_names) == 80
    assert detector.model_name == "yolov8n.pt"

    # Test hot-swap back to drone_best.pt
    assert detector.switch_model("models/drone_best.pt") is True
    assert len(detector.class_names) == 5
    assert detector.model_name == "drone_best.pt"

    # Test draw_detections on dummy frame
    dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    drawn = detector.draw_detections(dummy_frame, empty_res)
    assert drawn.shape == (480, 640, 3)

    print("  -> Passed")


if __name__ == "__main__":
    print("========================================")
    print("Running M1 Vision & Tracking Verification")
    print("========================================")
    test_discover_cameras()
    test_synthetic_frame_source()
    test_camera_threaded_capture()
    test_detection_dataclass()
    test_yolo_detector_and_locking()
    print("========================================")
    print("ALL VERIFICATION SUITES PASSED!")
    print("========================================")
