"""Multi-threaded Camera Capture, Device Auto-Discovery, and Synthetic Frame Source.

Features:
- Multi-threaded non-blocking OpenCV VideoCapture reading loop.
- Windows DirectShow (cv2.CAP_DSHOW) hardware auto-discovery.
- Automatic or manual fallback to high-fidelity synthetic tactical video stream.
- Thread-safe latest-frame caching and sliding-window FPS computation.
- Runtime dynamic camera switching and graceful resource cleanup.
"""

from __future__ import annotations

import logging
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import cv2
import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class CameraDeviceInfo:
    """Information about a detected video capture device."""
    id: Union[int, str]
    name: str
    width: int
    height: int
    fps: float
    is_available: bool
    is_synthetic: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "is_available": self.is_available,
            "is_synthetic": self.is_synthetic,
        }


def discover_cameras(max_devices: int = 5) -> List[Dict[str, Any]]:
    """Scan and return list of available camera capture devices.
    
    Probes indices 0..max_devices-1 using DirectShow on Windows for fast,
    non-blocking device querying, releasing handles immediately.
    Always provides a simulated synthetic source fallback.
    """
    discovered: List[CameraDeviceInfo] = []
    is_windows = sys.platform.startswith("win") or os.name == "nt"
    backend = cv2.CAP_DSHOW if is_windows else cv2.CAP_ANY

    for idx in range(max_devices):
        cap = None
        try:
            cap = cv2.VideoCapture(idx, backend)
            if cap is not None and cap.isOpened():
                # Read properties
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480
                fps = float(cap.get(cv2.CAP_PROP_FPS))
                if fps <= 0 or math.isnan(fps):
                    fps = 30.0
                
                # Verify we can grab at least one test frame
                ret, _ = cap.read()
                if ret:
                    dev_info = CameraDeviceInfo(
                        id=idx,
                        name=f"Camera {idx} ({'DirectShow' if is_windows else 'Default'})",
                        width=w,
                        height=h,
                        fps=round(fps, 2),
                        is_available=True,
                        is_synthetic=False,
                    )
                    discovered.append(dev_info)
        except Exception as err:
            logger.debug("Error probing camera index %d: %s", idx, err)
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass

    # Always include synthetic source in discovered list
    discovered.append(
        CameraDeviceInfo(
            id="synthetic",
            name="Synthetic Tactical Simulator (Virtual)",
            width=640,
            height=480,
            fps=30.0,
            is_available=True,
            is_synthetic=True,
        )
    )

    return [dev.to_dict() for dev in discovered]


class SyntheticFrameSource:
    """High-fidelity synthetic frame generator for simulation and camera fallback."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: float = 30.0,
        render_target: bool = True,
    ):
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.dt = 1.0 / max(1.0, fps)
        self.render_target = render_target
        self.frame_idx = 0
        self.start_time = time.time()
        self.last_frame_time = time.time()

        # Simulated target trajectory state
        self.target_pos = [self.width * 0.2, self.height * 0.3]
        self.target_vel = [120.0, 45.0]  # px/s

    def read(self) -> Tuple[bool, np.ndarray]:
        """Generate next synthetic frame with realistic timing pacing."""
        now = time.time()
        elapsed = now - self.last_frame_time
        sleep_dur = self.dt - elapsed
        if sleep_dur > 0:
            time.sleep(sleep_dur)
        self.last_frame_time = time.time()

        self.frame_idx += 1
        t = self.frame_idx * self.dt

        # Dark tactical background
        frame = np.full((self.height, self.width, 3), 28, dtype=np.uint8)

        # Tactical background grid lines
        grid_step = 80
        for x in range(0, self.width, grid_step):
            cv2.line(frame, (x, 0), (x, self.height), (38, 38, 38), 1)
        for y in range(0, self.height, grid_step):
            cv2.line(frame, (0, y), (self.width, y), (38, 38, 38), 1)

        # Artificial horizon
        mid_y = self.height // 2
        cv2.line(frame, (0, mid_y), (self.width, mid_y), (50, 50, 50), 1)
        cv2.line(frame, (self.width // 2, 0), (self.width // 2, self.height), (50, 50, 50), 1)

        # Render simulated moving drone target if enabled
        if self.render_target:
            # Update target position with bouncing boundary kinematics
            self.target_pos[0] += self.target_vel[0] * self.dt
            self.target_pos[1] += self.target_vel[1] * self.dt

            margin = 60
            if self.target_pos[0] < margin or self.target_pos[0] > self.width - margin:
                self.target_vel[0] *= -1.0
                self.target_pos[0] = max(margin, min(self.width - margin, self.target_pos[0]))

            if self.target_pos[1] < margin or self.target_pos[1] > self.height - margin:
                self.target_vel[1] *= -1.0
                self.target_pos[1] = max(margin, min(self.height - margin, self.target_pos[1]))

            tx, ty = int(self.target_pos[0]), int(self.target_pos[1])
            size = 32
            x1, y1 = tx - size // 2, ty - size // 2
            x2, y2 = tx + size // 2, ty + size // 2

            # Drone sprite: body & quad arms
            cv2.rectangle(frame, (x1, y1), (x2, y2), (200, 200, 200), 1)
            cv2.line(frame, (x1, y1), (x2, y2), (100, 100, 100), 1)
            cv2.line(frame, (x1, y2), (x2, y1), (100, 100, 100), 1)
            cv2.circle(frame, (tx, ty), 4, (0, 0, 255), -1)

            # Rotor hubs
            for rx, ry in [(x1, y1), (x2, y1), (x1, y2), (x2, y2)]:
                cv2.circle(frame, (rx, ry), 3, (0, 255, 255), -1)

        # Synthetic HUD overlay text
        cv2.putText(
            frame,
            f"SIMULATED VIDEO STREAM [{self.width}x{self.height}] - Frame: {self.frame_idx}",
            (15, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 200, 0),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"T: {t:.2f}s | FPS: {self.fps:.1f}",
            (15, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (160, 160, 160),
            1,
            cv2.LINE_AA,
        )

        return True, frame


class CameraCapture:
    """Multi-threaded OpenCV camera capture with automatic synthetic fallback.
    
    Provides thread-safe, non-blocking frame retrieval, sliding-window FPS tracking,
    and runtime device switching.
    """

    def __init__(
        self,
        camera_id: Union[int, str] = 0,
        width: int = 640,
        height: int = 480,
        fps: float = 30.0,
        auto_fallback: bool = True,
        render_synthetic_target: bool = True,
        custom_frame_generator: Optional[Callable[[], Tuple[bool, np.ndarray]]] = None,
    ):
        self.camera_id = camera_id
        self.requested_width = width
        self.requested_height = height
        self.target_fps = fps
        self.auto_fallback = auto_fallback
        self.render_synthetic_target = render_synthetic_target
        self.custom_frame_generator = custom_frame_generator

        # Internal state
        self._cap: Optional[cv2.VideoCapture] = None
        self._synthetic_source: Optional[SyntheticFrameSource] = None
        self._is_synthetic = False
        self._is_opened = False
        self._is_running = False

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # Frame cache & telemetry
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_timestamp: float = 0.0
        self._frame_count: int = 0
        self._fps_history: List[float] = []
        self._current_fps: float = 0.0
        self._actual_width: int = width
        self._actual_height: int = height

        # Initialize backend
        self._init_source()

    @property
    def is_synthetic(self) -> bool:
        """True if capturing from synthetic video source."""
        return self._is_synthetic

    @property
    def is_opened(self) -> bool:
        """True if camera or synthetic source is open and ready."""
        return self._is_opened

    @property
    def is_running(self) -> bool:
        """True if capture thread is active."""
        return self._is_running

    @property
    def current_fps(self) -> float:
        """Sliding window measured FPS."""
        with self._lock:
            return self._current_fps

    @property
    def frame_count(self) -> int:
        """Total frames captured since start."""
        with self._lock:
            return self._frame_count

    @property
    def resolution(self) -> Tuple[int, int]:
        """Current frame resolution (width, height)."""
        return (self._actual_width, self._actual_height)

    def _init_source(self) -> bool:
        """Initialize physical VideoCapture or fallback to synthetic source."""
        # Check if synthetic source is explicitly requested
        if (
            self.camera_id in ("synthetic", "test", "sim", -1, None)
            or self.custom_frame_generator is not None
        ):
            self._setup_synthetic()
            return True

        # Attempt to open physical camera
        is_windows = sys.platform.startswith("win") or os.name == "nt"
        backend = cv2.CAP_DSHOW if is_windows else cv2.CAP_ANY

        try:
            cam_idx = int(self.camera_id)
            self._cap = cv2.VideoCapture(cam_idx, backend)
            if self._cap is not None and self._cap.isOpened():
                # Set requested resolution and FPS
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.requested_width)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.requested_height)
                self._cap.set(cv2.CAP_PROP_FPS, self.target_fps)

                # Query actual camera properties
                self._actual_width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or self.requested_width
                self._actual_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or self.requested_height

                # Warmup check: read single frame
                ret, test_frame = self._cap.read()
                if ret and test_frame is not None:
                    self._is_synthetic = False
                    self._is_opened = True
                    self._latest_frame = test_frame.copy()
                    self._latest_timestamp = time.time()
                    logger.info(
                        "Hardware camera %s opened successfully (%dx%d)",
                        self.camera_id,
                        self._actual_width,
                        self._actual_height,
                    )
                    return True
                else:
                    logger.warning("Hardware camera %s opened but read returned False", self.camera_id)
                    if self._cap is not None:
                        self._cap.release()
                        self._cap = None
        except Exception as exc:
            logger.warning("Failed to open hardware camera %s: %s", self.camera_id, exc)
            if self._cap is not None:
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None

        if self.auto_fallback:
            logger.info("Falling back to synthetic video source.")
            self._setup_synthetic()
            return True

        self._is_opened = False
        return False

    def _setup_synthetic(self) -> None:
        """Setup synthetic video generator."""
        self._is_synthetic = True
        self._synthetic_source = SyntheticFrameSource(
            width=self.requested_width,
            height=self.requested_height,
            fps=self.target_fps,
            render_target=self.render_synthetic_target,
        )
        self._actual_width = self.requested_width
        self._actual_height = self.requested_height
        self._is_opened = True

        # Generate initial test frame
        _, frame = self._synthetic_source.read()
        self._latest_frame = frame
        self._latest_timestamp = time.time()

    def start(self) -> bool:
        """Start the background frame capture thread."""
        if self._is_running:
            return True

        if not self._is_opened and not self._init_source():
            return False

        self._stop_event.clear()
        self._is_running = True
        self._thread = threading.Thread(
            target=self._capture_worker,
            name="CameraCaptureWorker",
            daemon=True,
        )
        self._thread.start()
        logger.info("Camera capture worker started for source: %s", self.camera_id)
        return True

    def _capture_worker(self) -> None:
        """Background thread grabbing frames continuously."""
        prev_time = time.time()
        fps_samples: List[float] = []

        while not self._stop_event.is_set():
            frame: Optional[np.ndarray] = None
            success = False

            if self.custom_frame_generator is not None:
                try:
                    success, frame = self.custom_frame_generator()
                except Exception as e:
                    logger.debug("Custom generator error: %s", e)
                    success = False
            elif self._is_synthetic and self._synthetic_source is not None:
                success, frame = self._synthetic_source.read()
            elif self._cap is not None and self._cap.isOpened():
                success, frame = self._cap.read()
                if not success:
                    # If hardware disconnected mid-stream and fallback enabled
                    if self.auto_fallback:
                        logger.warning("Hardware camera frame drop. Switching to synthetic fallback.")
                        self._setup_synthetic()
                        continue
                    else:
                        time.sleep(0.01)
                        continue
            else:
                time.sleep(0.01)
                continue

            if success and frame is not None:
                now = time.time()
                dt = now - prev_time
                prev_time = now

                # Sliding window FPS tracking (last 30 samples)
                if dt > 0.0:
                    fps_samples.append(1.0 / dt)
                    if len(fps_samples) > 30:
                        fps_samples.pop(0)
                    calc_fps = sum(fps_samples) / len(fps_samples)
                else:
                    calc_fps = self.target_fps

                with self._lock:
                    self._latest_frame = frame
                    self._latest_timestamp = now
                    self._frame_count += 1
                    self._current_fps = round(calc_fps, 1)

        self._is_running = False

    def read(self, timeout: float = 1.0) -> Tuple[bool, Optional[np.ndarray]]:
        """Read the latest captured frame. Thread-safe and non-blocking."""
        start_t = time.time()
        while time.time() - start_t <= timeout:
            with self._lock:
                if self._latest_frame is not None:
                    return True, self._latest_frame.copy()
            if not self._is_running:
                # If not running in thread, attempt direct single read
                if self._is_synthetic and self._synthetic_source:
                    ret, f = self._synthetic_source.read()
                    if ret:
                        return True, f
                elif self._cap is not None and self._cap.isOpened():
                    ret, f = self._cap.read()
                    if ret:
                        return True, f
            time.sleep(0.005)

        return False, None

    def get_frame(self) -> Optional[np.ndarray]:
        """Get copy of the most recent frame without blocking."""
        with self._lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy()
            return None

    def get_frame_with_timestamp(self) -> Tuple[bool, Optional[np.ndarray], float]:
        """Get latest frame alongside capture timestamp."""
        with self._lock:
            if self._latest_frame is not None:
                return True, self._latest_frame.copy(), self._latest_timestamp
            return False, None, 0.0

    def change_camera(
        self,
        camera_id: Union[int, str],
        width: Optional[int] = None,
        height: Optional[int] = None,
    ) -> bool:
        """Switch video source dynamically at runtime."""
        was_running = self._is_running
        self.stop()

        self.camera_id = camera_id
        if width is not None:
            self.requested_width = width
        if height is not None:
            self.requested_height = height

        self._init_source()
        if was_running:
            return self.start()
        return self._is_opened

    def get_telemetry(self) -> Dict[str, Any]:
        """Return operational telemetry dictionary."""
        with self._lock:
            return {
                "camera_id": self.camera_id,
                "is_synthetic": self._is_synthetic,
                "is_opened": self._is_opened,
                "is_running": self._is_running,
                "fps": self._current_fps,
                "frame_count": self._frame_count,
                "width": self._actual_width,
                "height": self._actual_height,
                "last_timestamp": self._latest_timestamp,
            }

    def stop(self) -> None:
        """Stop background capture worker thread."""
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.5)
            self._thread = None
        self._is_running = False

    def release(self) -> None:
        """Stop capture and release all camera resources."""
        self.stop()
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        self._synthetic_source = None
        self._is_opened = False
        logger.info("Camera capture released for source: %s", self.camera_id)

    def __enter__(self) -> CameraCapture:
        self.start()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.release()
