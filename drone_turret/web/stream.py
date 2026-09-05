"""
Tactical HUD Video Overlay Renderer and Low-Latency MJPEG Video Streamer.

Milestone M5 Implementation for drone_turret_v2:
- Tactical military HUD overlay rendering:
  * Drone target bounding boxes (cyan for unselected, bright red/amber for locked target).
  * Tactical corner brackets, target class name, confidence %, track ID badge.
  * Dashed cyan forward trajectory projection (0.1s - 2.0s) from 6-state Kalman filter.
  * Ballistic Lead Point crosshair (distinct diamond marker with time-to-intercept readout).
  * Turret Aim reticle (blue/white circle crosshair showing servo orientation).
  * Comprehensive telemetry HUD banner (FPS, range, speed in km/h, status badge, pan/tilt).
- Low-latency MJPEG multipart streaming generator (multipart/x-mixed-replace).
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Generator, List, Optional, Tuple, Union

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# Tactical HUD Color Palette (BGR format)
COLOR_BG_DARK = (15, 18, 22)
COLOR_CYAN = (255, 240, 0)         # Primary tactical cyan / bright blue
COLOR_GREEN = (0, 255, 128)        # Normal tracking green
COLOR_AMBER = (0, 180, 255)        # Warning / Coasting amber
COLOR_RED = (40, 40, 255)          # Locked target alert red
COLOR_ORANGE_LEAD = (0, 140, 255)  # Ballistic intercept lead diamond
COLOR_BLUE_RETICLE = (255, 180, 50)# Turret aim reticle blue
COLOR_WHITE = (240, 240, 240)
COLOR_GRAY = (120, 120, 120)
COLOR_DARK_PANEL = (20, 25, 30)


class TacticalHUDOverlay:
    """
    Renders military-grade tactical HUD cyber graphics onto video frames.
    
    Overlays:
    - Bounding boxes with tactical corner brackets & ID badges.
    - Future kinematic trajectory predictions (dashed cyan curves).
    - Ballistic intercept lead point (orange diamond with time-to-intercept readout).
    - Physical/Simulated turret aim reticle (blue/white circle crosshair).
    - Real-time telemetry badges (FPS, Range, Speed km/h, Pan/Tilt angles, Lock Status).
    """

    def __init__(
        self,
        show_reticle: bool = True,
        show_trajectory: bool = True,
        show_lead_point: bool = True,
        show_telemetry: bool = True,
        show_grid: bool = True,
    ):
        self.show_reticle = show_reticle
        self.show_trajectory = show_trajectory
        self.show_lead_point = show_lead_point
        self.show_telemetry = show_telemetry
        self.show_grid = show_grid

    def render(
        self,
        frame: np.ndarray,
        detections: Optional[List[Any]] = None,
        locked_target: Optional[Any] = None,
        target_state: Optional[Any] = None,
        intercept_solution: Optional[Any] = None,
        current_angles: Optional[Tuple[float, float]] = None,
        telemetry: Optional[Dict[str, Any]] = None,
    ) -> np.ndarray:
        """
        Draws tactical HUD elements on top of the provided frame.
        
        Args:
            frame: Input BGR image (H x W x 3).
            detections: List of Detection objects.
            locked_target: Currently locked Detection object or None.
            target_state: 6-State TargetState from Kalman filter or None.
            intercept_solution: Ballistic InterceptSolution or None.
            current_angles: (current_pan_deg, current_tilt_deg) of turret servos.
            telemetry: Dictionary containing operational telemetry (fps, speed, range, status, etc.).
            
        Returns:
            Annotated BGR frame copy.
        """
        if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
            return frame

        canvas = frame.copy()
        h, w = canvas.shape[:2]
        cx, cy = w // 2, h // 2

        # 1. Subtle tactical background grid & artificial horizon
        if self.show_grid:
            self._draw_tactical_grid(canvas, w, h, cx, cy)

        # 2. Central boresight / Turret aim reticle
        if self.show_reticle:
            self._draw_aim_reticle(canvas, w, h, cx, cy, current_angles)

        # 3. Trajectory projection from Kalman state
        if self.show_trajectory and target_state is not None:
            self._draw_trajectory(canvas, target_state, w, h)

        # 4. Drone target detections & bounding boxes
        if detections:
            self._draw_detections(canvas, detections, locked_target)

        # 5. Ballistic intercept lead point crosshair & time readout
        if self.show_lead_point and intercept_solution is not None:
            self._draw_lead_point(canvas, intercept_solution, w, h)

        # 6. Telemetry HUD cards, badges & status header
        if self.show_telemetry:
            self._draw_telemetry_hud(
                canvas,
                w,
                h,
                target_state=target_state,
                intercept_solution=intercept_solution,
                locked_target=locked_target,
                current_angles=current_angles,
                telemetry=telemetry or {},
            )

        return canvas

    def _draw_tactical_grid(self, canvas: np.ndarray, w: int, h: int, cx: int, cy: int) -> None:
        """Draws subtle tactical grid lines, corner brackets, and pitch/roll markers."""
        # Viewport corner brackets
        bracket_len = 24
        bracket_pad = 12
        color = (80, 80, 80)

        # Top-left
        cv2.line(canvas, (bracket_pad, bracket_pad), (bracket_pad + bracket_len, bracket_pad), color, 1)
        cv2.line(canvas, (bracket_pad, bracket_pad), (bracket_pad, bracket_pad + bracket_len), color, 1)
        # Top-right
        cv2.line(canvas, (w - bracket_pad, bracket_pad), (w - bracket_pad - bracket_len, bracket_pad), color, 1)
        cv2.line(canvas, (w - bracket_pad, bracket_pad), (w - bracket_pad, bracket_pad + bracket_len), color, 1)
        # Bottom-left
        cv2.line(canvas, (bracket_pad, h - bracket_pad), (bracket_pad + bracket_len, h - bracket_pad), color, 1)
        cv2.line(canvas, (bracket_pad, h - bracket_pad), (bracket_pad, h - bracket_pad - bracket_len), color, 1)
        # Bottom-right
        cv2.line(canvas, (w - bracket_pad, h - bracket_pad), (w - bracket_pad - bracket_len, h - bracket_pad), color, 1)
        cv2.line(canvas, (w - bracket_pad, h - bracket_pad), (w - bracket_pad, h - bracket_pad - bracket_len), color, 1)

        # Center horizon cross ticks
        tick_color = (60, 60, 60)
        cv2.line(canvas, (cx - 40, cy), (cx - 15, cy), tick_color, 1)
        cv2.line(canvas, (cx + 15, cy), (cx + 40, cy), tick_color, 1)
        cv2.line(canvas, (cx, cy - 40), (cx, cy - 15), tick_color, 1)
        cv2.line(canvas, (cx, cy + 15), (cx, cy + 40), tick_color, 1)

    def _draw_aim_reticle(
        self,
        canvas: np.ndarray,
        w: int,
        h: int,
        cx: int,
        cy: int,
        current_angles: Optional[Tuple[float, float]] = None,
    ) -> None:
        """Draws the turret aim crosshair reticle representing current servo aim."""
        reticle_x, reticle_y = cx, cy

        # If current pan/tilt offset is provided, project onto screen center
        if current_angles is not None:
            pan_deg, tilt_deg = current_angles
            # 90, 90 is center
            pan_offset = pan_deg - 90.0
            tilt_offset = tilt_deg - 90.0
            # Scale degrees to pixels (~8 px per degree for 70 deg FOV on 640px)
            scale = w / 70.0
            reticle_x = int(cx + pan_offset * scale)
            reticle_y = int(cy - tilt_offset * scale)
            reticle_x = max(20, min(w - 20, reticle_x))
            reticle_y = max(20, min(h - 20, reticle_y))

        # Aim circle with 4 outer tick marks
        color = COLOR_BLUE_RETICLE
        r = 18
        cv2.circle(canvas, (reticle_x, reticle_y), r, color, 1, cv2.LINE_AA)
        cv2.circle(canvas, (reticle_x, reticle_y), 3, color, -1)

        # Reticle tick lines
        cv2.line(canvas, (reticle_x - r - 8, reticle_y), (reticle_x - r, reticle_y), color, 1, cv2.LINE_AA)
        cv2.line(canvas, (reticle_x + r, reticle_y), (reticle_x + r + 8, reticle_y), color, 1, cv2.LINE_AA)
        cv2.line(canvas, (reticle_x, reticle_y - r - 8), (reticle_x, reticle_y - r), color, 1, cv2.LINE_AA)
        cv2.line(canvas, (reticle_x, reticle_y + r), (reticle_x, reticle_y + r + 8), color, 1, cv2.LINE_AA)

        # Label
        cv2.putText(
            canvas,
            "TURRET AIM",
            (reticle_x + r + 10, reticle_y + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            color,
            1,
            cv2.LINE_AA,
        )

    def _draw_detections(
        self,
        canvas: np.ndarray,
        detections: List[Any],
        locked_target: Optional[Any] = None,
    ) -> None:
        """Draws tactical corner-bracketed bounding boxes and target identity tags."""
        locked_tid = getattr(locked_target, "track_id", None) if locked_target else None

        for det in detections:
            bbox = getattr(det, "box", None)
            if bbox is None or len(bbox) != 4:
                continue

            x1, y1, x2, y2 = [int(v) for v in bbox]
            tid = getattr(det, "track_id", None)
            conf = getattr(det, "confidence", 1.0)
            cls_name = getattr(det, "class_name", "Drone")

            is_locked = (locked_target is not None and (det == locked_target or (tid is not None and tid == locked_tid)))

            box_color = COLOR_RED if is_locked else COLOR_GREEN
            line_thickness = 2 if is_locked else 1

            # Draw tactical corner brackets instead of plain rectangle
            self._draw_corner_brackets(canvas, x1, y1, x2, y2, box_color, line_thickness)

            # Center target pip
            mid_x = (x1 + x2) // 2
            mid_y = (y1 + y2) // 2
            cv2.circle(canvas, (mid_x, mid_y), 3, box_color, -1)

            # Target tag label
            tag_tid = f"TRK #{tid}" if tid is not None else "TRK --"
            tag_text = f"[{tag_tid}] {cls_name.upper()} {conf:.0%}"
            if is_locked:
                tag_text = f"★ LOCKED {tag_text}"

            (tw, th), tb = cv2.getTextSize(tag_text, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)
            tag_y1 = max(0, y1 - th - 8)
            tag_y2 = y1

            # Background banner
            cv2.rectangle(canvas, (x1, tag_y1), (x1 + tw + 8, tag_y2), box_color, -1)
            cv2.putText(
                canvas,
                tag_text,
                (x1 + 4, tag_y2 - 3),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.40,
                (0, 0, 0) if not is_locked else (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

    def _draw_corner_brackets(
        self,
        canvas: np.ndarray,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        color: Tuple[int, int, int],
        thickness: int,
    ) -> None:
        """Draws 4 corner brackets for high-tech tactical appearance."""
        bw = max(4, (x2 - x1) // 4)
        bh = max(4, (y2 - y1) // 4)
        bw = min(bw, 16)
        bh = min(bh, 16)

        # Top-left
        cv2.line(canvas, (x1, y1), (x1 + bw, y1), color, thickness)
        cv2.line(canvas, (x1, y1), (x1, y1 + bh), color, thickness)
        # Top-right
        cv2.line(canvas, (x2, y1), (x2 - bw, y1), color, thickness)
        cv2.line(canvas, (x2, y1), (x2, y1 + bh), color, thickness)
        # Bottom-left
        cv2.line(canvas, (x1, y2), (x1 + bw, y2), color, thickness)
        cv2.line(canvas, (x1, y2), (x1, y2 - bh), color, thickness)
        # Bottom-right
        cv2.line(canvas, (x2, y2), (x2 - bw, y2), color, thickness)
        cv2.line(canvas, (x2, y2), (x2, y2 - bh), color, thickness)

    def _draw_trajectory(self, canvas: np.ndarray, target_state: Any, w: int, h: int) -> None:
        """Draws dashed cyan trajectory line projected forward into the future."""
        # Extract sampled 2D trajectory points from Kalman state
        traj_points: List[Tuple[float, float]] = []

        if hasattr(target_state, "trajectory_2d") and target_state.trajectory_2d:
            traj_points = [(float(p[0]), float(p[1])) for p in target_state.trajectory_2d]
        elif hasattr(target_state, "pos_2d") and hasattr(target_state, "vel_2d"):
            # Compute trajectory on the fly (0.1s to 2.0s in 20 steps)
            x0, y0 = target_state.pos_2d
            vx, vy = target_state.vel_2d
            ax = getattr(target_state, "acc_2d", (0.0, 0.0))[0]
            ay = getattr(target_state, "acc_2d", (0.0, 0.0))[1]
            for i in range(21):
                t_f = (i / 20.0) * 1.5
                px = x0 + vx * t_f + 0.5 * ax * (t_f**2)
                py = y0 + vy * t_f + 0.5 * ay * (t_f**2)
                traj_points.append((px, py))

        if len(traj_points) < 2:
            return

        # Draw dashed curve segments
        color = COLOR_CYAN
        for i in range(len(traj_points) - 1):
            pt1 = (int(round(traj_points[i][0])), int(round(traj_points[i][1])))
            pt2 = (int(round(traj_points[i + 1][0])), int(round(traj_points[i + 1][1])))

            # Draw alternating dash segments
            if i % 2 == 0:
                cv2.line(canvas, pt1, pt2, color, 2, cv2.LINE_AA)
            else:
                cv2.circle(canvas, pt2, 2, color, -1)

        # Draw trajectory endpoint pip and label
        last_pt = (int(round(traj_points[-1][0])), int(round(traj_points[-1][1])))
        if 0 <= last_pt[0] < w and 0 <= last_pt[1] < h:
            cv2.circle(canvas, last_pt, 4, color, -1)
            cv2.putText(
                canvas,
                "+1.5s PROJ",
                (last_pt[0] + 8, last_pt[1] + 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                color,
                1,
                cv2.LINE_AA,
            )

    def _draw_lead_point(self, canvas: np.ndarray, intercept_solution: Any, w: int, h: int) -> None:
        """
        Draws Ballistic Intercept Lead Point diamond crosshair with time-to-intercept readout.
        """
        pixel_xy = getattr(intercept_solution, "lead_pixel_xy", None)
        if pixel_xy is None:
            return

        lead_x, lead_y = int(pixel_xy[0]), int(pixel_xy[1])
        t_int = getattr(intercept_solution, "t_intercept", 0.0)
        drop_m = getattr(intercept_solution, "drop_m", 0.0)
        reachable = getattr(intercept_solution, "reachable", True)

        color = COLOR_ORANGE_LEAD if reachable else (100, 100, 100)

        # Diamond marker vertices
        d = 12
        diamond_pts = np.array([
            [lead_x, lead_y - d],
            [lead_x + d, lead_y],
            [lead_x, lead_y + d],
            [lead_x - d, lead_y],
        ], dtype=np.int32)

        cv2.polylines(canvas, [diamond_pts], isClosed=True, color=color, thickness=2, lineType=cv2.LINE_AA)
        # Inner dot
        cv2.circle(canvas, (lead_x, lead_y), 3, color, -1)

        # Crosshair prongs outside diamond
        prong = 6
        cv2.line(canvas, (lead_x, lead_y - d - prong), (lead_x, lead_y - d), color, 1, cv2.LINE_AA)
        cv2.line(canvas, (lead_x, lead_y + d), (lead_x, lead_y + d + prong), color, 1, cv2.LINE_AA)
        cv2.line(canvas, (lead_x - d - prong, lead_y), (lead_x - d, lead_y), color, 1, cv2.LINE_AA)
        cv2.line(canvas, (lead_x + d, lead_y), (lead_x + d + prong, lead_y), color, 1, cv2.LINE_AA)

        # Time-to-intercept & drop readout badge
        badge_text = f"LEAD Tint={t_int:.2f}s"
        drop_text = f"DROP {drop_m:.2f}m"

        (tw1, th1), _ = cv2.getTextSize(badge_text, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)
        (tw2, th2), _ = cv2.getTextSize(drop_text, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)

        bx = lead_x + d + 8
        by = lead_y - 8

        # Background badge box
        box_w = max(tw1, tw2) + 8
        box_h = th1 + th2 + 12
        cv2.rectangle(canvas, (bx, by), (bx + box_w, by + box_h), (20, 24, 30), -1)
        cv2.rectangle(canvas, (bx, by), (bx + box_w, by + box_h), color, 1)

        cv2.putText(
            canvas,
            badge_text,
            (bx + 4, by + th1 + 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            color,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            drop_text,
            (bx + 4, by + th1 + th2 + 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            COLOR_WHITE,
            1,
            cv2.LINE_AA,
        )

    def _draw_telemetry_hud(
        self,
        canvas: np.ndarray,
        w: int,
        h: int,
        target_state: Optional[Any],
        intercept_solution: Optional[Any],
        locked_target: Optional[Any],
        current_angles: Optional[Tuple[float, float]],
        telemetry: Dict[str, Any],
    ) -> None:
        """Draws top header status bar and telemetry overlays."""
        # Top-left System Info
        fps_val = telemetry.get("fps", 30.0)
        model_name = telemetry.get("model_name", "drone_best.pt")

        sys_text = f"SYS: ONLINE | FPS: {fps_val:.1f} | MODEL: {model_name}"
        cv2.putText(
            canvas,
            sys_text,
            (14, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            COLOR_WHITE,
            1,
            cv2.LINE_AA,
        )

        # Top-right Lock Status Badge
        is_locked = locked_target is not None or telemetry.get("target_locked", False)
        is_coasting = getattr(target_state, "is_coasting", False) or (telemetry.get("coast_frames", 0) > 0)

        if is_locked and not is_coasting:
            badge_text = "● LOCKED"
            badge_color = COLOR_RED
        elif is_coasting:
            coast_cnt = getattr(target_state, "coast_frames", 1)
            badge_text = f"▲ COASTING ({coast_cnt})"
            badge_color = COLOR_AMBER
        else:
            badge_text = "◌ SEARCHING"
            badge_color = COLOR_CYAN

        (bw, bh), _ = cv2.getTextSize(badge_text, cv2.FONT_HERSHEY_SIMPLEX, 0.50, 2)
        badge_x = w - bw - 24
        badge_y = 22

        cv2.rectangle(canvas, (badge_x - 8, 8), (w - 12, 8 + bh + 10), (20, 24, 30), -1)
        cv2.rectangle(canvas, (badge_x - 8, 8), (w - 12, 8 + bh + 10), badge_color, 1)
        cv2.putText(
            canvas,
            badge_text,
            (badge_x, badge_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            badge_color,
            2,
            cv2.LINE_AA,
        )

        # Bottom-left Telemetry Card: Range & Speed
        range_m = telemetry.get("distance_m")
        if range_m is None and target_state is not None:
            pos3d = getattr(target_state, "pos_3d", (0.0, 0.0, 0.0))
            range_m = math.sqrt(pos3d[0]**2 + pos3d[1]**2 + pos3d[2]**2)
        if range_m is None or range_m <= 0:
            range_m = 0.0

        range_src = telemetry.get("distance_source", "OPTICAL").upper()

        speed_kmh = telemetry.get("speed_kmh")
        if speed_kmh is None and target_state is not None:
            speed_kmh = getattr(target_state, "speed_kmh", 0.0)
        if speed_kmh is None:
            speed_kmh = 0.0

        card_y = h - 50
        cv2.putText(
            canvas,
            f"TARGET RANGE: {range_m:.1f} m [{range_src}]",
            (14, card_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            COLOR_CYAN,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"TARGET SPEED: {speed_kmh:.1f} km/h",
            (14, card_y + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            COLOR_GREEN if speed_kmh < 150 else COLOR_RED,
            1,
            cv2.LINE_AA,
        )

        # Bottom-right Gimbal Servo Angles & Hardware status
        pan_val = current_angles[0] if current_angles else telemetry.get("current_pan_angle", 90.0)
        tilt_val = current_angles[1] if current_angles else telemetry.get("current_tilt_angle", 90.0)
        lead_pan = telemetry.get("lead_pan_angle", getattr(intercept_solution, "aim_pan_deg", 90.0))
        lead_tilt = telemetry.get("lead_tilt_angle", getattr(intercept_solution, "aim_tilt_deg", 90.0))

        gimbal_text = f"AIM: P {pan_val:.1f}° | T {tilt_val:.1f}°"
        lead_text = f"LEAD: P {lead_pan:.1f}° | T {lead_tilt:.1f}°"

        (gw, _), _ = cv2.getTextSize(gimbal_text, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
        (lw, _), _ = cv2.getTextSize(lead_text, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
        max_rw = max(gw, lw)

        rx = w - max_rw - 18
        cv2.putText(
            canvas,
            gimbal_text,
            (rx, card_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            COLOR_WHITE,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            lead_text,
            (rx, card_y + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            COLOR_ORANGE_LEAD,
            1,
            cv2.LINE_AA,
        )


class MJPEGStreamer:
    """
    Low-latency multipart/x-mixed-replace frame streamer for live web browser display.
    """

    def __init__(self, jpeg_quality: int = 80, target_fps: float = 30.0):
        self.jpeg_quality = max(10, min(100, int(jpeg_quality)))
        self.target_fps = max(1.0, float(target_fps))
        self.frame_interval = 1.0 / self.target_fps
        self._encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]

    def encode_frame(self, frame: np.ndarray) -> bytes:
        """Compresses BGR frame to JPEG byte payload."""
        if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
            # Generate fallback black image
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                dummy,
                "NO VIDEO SIGNAL",
                (200, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
            _, encoded = cv2.imencode(".jpg", dummy, self._encode_param)
            return encoded.tobytes()

        ret, encoded = cv2.imencode(".jpg", frame, self._encode_param)
        if ret and encoded is not None:
            return encoded.tobytes()

        # Fallback
        return b""

    def format_mjpeg_chunk(self, jpeg_bytes: bytes) -> bytes:
        """Wraps JPEG bytes into multipart/x-mixed-replace boundary protocol."""
        return (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Content-Length: " + str(len(jpeg_bytes)).encode("ascii") + b"\r\n\r\n"
            + jpeg_bytes
            + b"\r\n"
        )

    def generate_stream(
        self,
        frame_source: Callable[[], Optional[np.ndarray]],
        stop_check: Optional[Callable[[], bool]] = None,
        max_frames: Optional[int] = None,
    ) -> Generator[bytes, None, None]:
        """
        Yields multipart MJPEG stream chunks indefinitely or up to max_frames.
        
        Args:
            frame_source: Callable returning the latest BGR frame.
            stop_check: Optional callable returning True when stream should end.
            max_frames: Optional maximum number of frames to yield before terminating.
        """
        last_time = time.time()
        yielded_count = 0

        while True:
            if stop_check is not None and stop_check():
                break

            if max_frames is not None and yielded_count >= max_frames:
                break

            now = time.time()
            elapsed = now - last_time
            sleep_needed = self.frame_interval - elapsed
            if sleep_needed > 0:
                time.sleep(sleep_needed)
            last_time = time.time()

            try:
                frame = frame_source()
            except Exception as err:
                logger.debug("Error retrieving frame for MJPEG stream: %s", err)
                frame = None

            jpeg_bytes = self.encode_frame(frame)
            if jpeg_bytes:
                yielded_count += 1
                yield self.format_mjpeg_chunk(jpeg_bytes)
