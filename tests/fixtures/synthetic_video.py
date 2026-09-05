"""Synthetic Video Generator & 3D Target Kinematics for Drone Turret v2 Testing.

Generates deterministic OpenCV video frames, 3D target trajectories,
ground-truth annotations, and tactical scenario streams for automated testing.
"""

from __future__ import annotations

import math
from typing import Dict, Generator, List, Optional, Tuple, Any
import cv2
import numpy as np


class TargetKinematics:
    """Represents a 3D kinematic target with physics state and 2D pinhole projection.
    
    Coordinate Frame:
    - X: Horizontal right (+X = right, -X = left) in meters
    - Y: Vertical up (+Y = up, -Y = down) in meters
    - Z: Range / depth from turret (+Z = away from turret) in meters
    """

    def __init__(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 50.0,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        ax: float = 0.0,
        ay: float = 0.0,
        az: float = 0.0,
        real_size: float = 0.40,  # 40cm drone diameter (e.g. DJI Mavic class)
        name: str = "drone",
    ):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.vx = float(vx)
        self.vy = float(vy)
        self.vz = float(vz)
        self.ax = float(ax)
        self.ay = float(ay)
        self.az = float(az)
        self.real_size = float(real_size)
        self.name = name
        self.elapsed_time = 0.0

    @property
    def speed_mps(self) -> float:
        """Magnitude of 3D velocity in m/s."""
        return math.sqrt(self.vx**2 + self.vy**2 + self.vz**2)

    @property
    def speed_kmh(self) -> float:
        """Magnitude of 3D velocity in km/h."""
        return self.speed_mps * 3.6

    def update(self, dt: float) -> None:
        """Step kinematics forward in time by dt seconds."""
        # Update velocities
        self.vx += self.ax * dt
        self.vy += self.ay * dt
        self.vz += self.az * dt
        # Update positions
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt
        self.elapsed_time += dt

    def project_to_camera(
        self,
        f: float = 800.0,
        cx: float = 320.0,
        cy: float = 240.0,
        min_pixel_size: int = 4,
    ) -> Tuple[int, int, int, int]:
        """Project 3D position and size onto 2D image coordinates (x1, y1, x2, y2)."""
        safe_z = max(0.1, self.z)
        px = cx + (f * self.x / safe_z)
        py = cy - (f * self.y / safe_z)  # Inverted Y for screen space

        pixel_size = max(min_pixel_size, int(f * self.real_size / safe_z))
        half_w = pixel_size // 2
        half_h = pixel_size // 2

        x1 = int(round(px - half_w))
        y1 = int(round(py - half_h))
        x2 = int(round(px + half_w))
        y2 = int(round(py + half_h))
        return (x1, y1, x2, y2)

    def get_center_pixel(
        self, f: float = 800.0, cx: float = 320.0, cy: float = 240.0
    ) -> Tuple[int, int]:
        """Get (cx_px, cy_px) projected centroid in image coordinates."""
        safe_z = max(0.1, self.z)
        px = int(round(cx + (f * self.x / safe_z)))
        py = int(round(cy - (f * self.y / safe_z)))
        return (px, py)


class SyntheticVideoGenerator:
    """Generates synthetic OpenCV frames and ground-truth streams for drone turret testing."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: float = 30.0,
        focal_length: float = 800.0,
    ):
        self.width = width
        self.height = height
        self.fps = fps
        self.dt = 1.0 / fps
        self.focal_length = focal_length
        self.cx = width / 2.0
        self.cy = height / 2.0
        self.frame_idx = 0

    def reset(self) -> None:
        """Reset internal frame counter."""
        self.frame_idx = 0

    def render_drone_sprite(
        self,
        frame: np.ndarray,
        bbox: Tuple[int, int, int, int],
        color: Tuple[int, int, int] = (200, 200, 200),
    ) -> None:
        """Draws a synthetic multirotor drone representation onto the frame."""
        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        w = max(4, x2 - x1)
        h = max(4, y2 - y1)

        # Central fuselage
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
        cv2.circle(frame, (cx, cy), max(2, min(w, h) // 4), (0, 0, 255), -1)

        # Motor arms / X-frame
        cv2.line(frame, (x1, y1), (x2, y2), (120, 120, 120), 1)
        cv2.line(frame, (x1, y2), (x2, y1), (120, 120, 120), 1)

        # 4 Rotor hubs
        rotor_r = max(2, min(w, h) // 6)
        cv2.circle(frame, (x1, y1), rotor_r, (0, 255, 255), -1)
        cv2.circle(frame, (x2, y1), rotor_r, (0, 255, 255), -1)
        cv2.circle(frame, (x1, y2), rotor_r, (0, 255, 255), -1)
        cv2.circle(frame, (x2, y2), rotor_r, (0, 255, 255), -1)

    def generate_single_frame(
        self,
        targets: List[TargetKinematics],
        is_occluded: bool = False,
        noise_sigma: float = 0.0,
        bright_flash: bool = False,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Renders one single frame and returns (frame, metadata)."""
        self.frame_idx += 1
        t = self.frame_idx * self.dt

        # Base tactical night/sky background
        if bright_flash:
            frame = np.full((self.height, self.width, 3), 245, dtype=np.uint8)
        else:
            frame = np.full((self.height, self.width, 3), 30, dtype=np.uint8)
            # Faint artificial horizon line
            cv2.line(
                frame,
                (0, self.height // 2),
                (self.width, self.height // 2),
                (45, 45, 45),
                1,
            )

        gt_targets = []
        for tid, target in enumerate(targets):
            x1, y1, x2, y2 = target.project_to_camera(
                self.focal_length, self.cx, self.cy
            )
            c_x, c_y = target.get_center_pixel(self.focal_length, self.cx, self.cy)

            is_visible = (
                not is_occluded
                and x2 > 0
                and x1 < self.width
                and y2 > 0
                and y1 < self.height
                and target.z > 0.5
            )

            if is_visible:
                self.render_drone_sprite(frame, (x1, y1, x2, y2))

            gt_targets.append(
                {
                    "track_id": tid + 1,
                    "name": target.name,
                    "bbox": [x1, y1, x2, y2],
                    "center": [c_x, c_y],
                    "pos_3d": [target.x, target.y, target.z],
                    "vel_3d": [target.vx, target.vy, target.vz],
                    "acc_3d": [target.ax, target.ay, target.az],
                    "speed_kmh": target.speed_kmh,
                    "distance_m": target.z,
                    "visible": is_visible,
                    "occluded": is_occluded,
                }
            )

        if noise_sigma > 0:
            noise = np.random.normal(0, noise_sigma, frame.shape).astype(np.float32)
            frame = np.clip(frame.astype(np.float32) + noise, 0, 255).astype(np.uint8)

        metadata = {
            "frame_id": self.frame_idx,
            "timestamp": t,
            "dt": self.dt,
            "targets": gt_targets,
        }
        return frame, metadata

    def stream_scenario(
        self,
        targets: List[TargetKinematics],
        total_frames: int = 60,
        occlusion_spans: Optional[List[Tuple[int, int]]] = None,
        noise_sigma: float = 0.0,
    ) -> Generator[Tuple[np.ndarray, Dict[str, Any]], None, None]:
        """Generator yielding (frame, metadata) across time."""
        occlusion_spans = occlusion_spans or []
        for _ in range(total_frames):
            is_occluded = any(
                start <= (self.frame_idx + 1) <= end for start, end in occlusion_spans
            )
            for target in targets:
                target.update(self.dt)
            frame, meta = self.generate_single_frame(
                targets, is_occluded=is_occluded, noise_sigma=noise_sigma
            )
            yield frame, meta

    # ---------------- Pre-built Tactical Scenarios -----------------

    @classmethod
    def create_flyby_scenario(
        cls,
        speed_kmh: float = 200.0,
        distance_m: float = 50.0,
        y_m: float = 0.0,
        total_frames: int = 60,
    ) -> Tuple[SyntheticVideoGenerator, List[TargetKinematics]]:
        """Orthogonal high-speed flyby from left to right at constant distance."""
        gen = cls()
        vx_mps = (speed_kmh / 3.6)
        # Start so target crosses near the center around half-time
        start_x = -vx_mps * (total_frames * gen.dt / 2.0)
        target = TargetKinematics(
            x=start_x, y=y_m, z=distance_m, vx=vx_mps, vy=0.0, vz=0.0
        )
        return gen, [target]

    @classmethod
    def create_headon_scenario(
        cls,
        start_dist: float = 80.0,
        speed_kmh: float = 150.0,
        y_m: float = 5.0,
        total_frames: int = 60,
    ) -> Tuple[SyntheticVideoGenerator, List[TargetKinematics]]:
        """Head-on fast diving approach towards the turret."""
        gen = cls()
        vz_mps = -(speed_kmh / 3.6)
        vy_mps = -(y_m / (total_frames * gen.dt))
        target = TargetKinematics(
            x=0.0, y=y_m, z=start_dist, vx=0.0, vy=vy_mps, vz=vz_mps
        )
        return gen, [target]

    @classmethod
    def create_zigzag_scenario(
        cls,
        distance_m: float = 40.0,
        speed_kmh: float = 90.0,
        freq_hz: float = 0.8,
        total_frames: int = 90,
    ) -> Tuple[SyntheticVideoGenerator, List[TargetKinematics]]:
        """Sinusoidal high-G evasive maneuver with oscillating acceleration."""
        gen = cls()
        vx_peak = (speed_kmh / 3.6)
        # We model this with a sinusoidal update in the test or with initial kinematics
        target = TargetKinematics(
            x=-10.0, y=0.0, z=distance_m, vx=vx_peak, vy=0.0, vz=0.0
        )
        return gen, [target]

    @classmethod
    def create_occlusion_scenario(
        cls,
        speed_kmh: float = 100.0,
        distance_m: float = 45.0,
        occlusion_frames: Tuple[int, int] = (25, 40),
        total_frames: int = 70,
    ) -> Tuple[SyntheticVideoGenerator, List[TargetKinematics], List[Tuple[int, int]]]:
        """Target undergoes total visual loss for a span of frames."""
        gen = cls()
        vx_mps = speed_kmh / 3.6
        start_x = -vx_mps * (total_frames * gen.dt / 2.0)
        target = TargetKinematics(
            x=start_x, y=2.0, z=distance_m, vx=vx_mps, vy=0.0, vz=0.0
        )
        return gen, [target], [occlusion_frames]

    @classmethod
    def create_crossing_scenario(
        cls, distance_m: float = 50.0, total_frames: int = 60
    ) -> Tuple[SyntheticVideoGenerator, List[TargetKinematics]]:
        """Two targets crossing trajectories in the field of view."""
        gen = cls()
        t1 = TargetKinematics(
            x=-20.0, y=2.0, z=distance_m, vx=20.0, vy=-1.0, vz=0.0, name="target_alpha"
        )
        t2 = TargetKinematics(
            x=20.0, y=-2.0, z=distance_m + 2.0, vx=-20.0, vy=1.0, vz=0.0, name="target_beta"
        )
        return gen, [t1, t2]
