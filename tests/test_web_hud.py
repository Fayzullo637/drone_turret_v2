"""
Comprehensive Unit & Integration Test Suite for Milestone M5 Web Backend & Tactical HUD.

Verifies:
1. Package exports in `drone_turret.web`.
2. `TacticalHUDOverlay` rendering across all visual elements (bounding boxes, Kalman trajectory,
   ballistic lead diamond, turret aim reticle, and telemetry cards).
3. `MJPEGStreamer` multipart chunk formatting and streaming generator.
4. FastAPI REST API endpoints:
   - GET / (HTML frontend)
   - GET /video_feed (MJPEG live stream)
   - GET /api/cameras (camera discovery)
   - GET /api/serial-ports & GET /api/hardware/ports (COM ports)
   - GET & POST /api/config (dynamic parameter tuning & boundary validation)
   - GET /api/status (comprehensive telemetry)
   - POST /api/turret/fire & /api/fire (pneumatic solenoid trigger)
   - POST /api/turret/home (servo homing)
   - POST /api/turret/lock (track locking)
5. WebSocket /ws/telemetry and /ws/control bidirectional communication.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import pytest
from fastapi.testclient import TestClient

from drone_turret.ballistics.calculator import InterceptSolution
from drone_turret.tracking.kalman_filter import FilterStatus, TargetState
from drone_turret.vision.detector import Detection
from drone_turret.web import (
    MJPEGStreamer,
    TacticalHUDOverlay,
    TurretConfigModel,
    TurretStateModel,
    app,
    create_app,
)
from drone_turret.web.app import PipelineCoordinator


@pytest.fixture
def web_client() -> TestClient:
    """Provides a fresh FastAPI test client with standalone coordinator."""
    test_app = create_app()
    with TestClient(test_app) as client:
        yield client


# =============================================================================
# 1. Package & Module Exports
# =============================================================================

def test_web_package_exports():
    """Verifies that drone_turret.web exports all key interfaces."""
    import drone_turret.web as web_pkg

    assert hasattr(web_pkg, "create_app")
    assert hasattr(web_pkg, "app")
    assert hasattr(web_pkg, "TacticalHUDOverlay")
    assert hasattr(web_pkg, "MJPEGStreamer")
    assert hasattr(web_pkg, "TurretConfigModel")
    assert hasattr(web_pkg, "TurretStateModel")


# =============================================================================
# 2. Tactical HUD Overlay Unit Tests
# =============================================================================

def test_hud_overlay_blank_frame():
    """Verifies overlay renders without crash on blank image."""
    overlay = TacticalHUDOverlay()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    rendered = overlay.render(frame)

    assert rendered is not None
    assert rendered.shape == (480, 640, 3)
    assert not np.array_equal(rendered, frame)  # Grid and reticle drawn


def test_hud_overlay_detections_rendering():
    """Verifies rendering of unlocked and locked target bounding boxes and tags."""
    overlay = TacticalHUDOverlay()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    det_unlocked = Detection(
        box=(50, 50, 150, 150),
        confidence=0.88,
        class_id=0,
        class_name="Drone",
        track_id=1,
    )
    det_locked = Detection(
        box=(200, 200, 320, 320),
        confidence=0.96,
        class_id=0,
        class_name="DJI Mavic",
        track_id=2,
    )

    rendered = overlay.render(
        frame=frame,
        detections=[det_unlocked, det_locked],
        locked_target=det_locked,
    )

    assert rendered is not None
    assert rendered.shape == (480, 640, 3)
    # Target pixels should have green/red colors
    assert np.any(rendered[50:150, 50:150] > 0)
    assert np.any(rendered[200:320, 200:320] > 0)


def test_hud_overlay_trajectory_rendering():
    """Verifies forward trajectory curve rendering from Kalman state."""
    overlay = TacticalHUDOverlay()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    sample_traj = [(100.0 + i * 10, 200.0 + i * 5) for i in range(21)]
    target_state = TargetState(
        pos_2d=(100.0, 200.0),
        vel_2d=(100.0, 50.0),
        acc_2d=(0.0, 0.0),
        pos_3d=(0.0, 0.0, 25.0),
        vel_3d=(10.0, 5.0, 0.0),
        speed_kmh=40.0,
        is_coasting=False,
        coast_frames=0,
        status=FilterStatus.TRACKING,
        trajectory_2d=sample_traj,
    )

    rendered = overlay.render(frame=frame, target_state=target_state)
    assert rendered is not None
    # Verify cyan trajectory pixels are drawn along the curve
    assert np.any(rendered[190:310, 90:310] > 0)


def test_hud_overlay_lead_point_rendering():
    """Verifies Ballistic Lead diamond crosshair and time-to-intercept readout."""
    overlay = TacticalHUDOverlay()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    intercept_sol = InterceptSolution(
        reachable=True,
        t_intercept=0.42,
        lead_pos_3d=(2.5, 1.2, 30.0),
        aim_pan_deg=104.5,
        aim_tilt_deg=82.3,
        lead_pixel_xy=(380, 180),
        drop_m=0.86,
    )

    rendered = overlay.render(frame=frame, intercept_solution=intercept_sol)
    assert rendered is not None
    # Verify pixels around (380, 180) contain orange lead marker
    lead_crop = rendered[170:190, 370:390]
    assert np.any(lead_crop > 0)


def test_hud_overlay_telemetry_banner():
    """Verifies rendering of full telemetry header and cards."""
    overlay = TacticalHUDOverlay()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    telemetry = {
        "fps": 29.8,
        "speed_kmh": 145.2,
        "distance_m": 34.5,
        "distance_source": "LIDAR",
        "target_locked": True,
        "tracking_state": "LOCKED",
        "model_name": "drone_best.pt",
        "current_pan_angle": 105.0,
        "current_tilt_angle": 83.0,
        "lead_pan_angle": 106.2,
        "lead_tilt_angle": 84.1,
    }

    rendered = overlay.render(frame=frame, telemetry=telemetry)
    assert rendered is not None
    # Top header bar text and bottom cards
    assert np.any(rendered[10:30, 10:200] > 0)
    assert np.any(rendered[430:475, 10:250] > 0)


# =============================================================================
# 3. MJPEG Streamer Unit Tests
# =============================================================================

def test_mjpeg_streamer_encoding():
    """Verifies frame compression and multipart chunk formatting."""
    streamer = MJPEGStreamer(jpeg_quality=75, target_fps=30.0)
    test_img = np.full((240, 320, 3), 128, dtype=np.uint8)

    jpeg_bytes = streamer.encode_frame(test_img)
    assert len(jpeg_bytes) > 0
    # Check JPEG SOI (0xFF 0xD8) magic bytes
    assert jpeg_bytes[:2] == b"\xff\xd8"

    chunk = streamer.format_mjpeg_chunk(jpeg_bytes)
    assert b"--frame\r\n" in chunk
    assert b"Content-Type: image/jpeg\r\n" in chunk
    assert b"Content-Length: " in chunk


def test_mjpeg_streamer_generator():
    """Verifies that generator yields multiple chunks properly."""
    streamer = MJPEGStreamer(jpeg_quality=60, target_fps=60.0)
    test_img = np.zeros((100, 100, 3), dtype=np.uint8)

    count = 0
    gen = streamer.generate_stream(lambda: test_img)
    for part in gen:
        count += 1
        assert b"--frame" in part
        if count >= 3:
            break
    assert count == 3


# =============================================================================
# 4. FastAPI REST API Endpoint Tests
# =============================================================================

def test_rest_index_endpoint(web_client):
    """Verifies GET / returns HTTP 200 with HTML tactical HUD."""
    res = web_client.get("/")
    assert res.status_code == 200
    assert "DRONE" in res.text or "Tactical HUD" in res.text


def test_rest_video_feed_stream(web_client):
    """Verifies GET /video_feed returns multipart/x-mixed-replace live stream."""
    with web_client.stream("GET", "/video_feed?max_frames=2") as res:
        assert res.status_code == 200
        content_type = res.headers.get("content-type", "")
        assert "multipart/x-mixed-replace" in content_type
        assert "boundary=frame" in content_type
        found_frame = False
        for chunk in res.iter_raw():
            if b"--frame" in chunk:
                found_frame = True
                break
        assert found_frame is True


def test_rest_get_cameras(web_client):
    """Verifies GET /api/cameras returns detected and synthetic cameras."""
    res = web_client.get("/api/cameras")
    assert res.status_code == 200
    cameras = res.json()
    assert isinstance(cameras, list)
    assert len(cameras) >= 1
    # Should include synthetic source
    ids = [c["id"] for c in cameras]
    assert "synthetic" in ids or 0 in ids


def test_rest_get_serial_ports(web_client):
    """Verifies GET /api/serial-ports and GET /api/hardware/ports return COM ports."""
    res1 = web_client.get("/api/serial-ports")
    assert res1.status_code == 200
    assert isinstance(res1.json(), list)

    res2 = web_client.get("/api/hardware/ports")
    assert res2.status_code == 200
    data = res2.json()
    assert "ports" in data
    port_names = [p["port"] for p in data["ports"]]
    assert "SIMULATION" in port_names


def test_rest_get_and_post_config(web_client):
    """Verifies dynamic config retrieval and parameter mutation."""
    # 1. GET initial config
    res_get = web_client.get("/api/config")
    assert res_get.status_code == 200
    initial_cfg = res_get.json()
    assert "confidence" in initial_cfg
    assert "muzzle_velocity" in initial_cfg
    assert "net_cd" in initial_cfg

    # 2. POST valid update
    new_cfg = {
        "confidence": 0.72,
        "muzzle_velocity": 92.0,
        "net_mass": 0.55,
        "net_cd": 1.35,
        "pid_kp": 0.15,
        "model_name": "drone_best.pt",
    }
    res_post = web_client.post("/api/config", json=new_cfg)
    assert res_post.status_code == 200
    assert res_post.json()["status"] == "ok"
    updated_cfg = res_post.json()["config"]
    assert updated_cfg["confidence"] == 0.72
    assert updated_cfg["muzzle_velocity"] == 92.0
    assert updated_cfg["net_cd"] == 1.35
    assert updated_cfg["model_name"] == "drone_best.pt"


def test_rest_config_validation_boundaries(web_client):
    """Verifies out-of-bounds parameter updates produce HTTP 422 Unprocessable Entity."""
    # Negative confidence
    res = web_client.post("/api/config", json={"confidence": -0.1})
    assert res.status_code == 422

    # Confidence > 1.0
    res = web_client.post("/api/config", json={"confidence": 1.5})
    assert res.status_code == 422

    # Non-positive muzzle velocity
    res = web_client.post("/api/config", json={"muzzle_velocity": 0.0})
    assert res.status_code == 422

    # Extreme net Cd
    res = web_client.post("/api/config", json={"net_cd": 3.5})
    assert res.status_code == 422


def test_rest_get_status_telemetry(web_client):
    """Verifies GET /api/status returns comprehensive system telemetry."""
    res = web_client.get("/api/status")
    assert res.status_code == 200
    data = res.json()
    assert "fps" in data
    assert "pan_angle" in data
    assert "tilt_angle" in data
    assert "lead_pan_angle" in data
    assert "lead_tilt_angle" in data
    assert "distance_m" in data
    assert "speed_kmh" in data
    assert "target_locked" in data
    assert "tracking_state" in data
    assert "simulation_mode" in data
    assert "fire_count" in data


def test_rest_fire_and_home_actions(web_client):
    """Verifies POST /api/turret/fire, /api/fire, and /api/turret/home."""
    # Fire
    res_fire = web_client.post("/api/turret/fire")
    assert res_fire.status_code == 200
    assert res_fire.json()["status"] == "fired"

    # Fire alias
    res_alias = web_client.post("/api/fire")
    assert res_alias.status_code == 200
    assert res_alias.json()["status"] == "fired"

    # Home
    res_home = web_client.post("/api/turret/home")
    assert res_home.status_code == 200
    assert res_home.json()["status"] == "homed"
    assert res_home.json()["pan_angle"] == 90.0
    assert res_home.json()["tilt_angle"] == 90.0


def test_rest_lock_target_action(web_client):
    """Verifies POST /api/turret/lock sets locked target track ID."""
    res = web_client.post("/api/turret/lock", json={"track_id": 4, "auto_lock": True})
    assert res.status_code == 200
    assert res.json()["status"] == "ok"
    assert res.json()["locked_track_id"] == 4


# =============================================================================
# 5. WebSocket Telemetry & Control Tests
# =============================================================================

def test_websocket_telemetry_stream(web_client):
    """Verifies WebSocket /ws/telemetry broadcasts live JSON telemetry packets."""
    with web_client.websocket_connect("/ws/telemetry") as ws:
        data = ws.receive_json()
        assert isinstance(data, dict)
        assert "fps" in data
        assert "pan_angle" in data
        assert "speed_kmh" in data
        assert "tracking_state" in data


def test_websocket_control_commands(web_client):
    """Verifies bidirectional control over WebSocket /ws/control."""
    with web_client.websocket_connect("/ws/control") as ws:
        # Fire command
        ws.send_json({"command": "fire"})
        resp_fire = ws.receive_json()
        assert resp_fire["status"] == "fired"

        # Home command
        ws.send_json({"command": "home"})
        resp_home = ws.receive_json()
        assert resp_home["status"] == "homed"

        # Config command
        ws.send_json({"command": "config", "payload": {"confidence": 0.65}})
        resp_cfg = ws.receive_json()
        assert resp_cfg["status"] == "config_updated"
        assert resp_cfg["config"]["confidence"] == 0.65
