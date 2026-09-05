"""
drone_turret_v2: AI Interceptor Turret with Ballistic Net Launcher.

CLI Entry Point & FastAPI Web Server Launcher.

Usage:
    python main.py [--host 0.0.0.0] [--port 8000] [--model models/drone_best.pt]
                   [--camera 0] [--sim] [--confidence 0.45] [--debug]
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import sys
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

from drone_turret import __author__, __version__
from drone_turret.config import SystemConfig, TurretConfigModel, TurretStateModel
from drone_turret.coordinator import PipelineCoordinator
from drone_turret.comms.serial_comm import list_serial_ports
from drone_turret.vision.camera import discover_cameras

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] [%(name)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("drone_turret.main")

# Global coordinator instance
coordinator: Optional[PipelineCoordinator] = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """FastAPI Lifespan context manager for startup and graceful shutdown."""
    global coordinator
    logger.info("Initializing Pipeline Coordinator on server startup...")
    if coordinator is not None:
        coordinator.start()
    yield
    logger.info("Shutting down Pipeline Coordinator on server shutdown...")
    if coordinator is not None:
        coordinator.stop()


def create_app(config: Optional[SystemConfig] = None) -> FastAPI:
    """Factory creating configured FastAPI web application."""
    global coordinator
    if coordinator is None:
        coordinator = PipelineCoordinator(config=config or SystemConfig())

    app = FastAPI(
        title="Drone Turret v2 - AI Interceptor HUD & API",
        description="AI-guided anti-drone interceptor turret with ballistic net launcher.",
        version=__version__,
        lifespan=lifespan,
    )

    # Enable CORS for browser access
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Mount static files if present
    static_dir = Path("static")
    if static_dir.exists() and static_dir.is_dir():
        app.mount("/static", StaticFiles(directory="static"), name="static")

    # --- REST Endpoints ---

    @app.get("/", response_class=HTMLResponse)
    async def index():
        """Serves the Single Page Application or fallback dashboard."""
        index_file = Path("static/index.html")
        if index_file.exists():
            return HTMLResponse(content=index_file.read_text(encoding="utf-8"))
        return HTMLResponse(
            content=f"""
        <!DOCTYPE html>
        <html>
        <head><title>Drone Turret v2</title></head>
        <body style="background:#111; color:#0f0; font-family:monospace; padding:2rem;">
            <h1>DRONE TURRET v2 - ACTIVE</h1>
            <p>System Version: {__version__}</p>
            <p>Video Feed: <a href="/video_feed" style="color:#0ff;">/video_feed</a></p>
            <p>Telemetry API: <a href="/api/status" style="color:#0ff;">/api/status</a></p>
            <p>Config API: <a href="/api/config" style="color:#0ff;">/api/config</a></p>
        </body>
        </html>
        """
        )

    @app.get("/video_feed")
    async def video_feed():
        """High-FPS MJPEG multipart video feed with tactical HUD overlays."""
        if coordinator is None:
            raise HTTPException(status_code=503, detail="Coordinator not initialized")
        return StreamingResponse(
            coordinator.generate_mjpeg_frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @app.get("/api/status")
    async def get_status() -> Dict[str, Any]:
        """Returns real-time system state and kinematics telemetry."""
        if coordinator is None:
            return TurretStateModel().model_dump()
        return coordinator.get_telemetry()

    @app.get("/api/config")
    async def get_config() -> Dict[str, Any]:
        """Returns current active system configuration."""
        if coordinator is None:
            return TurretConfigModel().model_dump()
        return coordinator.config.to_flat_dict()

    @app.post("/api/config")
    async def update_config(payload: TurretConfigModel) -> Dict[str, Any]:
        """Updates system parameters dynamically at runtime."""
        if coordinator is None:
            raise HTTPException(status_code=503, detail="Coordinator not initialized")

        update_dict = {k: v for k, v in payload.model_dump().items() if v is not None}
        coordinator.update_config(update_dict)
        return {"status": "ok", "config": coordinator.config.to_flat_dict()}

    @app.get("/api/cameras")
    async def list_cameras() -> List[Dict[str, Any]]:
        """Returns list of all available camera hardware and synthetic sources."""
        return discover_cameras()

    @app.get("/api/hardware/ports")
    @app.get("/api/serial-ports")
    async def list_ports() -> Dict[str, Any]:
        """Scans and lists system serial COM ports and simulation fallback."""
        ports = list_serial_ports()
        ports_list = [
            {"port": "SIMULATION", "description": "Virtual Simulation Loopback", "connected": True}
        ]
        for p in ports:
            ports_list.append({
                "port": p["device"],
                "description": p["description"],
                "connected": False,
            })
        return {"ports": ports_list}

    @app.post("/api/turret/fire")
    @app.post("/api/fire")
    async def fire_turret() -> Dict[str, Any]:
        """Step 1: Arms the turret for firing. Operator must confirm separately."""
        if coordinator is None:
            raise HTTPException(status_code=503, detail="Coordinator not initialized")
        coordinator.fire(confirmed=False)
        return {"status": "armed", "message": "Fire armed. POST /api/fire/confirm to launch.", "timestamp": time.time()}

    @app.post("/api/turret/fire/confirm")
    @app.post("/api/fire/confirm")
    async def fire_turret_confirm() -> Dict[str, Any]:
        """Step 2: Confirms and executes the fire command (human-in-the-loop)."""
        if coordinator is None:
            raise HTTPException(status_code=503, detail="Coordinator not initialized")
        success = coordinator.fire(confirmed=True)
        return {"status": "fired" if success else "rejected", "timestamp": time.time()}

    @app.post("/api/turret/home")
    async def home_turret() -> Dict[str, Any]:
        """Commands turret back to 90°, 90° center position."""
        if coordinator is None:
            raise HTTPException(status_code=503, detail="Coordinator not initialized")
        success = coordinator.home()
        return {"status": "homed" if success else "failed"}

    @app.post("/api/turret/lock")
    async def lock_target(payload: Dict[str, int]) -> Dict[str, Any]:
        """Explicitly lock onto target track ID."""
        if coordinator is None:
            raise HTTPException(status_code=503, detail="Coordinator not initialized")
        track_id = payload.get("track_id")
        if track_id is not None:
            coordinator.lock_track(track_id)
            return {"status": "locked", "track_id": track_id}
        raise HTTPException(status_code=400, detail="Missing track_id")

    @app.post("/api/turret/unlock")
    async def unlock_target() -> Dict[str, Any]:
        """Release active target lock."""
        if coordinator is None:
            raise HTTPException(status_code=503, detail="Coordinator not initialized")
        coordinator.unlock_track()
        return {"status": "unlocked"}

    @app.post("/api/turret/switch-model")
    async def switch_model(payload: Dict[str, str]) -> Dict[str, Any]:
        """Hot-swap YOLO model weights."""
        if coordinator is None:
            raise HTTPException(status_code=503, detail="Coordinator not initialized")
        model_name = payload.get("model_name")
        if not model_name:
            raise HTTPException(status_code=400, detail="Missing model_name")
        success = coordinator.switch_model(model_name)
        return {"status": "switched" if success else "failed", "model": model_name}

    @app.post("/api/turret/switch-camera")
    async def switch_camera(payload: Dict[str, Any]) -> Dict[str, Any]:
        """Dynamically switch camera capture device."""
        if coordinator is None:
            raise HTTPException(status_code=503, detail="Coordinator not initialized")
        cam_id = payload.get("camera_id")
        if cam_id is None:
            raise HTTPException(status_code=400, detail="Missing camera_id")
        success = coordinator.switch_camera(cam_id)
        return {"status": "switched" if success else "failed", "camera_id": cam_id}

    # --- WebSockets ---

    @app.websocket("/ws/telemetry")
    async def websocket_telemetry(ws: WebSocket):
        """High-frequency (30Hz) JSON telemetry stream."""
        await ws.accept()
        try:
            while True:
                if coordinator is not None:
                    data = coordinator.get_telemetry()
                    await ws.send_json(data)
                await asyncio.sleep(0.033)
        except (WebSocketDisconnect, Exception):
            pass

    @app.websocket("/ws/control")
    async def websocket_control(ws: WebSocket):
        """Bidirectional WebSocket command channel."""
        await ws.accept()
        try:
            while True:
                msg = await ws.receive_json()
                cmd = msg.get("command")
                if cmd == "FIRE" and coordinator:
                    coordinator.fire()
                    await ws.send_json({"status": "fired"})
                elif cmd == "HOME" and coordinator:
                    coordinator.home()
                    await ws.send_json({"status": "homed"})
                elif cmd == "CONFIG" and coordinator:
                    coordinator.update_config(msg.get("config", {}))
                    await ws.send_json({"status": "config_updated"})
        except (WebSocketDisconnect, Exception):
            pass

    return app


def print_banner(host: str, port: int, sim_mode: bool, model: str, camera: Any):
    """Prints high-visibility military tactical ASCII banner."""
    banner = f"""
================================================================================
   ____  ____   ___  _   _ _____   _____ _   _ ____  ____  _____ _____ 
  |  _ \|  _ \ / _ \| \ | | ____| |_   _| | | |  _ \|  _ \| ____|_   _|
  | | | | |_) | | | |  \| |  _|     | | | | | | |_) | |_) |  _|   | |  
  | |_| |  _ <| |_| | |\  | |___    | | | |_| |  _ <|  _ <| |___  | |  
  |____/|_| \_\\___/|_| \_|_____|   |_|  \___/|_| \_\_| \_\_____| |_|  
                 AI Interceptor Turret v{__version__} - Operational
================================================================================
  >> Web Interface:    http://{host if host != '0.0.0.0' else '127.0.0.1'}:{port}/
  >> Video Stream:     http://{host if host != '0.0.0.0' else '127.0.0.1'}:{port}/video_feed
  >> Telemetry API:    http://{host if host != '0.0.0.0' else '127.0.0.1'}:{port}/api/status
  >> Operation Mode:   {'[VIRTUAL SIMULATION]' if sim_mode else '[HARDWARE ATTACHED]'}
  >> Active Model:     {model}
  >> Camera Source:    {camera}
================================================================================
"""
    print(banner)


def main():
    """CLI entry point for drone_turret_v2 launcher."""
    parser = argparse.ArgumentParser(
        description="drone_turret_v2: AI Anti-Drone Interceptor Turret",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host IP address to bind FastAPI server")
    parser.add_argument("--port", type=int, default=8000, help="TCP port to listen on")
    parser.add_argument("--model", type=str, default="models/drone_best.pt", help="Path to YOLO model weights")
    parser.add_argument("--camera", type=str, default="0", help="Camera device index or 'synthetic'")
    parser.add_argument("--sim", action="store_true", default=False, help="Force virtual simulation mode without hardware")
    parser.add_argument("--debug", action="store_true", default=False, help="Enable debug logging")
    parser.add_argument("--confidence", type=float, default=0.45, help="Detection confidence threshold")
    parser.add_argument("--config", type=str, default=None, help="Optional path to custom JSON config")

    args = parser.parse_args()

    # Set log level
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.getLogger().setLevel(log_level)

    # Parse camera id
    try:
        cam_id: Union[int, str] = int(args.camera)
    except ValueError:
        cam_id = args.camera

    # Build configuration
    if args.config:
        sys_config = SystemConfig.load_json(args.config)
    else:
        sys_config = SystemConfig()

    sys_config.server.host = args.host
    sys_config.server.port = args.port
    sys_config.server.debug = args.debug
    sys_config.vision.model_path = args.model
    sys_config.vision.model_name = os.path.basename(args.model)
    sys_config.vision.camera_id = cam_id
    sys_config.vision.confidence_threshold = args.confidence
    sys_config.vision.confidence = args.confidence

    if args.sim:
        sys_config.hardware.simulation_mode = True

    # Print Banner
    print_banner(
        host=args.host,
        port=args.port,
        sim_mode=sys_config.hardware.simulation_mode,
        model=sys_config.vision.model_name,
        camera=sys_config.vision.camera_id,
    )

    # Create App
    app = create_app(config=sys_config)

    # Run Uvicorn
    uvicorn.run(
        app,
        host=args.host,
        port=args.port,
        log_level="debug" if args.debug else "info",
    )


if __name__ == "__main__":
    main()

