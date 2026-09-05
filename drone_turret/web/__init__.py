"""
Web Application, REST API, and Tactical HUD Streaming Package for drone_turret_v2.

Exposes:
- create_app: FastAPI application factory.
- app: Global FastAPI application instance.
- TacticalHUDOverlay: Military HUD video overlay renderer.
- MJPEGStreamer: Low-latency multipart/x-mixed-replace frame generator.
"""

from drone_turret.web.stream import MJPEGStreamer, TacticalHUDOverlay
from drone_turret.web.app import create_app, app, TurretConfigModel, TurretStateModel

__all__ = [
    "create_app",
    "app",
    "TacticalHUDOverlay",
    "MJPEGStreamer",
    "TurretConfigModel",
    "TurretStateModel",
]
