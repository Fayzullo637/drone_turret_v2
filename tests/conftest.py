"""Root pytest configuration and shared test fixtures for drone_turret_v2."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Dict, Generator, List, Optional, Tuple
import numpy as np
import pytest

# Ensure project root is in sys.path
PROJECT_ROOT = Path(__file__).parent.parent.resolve()
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tests.fixtures.mock_serial import MockSerialPort, TFMiniPacketGenerator
from tests.fixtures.physics_benchmarks import BallisticsBenchmarks
from tests.fixtures.synthetic_video import SyntheticVideoGenerator, TargetKinematics


@pytest.fixture
def mock_serial() -> MockSerialPort:
    """Provides a fresh, thread-safe in-memory virtual serial port."""
    port = MockSerialPort(port="COM_MOCK_TEST", baudrate=115200, timeout=0.1)
    yield port
    port.close()


@pytest.fixture
def tfmini_gen() -> type[TFMiniPacketGenerator]:
    """Provides the TFMini LiDAR binary packet encoder and mutator."""
    return TFMiniPacketGenerator


@pytest.fixture
def video_gen() -> SyntheticVideoGenerator:
    """Provides a deterministic 640x480 @ 30fps synthetic video generator."""
    return SyntheticVideoGenerator(width=640, height=480, fps=30.0, focal_length=800.0)


@pytest.fixture
def physics_benchmarks() -> type[BallisticsBenchmarks]:
    """Provides analytical and numerical reference solutions for ballistics."""
    return BallisticsBenchmarks


@pytest.fixture
def sample_frame() -> np.ndarray:
    """Provides a standard 640x480 BGR synthetic test image."""
    gen = SyntheticVideoGenerator(width=640, height=480)
    target = TargetKinematics(x=0.0, y=0.0, z=50.0)
    frame, _ = gen.generate_single_frame([target])
    return frame


@pytest.fixture
def target_kinematics_flyby() -> TargetKinematics:
    """Provides a target kinematic state flying orthogonally at 200 km/h at 50m."""
    vx_mps = 200.0 / 3.6  # 55.56 m/s
    return TargetKinematics(x=-20.0, y=1.5, z=50.0, vx=vx_mps, vy=0.0, vz=0.0)


@pytest.fixture
def target_kinematics_headon() -> TargetKinematics:
    """Provides a target kinematic state diving head-on at 150 km/h."""
    vz_mps = -(150.0 / 3.6)
    return TargetKinematics(x=0.0, y=5.0, z=80.0, vx=0.0, vy=-1.0, vz=vz_mps)
