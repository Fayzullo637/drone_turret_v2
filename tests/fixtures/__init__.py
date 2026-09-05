"""Test fixtures, mocks, synthetic generators, and physics reference models."""

from tests.fixtures.synthetic_video import SyntheticVideoGenerator, TargetKinematics
from tests.fixtures.mock_serial import MockSerialPort, TFMiniPacketGenerator
from tests.fixtures.physics_benchmarks import BallisticsBenchmarks

__all__ = [
    "SyntheticVideoGenerator",
    "TargetKinematics",
    "MockSerialPort",
    "TFMiniPacketGenerator",
    "BallisticsBenchmarks",
]
