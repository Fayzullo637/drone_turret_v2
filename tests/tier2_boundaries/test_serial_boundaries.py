"""Tier 2 Boundary Tests: Serial Transport & LiDAR Protocol Edge Cases (Features F12, F15).

Tests protocol boundary conditions: corrupt byte flood, packet fragmentation,
0xFFFF sensor saturation, servo angle clamping [-90, +270], and closed port exceptions.
"""

from __future__ import annotations

import pytest

from tests.fixtures.mock_serial import MockSerialPort, TFMiniPacketGenerator
from tests.tier1_features.test_lidar_parser import TFMiniLidarParser
from tests.tier1_features.test_arduino_comm import ArduinoSerialCommunicator


def test_serial_corrupted_byte_flood(mock_serial):
    """T2.4.1: Verifies parser discards 5000 random bytes of line noise without buffer overflow."""
    parser = TFMiniLidarParser()
    mock_serial.inject_corrupted_noise(count=5000)

    # Read and feed in chunks
    while mock_serial.in_waiting > 0:
        chunk = mock_serial.read(64)
        parser.feed_bytes(chunk)

    assert len(parser.buffer) < 20  # Buffer did not accumulate junk unbounded


def test_serial_partial_packet_split(mock_serial, tfmini_gen):
    """T2.4.2: Verifies 9-byte packet split across 3 fragments accumulates and parses."""
    parser = TFMiniLidarParser()
    pkt = tfmini_gen.build_packet(distance_cm=3300, strength=800)

    # Split into 3-byte pieces
    chunk1, chunk2, chunk3 = pkt[:3], pkt[3:6], pkt[6:]

    res1 = parser.feed_bytes(chunk1)
    assert len(res1) == 0

    res2 = parser.feed_bytes(chunk2)
    assert len(res2) == 0

    res3 = parser.feed_bytes(chunk3)
    assert len(res3) == 1
    assert res3[0] == pytest.approx(33.0)


def test_serial_lidar_max_range_0xffff(mock_serial, tfmini_gen):
    """T2.4.3: Verifies 0xFFFF (out-of-range sensor code) is discarded without error."""
    parser = TFMiniLidarParser()
    pkt = tfmini_gen.build_out_of_range_packet()

    res = parser.feed_bytes(pkt)
    assert len(res) == 0
    assert parser.last_distance_m is None


def test_arduino_angle_clamping_negative(mock_serial):
    """T2.4.4: Verifies negative commanded angle is clamped to 0."""
    comm = ArduinoSerialCommunicator(mock_serial=mock_serial)
    cmd = comm.send_angles(-45.0, -10.0)

    assert cmd == "0,0\n"
    assert mock_serial.get_last_command() == (0, 0)


def test_arduino_angle_clamping_overflow(mock_serial):
    """T2.4.5: Verifies angle > 180 is clamped to 180."""
    comm = ArduinoSerialCommunicator(mock_serial=mock_serial)
    cmd = comm.send_angles(250.0, 195.0)

    assert cmd == "180,180\n"
    assert mock_serial.get_last_command() == (180, 180)


def test_serial_closed_port_read_write(mock_serial):
    """T2.4.6: Verifies reading or writing to a closed port raises IOError."""
    mock_serial.close()
    assert mock_serial.is_open is False

    with pytest.raises(IOError, match="closed"):
        mock_serial.write(b"90,90\n")

    with pytest.raises(IOError, match="closed"):
        mock_serial.read(10)
