"""
Tier 1 Feature Tests: F12 - TFMini / TF-Luna LiDAR Serial Binary Parser.
"""

from __future__ import annotations

import time
import pytest

from drone_turret.sensors.lidar import (
    FRAME_HEADER,
    FRAME_LENGTH,
    LidarParser,
    LidarReading,
    LidarSerialReader,
)
from tests.fixtures.mock_serial import MockSerialPort, TFMiniPacketGenerator


def test_lidar_parse_exact_valid_packet():
    """Verifies standard 9-byte packet decoding: distance, strength, temperature, and checksum."""
    parser = LidarParser(min_strength=100)
    
    # Distance = 1050 cm (10.5 m), Strength = 850, Temp = 28.5 °C
    # In TFMini datasheet: Temp_c = (temp_raw / 8) - 256 -> temp_raw = (28.5 + 256) * 8 = 2276
    dist_cm = 1050
    strength = 850
    temp_raw = int((28.5 + 256.0) * 8)
    
    payload = [
        0x59, 0x59,
        dist_cm & 0xFF, (dist_cm >> 8) & 0xFF,
        strength & 0xFF, (strength >> 8) & 0xFF,
        temp_raw & 0xFF, (temp_raw >> 8) & 0xFF,
    ]
    checksum = sum(payload) & 0xFF
    packet = bytes(payload + [checksum])

    reading = parser.parse_packet(packet)
    assert reading is not None
    assert reading.is_valid is True
    assert reading.distance_m == 10.5
    assert reading.distance_cm == 1050
    assert reading.strength == 850
    assert pytest.approx(reading.temperature_c, abs=0.1) == 28.5


def test_lidar_checksum_rejection():
    """Verifies that packets with corrupted checksums are strictly rejected."""
    parser = LidarParser()
    valid_pkt = TFMiniPacketGenerator.build_packet(distance_cm=500, strength=600)
    
    # Corrupt checksum byte
    corrupt_pkt = bytearray(valid_pkt)
    corrupt_pkt[8] = (corrupt_pkt[8] + 1) & 0xFF
    
    reading = parser.parse_packet(bytes(corrupt_pkt))
    assert reading is None
    assert parser.checksum_errors == 1


def test_lidar_weak_signal_filtering():
    """Verifies that signals with strength below threshold (e.g. < 100) are flagged invalid."""
    parser = LidarParser(min_strength=100)
    weak_pkt = TFMiniPacketGenerator.build_packet(distance_cm=450, strength=50)
    
    reading = parser.parse_packet(weak_pkt)
    assert reading is not None
    assert reading.is_valid is False
    assert reading.strength == 50
    assert parser.weak_signals == 1


def test_lidar_streaming_auto_resync_with_noise():
    """Verifies stream parser drops corrupted bytes and auto-resynchronizes on valid header."""
    parser = LidarParser()
    
    pkt1 = TFMiniPacketGenerator.build_packet(distance_cm=300, strength=500)
    pkt2 = TFMiniPacketGenerator.build_packet(distance_cm=800, strength=600)
    
    # Stream: [garbage bytes] + [pkt1] + [garbage bytes] + [pkt2]
    garbage = b"\x01\x02\x59\x04\xFF\xAA\xBB\xCC"  # Contains single 0x59 without second header byte
    stream = garbage + pkt1 + b"\xDE\xAD\xBE\xEF" + pkt2
    
    readings = parser.feed(stream)
    assert len(readings) == 2
    assert readings[0].distance_m == 3.0
    assert readings[1].distance_m == 8.0
    assert parser.dropped_bytes > 0


def test_lidar_distance_bounds_validation():
    """Verifies min and max physical range boundary enforcement."""
    parser = LidarParser(min_distance_m=0.2, max_distance_m=40.0)
    
    # Below min distance (0.05m = 5cm)
    pkt_too_close = TFMiniPacketGenerator.build_packet(distance_cm=5, strength=500)
    r1 = parser.parse_packet(pkt_too_close)
    assert r1 is not None and r1.is_valid is False

    # Above max distance (45.0m = 4500cm)
    pkt_too_far = TFMiniPacketGenerator.build_packet(distance_cm=4500, strength=500)
    r2 = parser.parse_packet(pkt_too_far)
    assert r2 is not None and r2.is_valid is False

    # Valid in-range (25.0m = 2500cm)
    pkt_ok = TFMiniPacketGenerator.build_packet(distance_cm=2500, strength=500)
    r3 = parser.parse_packet(pkt_ok)
    assert r3 is not None and r3.is_valid is True


def test_lidar_serial_reader_async():
    """Verifies LidarSerialReader background thread processing with mock serial transport."""
    mock_port = MockSerialPort(port="COM_MOCK_LIDAR")
    reader = LidarSerialReader(mock_transport=mock_port)
    
    assert reader.start() is True
    assert reader.is_connected is True

    # Inject multiple packets into mock serial stream
    mock_port.inject_tfmini_packet(distance_cm=1250, strength=900)
    time.sleep(0.05)
    
    reading = reader.get_latest_reading()
    assert reading is not None
    assert reading.distance_m == 12.5
    assert reading.is_valid is True
    assert reader.get_latest_distance() == 12.5

    reader.stop()
    assert reader.is_connected is False
