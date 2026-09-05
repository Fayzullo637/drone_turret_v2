"""Tier 1 Unit Tests: TFMini LiDAR 9-Byte Serial Protocol Parser (Feature F12).

Verifies binary frame decoding, header synchronization (0x59 0x59), checksum validation,
signal strength filtering, out-of-range rejection, and streaming resilience against corrupted bytes.
"""

from __future__ import annotations

from typing import List, Optional, Tuple
import pytest

from tests.fixtures.mock_serial import MockSerialPort, TFMiniPacketGenerator


class TFMiniLidarParser:
    """Reference implementation of TFMini/TF-Luna 9-byte binary LiDAR stream parser."""

    HEADER = 0x59
    MIN_SIGNAL_STRENGTH = 100

    def __init__(self):
        self.buffer = bytearray()
        self.last_distance_m: Optional[float] = None
        self.last_strength: int = 0
        self.last_temp_c: float = 0.0

    def feed_bytes(self, chunk: bytes) -> List[float]:
        """Feeds incoming serial bytes and returns list of decoded valid distances in meters."""
        self.buffer.extend(chunk)
        valid_distances = []

        while len(self.buffer) >= 9:
            # Find header 0x59 0x59
            if self.buffer[0] != self.HEADER or self.buffer[1] != self.HEADER:
                self.buffer.pop(0)
                continue

            # Extract 9-byte frame
            frame = self.buffer[:9]

            # Verify checksum
            expected_checksum = sum(frame[:8]) & 0xFF
            actual_checksum = frame[8]

            if actual_checksum != expected_checksum:
                # Checksum failed, discard header byte and resync
                self.buffer.pop(0)
                continue

            # Frame is syntactically valid; consume it
            self.buffer = self.buffer[9:]

            dist_cm = frame[2] | (frame[3] << 8)
            strength = frame[4] | (frame[5] << 8)
            temp_raw = frame[6] | (frame[7] << 8)

            # Signal strength validation
            if strength < self.MIN_SIGNAL_STRENGTH or dist_cm >= 0xFFFF or dist_cm <= 0:
                continue

            dist_m = dist_cm / 100.0
            self.last_distance_m = dist_m
            self.last_strength = strength
            self.last_temp_c = temp_raw / 10.0
            valid_distances.append(dist_m)

        return valid_distances


def test_lidar_9byte_packet_parsing(tfmini_gen):
    """T1.6.1: Verifies parsing of standard 9-byte packet representing 25.50m (2550 cm)."""
    parser = TFMiniLidarParser()
    pkt = tfmini_gen.build_packet(distance_cm=2550, strength=1200, temp_c=25.0)

    distances = parser.feed_bytes(pkt)
    assert len(distances) == 1
    assert distances[0] == pytest.approx(25.50)
    assert parser.last_strength == 1200
    assert parser.last_temp_c == pytest.approx(25.0)


def test_lidar_checksum_validation(tfmini_gen):
    """T1.6.2: Verifies packet with invalid checksum is rejected."""
    parser = TFMiniLidarParser()
    pkt = tfmini_gen.build_corrupted_checksum_packet(distance_cm=1500)

    distances = parser.feed_bytes(pkt)
    assert len(distances) == 0
    assert parser.last_distance_m is None


def test_lidar_weak_signal_rejection(tfmini_gen):
    """T1.6.3: Verifies packet with signal strength < 100 is filtered out as unreliable."""
    parser = TFMiniLidarParser()
    pkt = tfmini_gen.build_weak_signal_packet(distance_cm=1000, strength=60)

    distances = parser.feed_bytes(pkt)
    assert len(distances) == 0


def test_lidar_out_of_range_handling(tfmini_gen):
    """T1.6.4: Verifies 0xFFFF (out-of-range sensor code) is filtered."""
    parser = TFMiniLidarParser()
    pkt = tfmini_gen.build_out_of_range_packet()

    distances = parser.feed_bytes(pkt)
    assert len(distances) == 0


def test_lidar_stream_resynchronization(tfmini_gen):
    """T1.6.5: Verifies parser recovers and resynchronizes after arbitrary noise bytes."""
    parser = TFMiniLidarParser()
    noise = bytes([0x12, 0x34, 0x59, 0x88, 0x99, 0x00])  # Garbage containing partial false header
    valid_pkt = tfmini_gen.build_packet(distance_cm=4200, strength=1500)

    distances = parser.feed_bytes(noise + valid_pkt)
    assert len(distances) == 1
    assert distances[0] == pytest.approx(42.00)


def test_lidar_chunked_streaming(tfmini_gen):
    """T1.6.6: Verifies packet split across 1-byte read increments parses correctly."""
    parser = TFMiniLidarParser()
    pkt = tfmini_gen.build_packet(distance_cm=1830, strength=900)

    distances = []
    for b in pkt:
        res = parser.feed_bytes(bytes([b]))
        distances.extend(res)

    assert len(distances) == 1
    assert distances[0] == pytest.approx(18.30)
