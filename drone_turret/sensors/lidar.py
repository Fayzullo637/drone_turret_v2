"""
LiDAR Serial Parser & Sensor Reader for TFMini / TF-Luna.

Specifications:
- 9-byte binary packet format:
  Byte 0: 0x59 (Header 1)
  Byte 1: 0x59 (Header 2)
  Byte 2: Dist_L (Distance lower 8 bits in cm)
  Byte 3: Dist_H (Distance higher 8 bits in cm)
  Byte 4: Strength_L (Signal strength lower 8 bits)
  Byte 5: Strength_H (Signal strength higher 8 bits)
  Byte 6: Temp_L (Chip temperature lower 8 bits)
  Byte 7: Temp_H (Chip temperature higher 8 bits)
  Byte 8: Checksum (Lower 8 bits of sum of bytes 0..7)
- Checksum validation: (sum(Byte[0..7])) & 0xFF == Byte[8]
- Signal strength threshold filtering (>= 100)
- Automatic byte-stream resynchronization on noise or packet corruption
- Thread-safe non-blocking serial reader with hardware/mock support
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, List, Optional

logger = logging.getLogger(__name__)

FRAME_HEADER = b"\x59\x59"
FRAME_LENGTH = 9
DEFAULT_MIN_STRENGTH = 100
DEFAULT_MIN_DISTANCE_M = 0.1
DEFAULT_MAX_DISTANCE_M = 50.0


@dataclass(frozen=True)
class LidarReading:
    """Represents a single decoded LiDAR measurement packet."""
    distance_m: float
    distance_cm: int
    strength: int
    temperature_c: float
    is_valid: bool
    timestamp: float
    raw_bytes: bytes = b""

    @property
    def is_reliable(self) -> bool:
        """Returns True if measurement is valid and meets strength criteria."""
        return self.is_valid and self.strength >= DEFAULT_MIN_STRENGTH


class LidarParser:
    """
    Stateful streaming binary parser for TFMini / TF-Luna LiDAR modules.
    
    Maintains an internal byte buffer, scans for synchronization headers (0x59 0x59),
    verifies checksums, filters weak signals, and automatically resynchronizes
    after corrupted or dropped bytes.
    """

    def __init__(
        self,
        min_strength: int = DEFAULT_MIN_STRENGTH,
        min_distance_m: float = DEFAULT_MIN_DISTANCE_M,
        max_distance_m: float = DEFAULT_MAX_DISTANCE_M,
    ) -> None:
        self.min_strength = min_strength
        self.min_distance_m = min_distance_m
        self.max_distance_m = max_distance_m
        self._buffer = bytearray()
        
        # Telemetry & Health Statistics
        self.total_bytes_received: int = 0
        self.total_packets_parsed: int = 0
        self.valid_packets: int = 0
        self.checksum_errors: int = 0
        self.dropped_bytes: int = 0
        self.weak_signals: int = 0

    def parse_packet(self, packet_bytes: bytes | bytearray) -> Optional[LidarReading]:
        """
        Parses an exact 9-byte packet.
        
        Returns LidarReading if packet has correct header and checksum,
        or None if format/checksum is invalid.
        """
        if len(packet_bytes) != FRAME_LENGTH:
            return None

        if packet_bytes[0] != 0x59 or packet_bytes[1] != 0x59:
            return None

        # Checksum calculation: sum of bytes 0..7 modulo 256
        calc_checksum = sum(packet_bytes[:8]) & 0xFF
        expected_checksum = packet_bytes[8]

        if calc_checksum != expected_checksum:
            self.checksum_errors += 1
            return None

        # Decode fields (little-endian unsigned 16-bit integers)
        dist_cm = packet_bytes[2] | (packet_bytes[3] << 8)
        strength = packet_bytes[4] | (packet_bytes[5] << 8)
        temp_raw = packet_bytes[6] | (packet_bytes[7] << 8)

        # Temperature conversion according to Benewake datasheet:
        # Temp(°C) = (Temp_raw / 8) - 256
        temp_c = (temp_raw / 8.0) - 256.0
        dist_m = dist_cm / 100.0

        # Validate range and strength
        is_strength_ok = strength >= self.min_strength
        is_range_ok = self.min_distance_m <= dist_m <= self.max_distance_m
        is_valid = is_strength_ok and is_range_ok

        self.total_packets_parsed += 1
        if is_valid:
            self.valid_packets += 1
        elif not is_strength_ok:
            self.weak_signals += 1

        return LidarReading(
            distance_m=round(dist_m, 3),
            distance_cm=dist_cm,
            strength=strength,
            temperature_c=round(temp_c, 2),
            is_valid=is_valid,
            timestamp=time.time(),
            raw_bytes=bytes(packet_bytes),
        )

    def feed(self, raw_bytes: bytes | bytearray) -> List[LidarReading]:
        """
        Feeds raw serial byte chunks into the parser stream.
        
        Scans for 0x59 0x59 headers, extracts packets, verifies checksums,
        and returns all valid readings found in the stream.
        """
        if not raw_bytes:
            return []

        self.total_bytes_received += len(raw_bytes)
        self._buffer.extend(raw_bytes)
        readings: List[LidarReading] = []

        while len(self._buffer) >= FRAME_LENGTH:
            # Search for header 0x59 0x59 in buffer
            header_idx = self._buffer.find(FRAME_HEADER)

            if header_idx == -1:
                # No header in current buffer.
                # Keep last byte if it might be the first byte of header (0x59)
                if len(self._buffer) > 0 and self._buffer[-1] == 0x59:
                    self.dropped_bytes += len(self._buffer) - 1
                    self._buffer = self._buffer[-1:]
                else:
                    self.dropped_bytes += len(self._buffer)
                    self._buffer.clear()
                break

            if header_idx > 0:
                # Discard noise bytes before header (auto-resync)
                self.dropped_bytes += header_idx
                del self._buffer[:header_idx]

            # We now have buffer starting with 0x59 0x59
            if len(self._buffer) < FRAME_LENGTH:
                # Need more bytes to complete packet
                break

            candidate_packet = bytes(self._buffer[:FRAME_LENGTH])
            reading = self.parse_packet(candidate_packet)

            if reading is not None:
                # Successfully decoded valid packet
                readings.append(reading)
                del self._buffer[:FRAME_LENGTH]
            else:
                # Checksum failed or false header inside payload
                # Advance by 1 byte to search for the next true header
                self.dropped_bytes += 1
                del self._buffer[:1]

        return readings

    def reset(self) -> None:
        """Clears internal stream buffer and resets counters."""
        self._buffer.clear()
        self.total_bytes_received = 0
        self.total_packets_parsed = 0
        self.valid_packets = 0
        self.checksum_errors = 0
        self.dropped_bytes = 0
        self.weak_signals = 0


class LidarSerialReader:
    """
    Asynchronous, thread-safe serial reader for TFMini / TF-Luna LiDAR.
    
    Runs a background daemon thread that reads serial chunks, feeds them to LidarParser,
    and maintains the latest verified distance reading.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 115200,
        timeout: float = 0.5,
        min_strength: int = DEFAULT_MIN_STRENGTH,
        on_reading_callback: Optional[Callable[[LidarReading], None]] = None,
        mock_transport: Optional[Any] = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.parser = LidarParser(min_strength=min_strength)
        self.on_reading_callback = on_reading_callback
        self.mock_transport = mock_transport

        self._serial: Optional[Any] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._latest_reading: Optional[LidarReading] = None
        self._is_connected = False

    @property
    def is_connected(self) -> bool:
        """Indicates if serial port or mock transport is currently active."""
        with self._lock:
            return self._is_connected

    def start(self) -> bool:
        """Opens serial connection and starts background reader thread."""
        if self._running:
            return True

        if self.mock_transport is not None:
            self._serial = self.mock_transport
            self._is_connected = True
        elif self.port:
            try:
                import serial
                self._serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                )
                self._is_connected = True
                logger.info("Connected to LiDAR on %s @ %d baud", self.port, self.baudrate)
            except Exception as e:
                logger.error("Failed to connect to LiDAR on %s: %s", self.port, e)
                self._is_connected = False
                return False
        else:
            logger.warning("No LiDAR port or mock transport provided.")
            return False

        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True, name="LiDAR-Reader")
        self._thread.start()
        return True

    def stop(self) -> None:
        """Stops background thread and closes serial connection."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        
        with self._lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
            self._is_connected = False

    def get_latest_reading(self) -> Optional[LidarReading]:
        """Returns the most recent decoded LiDAR reading."""
        with self._lock:
            return self._latest_reading

    def get_latest_distance(self, max_age_s: float = 1.0) -> Optional[float]:
        """
        Returns latest valid distance in meters if received within max_age_s.
        Returns None if no reading, reading invalid, or reading too old.
        """
        with self._lock:
            if (
                self._latest_reading is not None
                and self._latest_reading.is_valid
                and (time.time() - self._latest_reading.timestamp) <= max_age_s
            ):
                return self._latest_reading.distance_m
            return None

    def inject_bytes(self, data: bytes) -> List[LidarReading]:
        """Directly injects bytes (useful for unit tests and simulation)."""
        readings = self.parser.feed(data)
        if readings:
            with self._lock:
                for r in readings:
                    if r.is_valid:
                        self._latest_reading = r
        return readings

    def _reader_loop(self) -> None:
        """Background thread loop consuming serial data."""
        while self._running:
            try:
                if self._serial is None:
                    break

                raw_chunk = b""
                if hasattr(self._serial, "read"):
                    in_waiting = getattr(self._serial, "in_waiting", 0)
                    bytes_to_read = max(FRAME_LENGTH, in_waiting) if in_waiting > 0 else FRAME_LENGTH
                    raw_chunk = self._serial.read(bytes_to_read)
                
                if raw_chunk:
                    readings = self.parser.feed(raw_chunk)
                    if readings:
                        with self._lock:
                            for r in readings:
                                if r.is_valid:
                                    self._latest_reading = r
                                if self.on_reading_callback:
                                    try:
                                        self.on_reading_callback(r)
                                    except Exception as cb_err:
                                        logger.error("LiDAR callback error: %s", cb_err)
                else:
                    time.sleep(0.005)
            except Exception as e:
                logger.error("LiDAR serial loop encountered error: %s", e)
                with self._lock:
                    self._is_connected = False
                time.sleep(0.1)
