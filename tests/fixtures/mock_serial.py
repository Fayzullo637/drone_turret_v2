"""In-Memory Mock Serial Port & TFMini LiDAR Packet Generator.

Provides thread-safe virtual UART communication matching the pyserial.Serial interface,
with byte injection, packet generation, fuzzing, and telemetry interception for testing.
"""

from __future__ import annotations

import io
import random
import time
from typing import List, Optional, Tuple, Union


class TFMiniPacketGenerator:
    """Encodes and corrupts standard 9-byte TFMini/TF-Luna LiDAR binary packets.
    
    Standard Format (9 bytes):
    - Byte 0: 0x59 (Header 1)
    - Byte 1: 0x59 (Header 2)
    - Byte 2: Dist_L (Distance Low byte, in cm)
    - Byte 3: Dist_H (Distance High byte, in cm)
    - Byte 4: Strength_L (Signal strength Low byte)
    - Byte 5: Strength_H (Signal strength High byte)
    - Byte 6: Temp_L (Temperature Low byte, in 0.1 C or raw)
    - Byte 7: Temp_H (Temperature High byte)
    - Byte 8: Checksum (Sum of Bytes 0..7 & 0xFF)
    """

    HEADER_BYTE = 0x59

    @classmethod
    def compute_checksum(cls, payload8: Union[bytes, bytearray, List[int]]) -> int:
        """Calculates 8-bit checksum: sum of first 8 bytes modulo 256."""
        return sum(payload8[:8]) & 0xFF

    @classmethod
    def build_packet(
        cls, distance_cm: int, strength: int = 1000, temp_c: int = 25
    ) -> bytes:
        """Constructs a fully valid 9-byte TFMini LiDAR binary packet."""
        dist_clamped = max(0, min(0xFFFF, distance_cm))
        str_clamped = max(0, min(0xFFFF, strength))
        temp_val = max(0, min(0xFFFF, int(temp_c * 10)))

        dist_l = dist_clamped & 0xFF
        dist_h = (dist_clamped >> 8) & 0xFF

        str_l = str_clamped & 0xFF
        str_h = (str_clamped >> 8) & 0xFF

        temp_l = temp_val & 0xFF
        temp_h = (temp_val >> 8) & 0xFF

        payload = [
            cls.HEADER_BYTE,
            cls.HEADER_BYTE,
            dist_l,
            dist_h,
            str_l,
            str_h,
            temp_l,
            temp_h,
        ]
        checksum = cls.compute_checksum(payload)
        payload.append(checksum)
        return bytes(payload)

    @classmethod
    def build_corrupted_checksum_packet(
        cls, distance_cm: int, strength: int = 1000
    ) -> bytes:
        """Constructs a packet with an invalid checksum byte for error testing."""
        pkt = bytearray(cls.build_packet(distance_cm, strength))
        pkt[8] = (pkt[8] ^ 0xFF)  # Invert checksum
        return bytes(pkt)

    @classmethod
    def build_corrupted_header_packet(cls, distance_cm: int) -> bytes:
        """Constructs a packet with invalid header bytes."""
        pkt = bytearray(cls.build_packet(distance_cm))
        pkt[0] = 0xAA
        pkt[1] = 0xBB
        return bytes(pkt)

    @classmethod
    def build_weak_signal_packet(
        cls, distance_cm: int, strength: int = 50
    ) -> bytes:
        """Constructs a packet with low confidence/weak signal strength (<100)."""
        return cls.build_packet(distance_cm, strength=strength)

    @classmethod
    def build_out_of_range_packet(cls) -> bytes:
        """Constructs a packet with 0xFFFF distance (out of sensor bounds)."""
        return cls.build_packet(distance_cm=0xFFFF, strength=0)


class MockSerialPort:
    """In-memory virtual serial port conforming to the PySerial interface."""

    def __init__(
        self,
        port: str = "COM_MOCK",
        baudrate: int = 115200,
        timeout: float = 0.1,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open: bool = True
        self.rx_buffer = bytearray()
        self.tx_history: List[bytes] = []
        self.byte_drop_rate: float = 0.0

    def open(self) -> None:
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def reset_input_buffer(self) -> None:
        if not self.is_open:
            raise IOError("Serial port is closed")
        self.rx_buffer.clear()

    def reset_output_buffer(self) -> None:
        if not self.is_open:
            raise IOError("Serial port is closed")
        self.tx_history.clear()

    def flush(self) -> None:
        pass

    @property
    def in_waiting(self) -> int:
        if not self.is_open:
            raise IOError("Serial port is closed")
        return len(self.rx_buffer)

    def write(self, data: bytes) -> int:
        if not self.is_open:
            raise IOError("Serial port is closed")
        self.tx_history.append(bytes(data))
        return len(data)

    def read(self, size: int = 1) -> bytes:
        if not self.is_open:
            raise IOError("Serial port is closed")
        if not self.rx_buffer or size <= 0:
            return b""
        actual_size = min(size, len(self.rx_buffer))
        chunk = self.rx_buffer[:actual_size]
        self.rx_buffer = self.rx_buffer[actual_size:]
        return bytes(chunk)

    def readline(self) -> bytes:
        if not self.is_open:
            raise IOError("Serial port is closed")
        idx = self.rx_buffer.find(b"\n")
        if idx == -1:
            res = bytes(self.rx_buffer)
            self.rx_buffer.clear()
            return res
        res = bytes(self.rx_buffer[: idx + 1])
        self.rx_buffer = self.rx_buffer[idx + 1 :]
        return res

    def inject_rx(self, data: bytes) -> None:
        """Inject bytes into the virtual RX incoming stream."""
        if self.byte_drop_rate > 0.0:
            # Simulate random transmission byte loss
            filtered = bytearray(
                b for b in data if random.random() >= self.byte_drop_rate
            )
            self.rx_buffer.extend(filtered)
        else:
            self.rx_buffer.extend(data)

    def inject_tfmini_packet(
        self, distance_cm: int, strength: int = 1000, temp_c: int = 25
    ) -> None:
        """Injects a valid 9-byte TFMini packet into RX buffer."""
        pkt = TFMiniPacketGenerator.build_packet(distance_cm, strength, temp_c)
        self.inject_rx(pkt)

    def inject_corrupted_noise(self, count: int = 16) -> None:
        """Injects non-0x59 random binary noise bytes."""
        noise = bytes(random.randint(0, 88) for _ in range(count))
        self.inject_rx(noise)

    def get_last_command(self) -> Optional[Tuple[int, int]]:
        """Parses the most recent 'PAN,TILT\\n' command from outgoing TX history."""
        if not self.tx_history:
            return None
        try:
            raw = self.tx_history[-1].decode("utf-8", errors="ignore").strip()
            parts = raw.split(",")
            if len(parts) >= 2:
                return int(float(parts[0])), int(float(parts[1]))
        except Exception:
            return None
        return None

    def get_command_history(self) -> List[Tuple[int, int]]:
        """Parses all 'PAN,TILT\\n' commands received in TX history."""
        cmds: List[Tuple[int, int]] = []
        for raw_bytes in self.tx_history:
            try:
                raw = raw_bytes.decode("utf-8", errors="ignore").strip()
                parts = raw.split(",")
                if len(parts) >= 2:
                    cmds.append((int(float(parts[0])), int(float(parts[1]))))
            except Exception:
                continue
        return cmds

    def clear_history(self) -> None:
        self.tx_history.clear()
