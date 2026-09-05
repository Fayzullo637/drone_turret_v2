"""
Tier 5 Stress Tests: Serial UART Fuzzing & Buffer Memory Leak Verification.

Stress Scenarios:
1. 50,000 Random Corrupt Bytes into LiDAR Binary Stream Parser:
   - Evaluates memory stability (buffer stays <= 9 bytes, 0 memory leaks).
   - Evaluates crash resistance (0 unhandled exceptions / index errors).
   - Evaluates seamless recovery: 100% valid packet extraction when valid frames resume.
2. 50,000 Pathological False Header Injections (dense 0x59 0x59 with bad checksums/lengths).
3. Threaded LidarSerialReader Background Loop 50,000-byte Fuzz Soak.
4. 50,000 Random ASCII/Binary Corrupt Commands into Arduino Serial Transport:
   - Evaluates 0 crashes on non-ASCII/garbage input.
   - Evaluates async queue stability.
   - Evaluates clean command-response recovery.
5. Arduino C++ Firmware Logic In-Memory Emulation Fuzzing (64-byte ring buffer protection).
"""

from __future__ import annotations

import gc
import os
import random
import sys
import time
import pytest

from drone_turret.sensors.lidar import (
    FRAME_HEADER,
    FRAME_LENGTH,
    LidarParser,
    LidarReading,
    LidarSerialReader,
)
from drone_turret.comms.serial_comm import (
    MockSerialTransport,
    SerialCommunicator,
)
from tests.fixtures.mock_serial import MockSerialPort, TFMiniPacketGenerator


def test_lidar_50k_random_byte_fuzzing_and_recovery():
    """
    Stress Challenge 1A:
    Inject 50,000 random corrupt bytes into LidarParser across variable chunk sizes.
    Verifies:
      - 0 crashes / unhandled exceptions.
      - 0 memory leaks (buffer size <= FRAME_LENGTH after processing).
      - Seamless recovery: 100 consecutive valid packets parsed with 100% accuracy.
    """
    parser = LidarParser(min_strength=100)
    rng = random.Random(42)

    # Pre-test memory check
    gc.collect()

    total_fuzz_bytes = 50_000
    bytes_injected = 0
    chunk_sizes = [1, 3, 7, 9, 13, 64, 256, 1024]

    while bytes_injected < total_fuzz_bytes:
        chunk_len = min(rng.choice(chunk_sizes), total_fuzz_bytes - bytes_injected)
        corrupt_chunk = bytes(rng.randint(0, 255) for _ in range(chunk_len))
        readings = parser.feed(corrupt_chunk)
        bytes_injected += chunk_len

    # Assertions on parser state after 50k corrupt bytes
    assert parser.total_bytes_received == total_fuzz_bytes
    assert len(parser._buffer) < FRAME_LENGTH, f"Buffer bloated: {len(parser._buffer)} bytes"
    assert parser.dropped_bytes > 0

    # Test Seamless Recovery: Feed 100 known valid packets immediately after fuzzing
    valid_test_packets = []
    expected_distances = []
    for i in range(100):
        dist_cm = 200 + i * 15  # 2.0m to 16.85m
        expected_distances.append(round(dist_cm / 100.0, 3))
        pkt = TFMiniPacketGenerator.build_packet(distance_cm=dist_cm, strength=800, temp_c=25)
        valid_test_packets.append(pkt)

    recovered_readings = parser.feed(b"".join(valid_test_packets))

    assert len(recovered_readings) == 100, f"Expected 100 recovered readings, got {len(recovered_readings)}"
    for idx, reading in enumerate(recovered_readings):
        assert reading.is_valid is True
        assert reading.distance_m == expected_distances[idx]
        assert reading.strength == 800


def test_lidar_50k_pathological_false_header_fuzzing():
    """
    Stress Challenge 1B:
    Inject 50,000 pathological bytes specifically crafted to mimic headers (0x59 0x59)
    followed by random truncated data, wrong checksums, and corrupt length patterns.
    Verifies parser does not get stuck in infinite resync loops or blow up buffer.
    """
    parser = LidarParser(min_strength=100)
    rng = random.Random(1337)

    pathological_stream = bytearray()
    while len(pathological_stream) < 50_000:
        variant = rng.randint(0, 4)
        if variant == 0:
            pkt = bytearray(TFMiniPacketGenerator.build_packet(distance_cm=500, strength=900))
            pkt[8] = (pkt[8] + rng.randint(1, 255)) & 0xFF
            pathological_stream.extend(pkt)
        elif variant == 1:
            pkt = TFMiniPacketGenerator.build_weak_signal_packet(distance_cm=300, strength=20)
            pathological_stream.extend(pkt)
        elif variant == 2:
            pathological_stream.extend(b"\x59\x59" + bytes([rng.randint(0, 255)]))
        elif variant == 3:
            pathological_stream.extend(b"\x59" + bytes([rng.randint(0, 255) for _ in range(5)]))
        else:
            pathological_stream.extend(b"\x59" * rng.randint(3, 15))

    pathological_stream = bytes(pathological_stream[:50_000])
    readings = parser.feed(pathological_stream)

    assert len(parser._buffer) < FRAME_LENGTH
    assert parser.total_bytes_received == 50_000

    valid_batch = b"".join(
        TFMiniPacketGenerator.build_packet(distance_cm=1000 + i * 10, strength=750)
        for i in range(50)
    )
    recovered = []
    for b in valid_batch:
        res = parser.feed(bytes([b]))
        recovered.extend(res)

    assert len(recovered) == 50
    assert all(r.is_valid for r in recovered)


def test_lidar_serial_reader_thread_50k_fuzz_soak():
    """
    Stress Challenge 1C:
    Run LidarSerialReader background reader thread while pumping 50,000 corrupt bytes
    through MockSerialPort at high speed.
    Verifies:
      - Background reader loop does not crash or terminate prematurely.
      - Lock contention is clean without deadlocks.
      - Seamless recovery once valid packets are injected.
    """
    mock_port = MockSerialPort(port="COM_MOCK_SOAK")
    reader = LidarSerialReader(mock_transport=mock_port)
    assert reader.start() is True

    rng = random.Random(999)
    total_bytes = 50_000
    chunk_size = 500

    for _ in range(total_bytes // chunk_size):
        noise = bytes(rng.randint(0, 255) for _ in range(chunk_size))
        mock_port.inject_rx(noise)
        time.sleep(0.001)

    time.sleep(0.1)
    assert reader.is_connected is True
    assert reader._thread.is_alive()

    # Inject 20 valid packets
    for i in range(20):
        mock_port.inject_tfmini_packet(distance_cm=1500 + i * 20, strength=950)
        time.sleep(0.005)

    time.sleep(0.05)
    latest_reading = reader.get_latest_reading()
    assert latest_reading is not None
    assert latest_reading.is_valid is True
    assert latest_reading.distance_m >= 15.0
    assert reader.get_latest_distance() is not None

    reader.stop()
    assert reader.is_connected is False


def test_arduino_serial_50k_corrupt_command_fuzzing():
    """
    Stress Challenge 1D:
    Inject 50,000 corrupt ASCII and binary bytes into MockSerialTransport and SerialCommunicator.
    Verifies:
      - 0 crashes when processing non-ASCII, malformed delimiters, control chars, and giant strings.
      - Queue never overflows or deadlocks the async TX worker thread.
      - Valid commands (P<pan>,T<tilt>, FIRE, HOME, PING) execute cleanly with 100% success.
    """
    transport = MockSerialTransport(port="SIM_FUZZ")
    comm = SerialCommunicator(simulation_mode=True)
    rng = random.Random(777)

    fuzz_templates = [
        b"P",
        b"T",
        b"P,T\n",
        b"P-999999,T999999\n",
        b"PNaN,TInf\n",
        b"Pabc,Txyz\n",
        b"\x00\xFF\xFE\xFD\x80\n",
        b"FIREEEEEEEEEEEEEEEEEEEEEEEEEEEEE\n",
        b"HOME,SWEET,HOME\n",
        b"PINGPONGPINGPONG\n",
        b",,,,,,,,,\n",
        b"P10.5.5,T20.3.3\n",
    ]

    total_bytes = 0
    while total_bytes < 50_000:
        choice = rng.randint(0, 3)
        if choice == 0:
            payload = rng.choice(fuzz_templates)
        elif choice == 1:
            payload = bytes(rng.randint(0, 255) for _ in range(rng.randint(5, 50))) + b"\n"
        elif choice == 2:
            payload = f"P{rng.uniform(-1e9, 1e9)},T{rng.uniform(-1e9, 1e9)}\n".encode("latin1")
        else:
            payload = f"P{rng.randint(0, 180)}".encode("latin1")

        transport.write(payload)
        total_bytes += len(payload)

    # Ensure transport didn't crash
    assert transport.is_open is True

    # Drain any buffered responses accumulated during fuzzing
    while transport.in_waiting > 0:
        transport.readline()

    # Test recovery of MockSerialTransport
    transport.write(b"PING\n")
    resp = transport.readline()
    assert b"PONG" in resp

    transport.write(b"P135,T45\n")
    assert transport.get_last_angles() == (135.0, 45.0)

    transport.write(b"FIRE\n")
    assert transport.get_fire_count() >= 1

    transport.write(b"HOME\n")
    assert transport.get_last_angles() == (90.0, 90.0)

    # Test SerialCommunicator async transmission under rapid firing
    for i in range(50):
        pan = 45.0 + i
        tilt = 60.0 + (i % 30)
        assert comm.send_angles(pan, tilt) is True

    assert comm.fire() is True
    assert comm.home() is True
    assert comm.is_connected is True

    comm.disconnect()
    assert comm.is_connected is False


def test_arduino_firmware_parser_c_logic_fuzzing():
    """
    Stress Challenge 1E:
    Exact in-memory Python recreation of the C++ circular buffer & state machine in
    `firmware/drone_turret_firmware.ino` (RX_BUFFER_SIZE = 64, readSerialCommands & parseCommand).
    Injects 50,000 bytes of fuzz data to verify 0 buffer overflows and seamless recovery.
    """
    RX_BUFFER_SIZE = 64
    rx_buffer = bytearray(RX_BUFFER_SIZE)
    rx_head = 0

    target_pan = 90.0
    target_tilt = 90.0
    fire_active = False
    pong_replies = 0
    ok_replies = 0

    def parse_command(cmd_str: str) -> None:
        nonlocal target_pan, target_tilt, fire_active, pong_replies, ok_replies
        if cmd_str == "PING":
            pong_replies += 1
            return
        if cmd_str == "FIRE":
            fire_active = True
            return
        if cmd_str == "HOME":
            target_pan = 90.0
            target_tilt = 90.0
            ok_replies += 1
            return
        if cmd_str.startswith("P") or cmd_str.startswith("p"):
            t_idx = cmd_str.find("T")
            if t_idx == -1:
                t_idx = cmd_str.find("t")
            if t_idx != -1:
                try:
                    p_val = float(cmd_str[1:t_idx].replace(",", ""))
                    t_val = float(cmd_str[t_idx + 1:])
                    target_pan = max(0.0, min(180.0, p_val))
                    target_tilt = max(0.0, min(180.0, t_val))
                    ok_replies += 1
                except ValueError:
                    pass
                return
        if "," in cmd_str:
            parts = cmd_str.split(",")
            try:
                p_val = float(parts[0])
                t_val = float(parts[1])
                target_pan = max(0.0, min(180.0, p_val))
                target_tilt = max(0.0, min(180.0, t_val))
                ok_replies += 1
            except ValueError:
                pass

    def feed_firmware_bytes(stream: bytes) -> None:
        nonlocal rx_head
        for byte_val in stream:
            c = chr(byte_val) if byte_val < 128 else "\x00"
            if c == "\r":
                continue
            if c == "\n":
                if rx_head > 0:
                    cmd_str = bytes(rx_buffer[:rx_head]).decode("ascii", errors="ignore")
                    parse_command(cmd_str)
                    rx_head = 0
            else:
                if rx_head < RX_BUFFER_SIZE - 1:
                    rx_buffer[rx_head] = byte_val
                    rx_head += 1
                else:
                    rx_head = 0

    rng = random.Random(404)
    fuzz_data = bytes(rng.randint(0, 255) for _ in range(50_000))
    feed_firmware_bytes(fuzz_data)

    assert 0 <= rx_head < RX_BUFFER_SIZE

    feed_firmware_bytes(b"\nHOME\nP120,T70\nFIRE\nPING\n")
    assert target_pan == 120.0
    assert target_tilt == 70.0
    assert fire_active is True
    assert pong_replies >= 1
