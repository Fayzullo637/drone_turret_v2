"""
Tier 1 Feature Tests: F15 - Hardware Serial Communications & Simulation Mode.
"""

from __future__ import annotations

import time
import pytest

from drone_turret.comms.serial_comm import (
    CMD_FIRE,
    CMD_HOME,
    CMD_PING,
    ArduinoController,
    MockSerialTransport,
    SerialCommunicator,
    find_arduino_port,
    list_serial_ports,
)


def test_mock_serial_transport_protocol_parsing():
    """Verifies MockSerialTransport correctly parses position, fire, home, and ping commands."""
    transport = MockSerialTransport()
    
    # Position standard format
    transport.write(b"P115,T75\n")
    resp = transport.readline()
    assert resp == b"OK P:115 T:75\n"
    assert transport.get_last_angles() == (115.0, 75.0)

    # Fire command
    transport.write(b"FIRE\n")
    resp_fire = transport.readline()
    assert resp_fire == b"FIRED\n"
    assert transport.get_fire_count() == 1

    # Ping command
    transport.write(b"PING\n")
    resp_ping = transport.readline()
    assert resp_ping == b"PONG\n"

    # Home command
    transport.write(b"HOME\n")
    resp_home = transport.readline()
    assert resp_home == b"OK P:90 T:90\n"
    assert transport.get_last_angles() == (90.0, 90.0)


def test_serial_communicator_simulation_mode():
    """Verifies SerialCommunicator automatically operates in virtual simulation mode without hardware."""
    comm = SerialCommunicator(simulation_mode=True)
    
    assert comm.is_connected is True
    assert comm.is_simulated is True
    assert comm.active_port == "SIMULATION"

    # Send angles
    ok = comm.send_angles(105.0, 82.0, immediate=True)
    assert ok is True
    assert comm.last_angles == (105.0, 82.0)

    # Trigger fire
    ok_fire = comm.fire(immediate=True)
    assert ok_fire is True
    
    status = comm.get_status()
    assert status["simulated"] is True
    assert status["fire_count"] == 1

    comm.disconnect()
    assert comm.is_connected is False


def test_serial_communicator_legacy_format():
    """Verifies legacy '<pan>,<tilt>\n' formatting support for v1 firmware backward compatibility."""
    comm = SerialCommunicator(simulation_mode=True, use_legacy_format=True)
    
    comm.send_angles(120.0, 60.0, immediate=True)
    assert comm.last_angles == (120.0, 60.0)
    
    # Check underlying mock transport history
    assert isinstance(comm._serial, MockSerialTransport)
    history = comm._serial.get_history()
    assert any("120,60\n" in h for h in history)
    
    comm.disconnect()


def test_serial_port_discovery_utilities():
    """Verifies port discovery routines return structured lists without raising exceptions."""
    ports = list_serial_ports()
    assert isinstance(ports, list)
    
    arduino_port = find_arduino_port()
    # Should either be None or a string port name
    assert arduino_port is None or isinstance(arduino_port, str)


def test_serial_communicator_async_queue():
    """Verifies non-blocking background transmission worker drains command queue."""
    comm = SerialCommunicator(simulation_mode=True, tx_rate_hz=100.0)
    
    for i in range(5):
        comm.send_angles(90.0 + i, 90.0 - i, immediate=False)
        
    time.sleep(0.1)  # Allow worker to process queue
    assert comm.last_angles == (94.0, 86.0)
    
    comm.disconnect()
