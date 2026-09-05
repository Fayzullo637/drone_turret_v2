"""Tier 1 Unit Tests: Arduino Serial Protocol & Simulation Mode (Feature F15).

Verifies ASCII command formatting 'PAN,TILT\\n', servo angle integer clamping,
fire trigger commands, simulation fallback, and command history logging.
"""

from __future__ import annotations

from typing import List, Optional, Tuple
import pytest

from tests.fixtures.mock_serial import MockSerialPort


class ArduinoSerialCommunicator:
    """Manages serial communications with Arduino Uno or in-memory simulation mock."""

    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, mock_serial: Optional[MockSerialPort] = None):
        self.port_name = port
        self.baudrate = baudrate
        self.serial = mock_serial or MockSerialPort(port=port or "SIMULATION", baudrate=baudrate)
        self.simulation_mode = mock_serial is None and (port is None or port == "SIMULATION")
        self.last_sent_pan = 90
        self.last_sent_tilt = 90

    def send_angles(self, pan_deg: float, tilt_deg: float) -> str:
        """Clamps angles to [0, 180], rounds to integer, and transmits 'PAN,TILT\\n'."""
        pan_int = int(round(max(0.0, min(180.0, pan_deg))))
        tilt_int = int(round(max(0.0, min(180.0, tilt_deg))))

        self.last_sent_pan = pan_int
        self.last_sent_tilt = tilt_int

        cmd_str = f"{pan_int},{tilt_int}\n"
        self.serial.write(cmd_str.encode("ascii"))
        return cmd_str

    def send_fire_command(self) -> str:
        """Sends fire trigger command 'FIRE\\n'."""
        cmd = "FIRE\n"
        self.serial.write(cmd.encode("ascii"))
        return cmd

    def close(self) -> None:
        self.serial.close()


def test_arduino_format_command(mock_serial):
    """T1.9.1: Verifies pan=95.4, tilt=87.2 formats to '95,87\\n'."""
    comm = ArduinoSerialCommunicator(mock_serial=mock_serial)
    cmd = comm.send_angles(95.4, 87.2)

    assert cmd == "95,87\n"
    assert mock_serial.get_last_command() == (95, 87)


def test_arduino_simulation_mode_fallback():
    """T1.9.2: Verifies initializing without hardware port enters simulation mode cleanly."""
    comm = ArduinoSerialCommunicator(port=None)
    assert comm.simulation_mode is True
    cmd = comm.send_angles(120, 60)
    assert cmd == "120,60\n"


def test_arduino_fire_command(mock_serial):
    """T1.9.3: Verifies fire trigger command 'FIRE\\n' is transmitted."""
    comm = ArduinoSerialCommunicator(mock_serial=mock_serial)
    comm.send_fire_command()

    assert len(mock_serial.tx_history) == 1
    assert mock_serial.tx_history[0] == b"FIRE\n"


def test_arduino_command_clamping(mock_serial):
    """T1.9.4: Verifies out-of-range angles (-15, 210) are clamped to [0, 180]."""
    comm = ArduinoSerialCommunicator(mock_serial=mock_serial)
    cmd = comm.send_angles(-15.0, 210.0)

    assert cmd == "0,180\n"
    assert mock_serial.get_last_command() == (0, 180)


def test_arduino_command_history_tracking(mock_serial):
    """T1.9.5: Verifies sequence of commands is properly preserved in TX history."""
    comm = ArduinoSerialCommunicator(mock_serial=mock_serial)
    comm.send_angles(90, 90)
    comm.send_angles(95, 92)
    comm.send_angles(100, 95)

    history = mock_serial.get_command_history()
    assert len(history) == 3
    assert history == [(90, 90), (95, 92), (100, 95)]


def test_arduino_close_port(mock_serial):
    """T1.9.6: Verifies serial connection terminates properly upon close()."""
    comm = ArduinoSerialCommunicator(mock_serial=mock_serial)
    assert mock_serial.is_open is True
    comm.close()
    assert mock_serial.is_open is False
