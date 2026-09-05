"""
Comms package: Serial hardware communicator, port discovery, and simulation transports.
"""

from drone_turret.comms.serial_comm import (
    CMD_FIRE,
    CMD_HOME,
    CMD_PING,
    DEFAULT_BAUD,
    FORMAT_LEGACY,
    FORMAT_PRECISE,
    FORMAT_STANDARD,
    ArduinoController,
    MockSerialTransport,
    SerialCommunicator,
    find_arduino_port,
    find_lidar_port,
    list_serial_ports,
)

__all__ = [
    "SerialCommunicator",
    "ArduinoController",
    "MockSerialTransport",
    "list_serial_ports",
    "find_arduino_port",
    "find_lidar_port",
    "CMD_FIRE",
    "CMD_HOME",
    "CMD_PING",
    "FORMAT_STANDARD",
    "FORMAT_PRECISE",
    "FORMAT_LEGACY",
    "DEFAULT_BAUD",
]
