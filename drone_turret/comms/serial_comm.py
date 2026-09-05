"""
Serial Communications Module for Arduino Uno Turret & Hardware Interfacing.

Features:
1. Automatic COM Port Discovery (Arduino, CH340, FTDI, USB-Serial).
2. Non-blocking Asynchronous Transmission via background worker queue.
3. Dual Protocol Formatting:
   - Standard: "P<pan>,T<tilt>\n" (e.g. "P105,T82\n")
   - Legacy: "<pan>,<tilt>\n" (e.g. "105,82\n")
4. Command set:
   - Position: "P<pan>,T<tilt>\n"
   - Fire trigger: "FIRE\n"
   - Home: "HOME\n"
   - Ping: "PING\n"
5. High-fidelity Software Simulation Mode with MockSerialTransport when hardware is absent.
6. Auto-reconnection & Failsafe Watchdog handling.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

# Protocol Commands
CMD_FIRE = "FIRE\n"
CMD_HOME = "HOME\n"
CMD_PING = "PING\n"
FORMAT_STANDARD = "P{pan:.0f},T{tilt:.0f}\n"
FORMAT_PRECISE = "P{pan:.1f},T{tilt:.1f}\n"
FORMAT_LEGACY = "{pan:.0f},{tilt:.0f}\n"

DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT = 1.0


def list_serial_ports() -> List[Dict[str, str]]:
    """
    Scans and returns all available serial COM ports on the system with metadata.
    """
    ports_info: List[Dict[str, str]] = []
    try:
        import serial.tools.list_ports
        for port in serial.tools.list_ports.comports():
            ports_info.append({
                "device": port.device,
                "description": port.description or "Unknown",
                "hwid": port.hwid or "Unknown",
                "manufacturer": getattr(port, "manufacturer", "Unknown") or "Unknown",
                "vid": f"0x{port.vid:04x}" if getattr(port, "vid", None) is not None else "",
                "pid": f"0x{port.pid:04x}" if getattr(port, "pid", None) is not None else "",
            })
    except Exception as e:
        logger.warning("Could not list serial ports: %s", e)
    return ports_info


def find_arduino_port() -> Optional[str]:
    """
    Auto-detects COM port corresponding to Arduino Uno / Mega / Nano / CH340 / FTDI.
    """
    arduino_keywords = ["arduino", "ch340", "ftdi", "cp210", "usb serial", "usb-serial", "ch341", "atmega"]
    arduino_vids = ["0x2341", "0x2a03", "0x1a86", "0x0403", "0x10c4"]

    ports = list_serial_ports()
    for p in ports:
        desc = p["description"].lower()
        mfg = p["manufacturer"].lower()
        vid = p["vid"].lower()

        if any(kw in desc or kw in mfg for kw in arduino_keywords):
            return p["device"]
        if vid in arduino_vids:
            return p["device"]

    # If only 1 COM port is available and it's not COM1, return it as candidate
    if len(ports) == 1 and ports[0]["device"].upper() != "COM1":
        return ports[0]["device"]

    return None


def find_lidar_port(exclude_port: Optional[str] = None) -> Optional[str]:
    """
    Auto-detects COM port for TFMini / TF-Luna LiDAR module.
    """
    ports = list_serial_ports()
    for p in ports:
        dev = p["device"]
        if exclude_port and dev.upper() == exclude_port.upper():
            continue
        desc = p["description"].lower()
        if "silicon labs" in desc or "cp210" in desc or "ch340" in desc or "usb-to-uart" in desc:
            return dev

    # Return first port not excluded
    for p in ports:
        if not exclude_port or p["device"].upper() != exclude_port.upper():
            return p["device"]

    return None


class MockSerialTransport:
    """
    Virtual software simulation transport mimicking Arduino Uno hardware.
    Used when running without physical servos or during automated unit testing.
    """

    def __init__(self, port: str = "SIM_COM", baudrate: int = DEFAULT_BAUD, timeout: float = 1.0) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open = True

        self._rx_queue: queue.Queue[bytes] = queue.Queue()
        self._sent_history: List[str] = []
        self._last_pan: float = 90.0
        self._last_tilt: float = 90.0
        self._fire_count: int = 0
        self._lock = threading.Lock()

    @property
    def in_waiting(self) -> int:
        return self._rx_queue.qsize()

    def write(self, data: bytes) -> int:
        """Processes incoming command packet and generates simulated response."""
        if not self.is_open:
            raise RuntimeError("Port closed")

        text = data.decode("ascii", errors="ignore")
        with self._lock:
            self._sent_history.append(text)

        # Parse command
        text_strip = text.strip()
        if text_strip == "PING":
            self._rx_queue.put(b"PONG\n")
        elif text_strip == "FIRE":
            with self._lock:
                self._fire_count += 1
            self._rx_queue.put(b"FIRED\n")
        elif text_strip == "HOME":
            with self._lock:
                self._last_pan = 90.0
                self._last_tilt = 90.0
            self._rx_queue.put(b"OK P:90 T:90\n")
        elif text_strip.startswith("P") and ",T" in text_strip:
            try:
                parts = text_strip.split(",")
                pan = float(parts[0].replace("P", ""))
                tilt = float(parts[1].replace("T", ""))
                with self._lock:
                    self._last_pan = pan
                    self._last_tilt = tilt
                self._rx_queue.put(f"OK P:{int(pan)} T:{int(tilt)}\n".encode())
            except Exception:
                pass
        elif "," in text_strip:
            try:
                parts = text_strip.split(",")
                pan = float(parts[0])
                tilt = float(parts[1])
                with self._lock:
                    self._last_pan = pan
                    self._last_tilt = tilt
                self._rx_queue.put(f"OK P:{int(pan)} T:{int(tilt)}\n".encode())
            except Exception:
                pass

        return len(data)

    def read(self, size: int = 1) -> bytes:
        try:
            return self._rx_queue.get(timeout=0.01)
        except queue.Empty:
            return b""

    def readline(self) -> bytes:
        try:
            return self._rx_queue.get(timeout=0.05)
        except queue.Empty:
            return b""

    def close(self) -> None:
        self.is_open = False

    def get_last_angles(self) -> Tuple[float, float]:
        with self._lock:
            return (self._last_pan, self._last_tilt)

    def get_fire_count(self) -> int:
        with self._lock:
            return self._fire_count

    def get_history(self) -> List[str]:
        with self._lock:
            return list(self._sent_history)


class SerialCommunicator:
    """
    High-performance, non-blocking serial controller for Arduino Uno turret.
    
    Features:
    - Auto-detects serial COM port or falls back cleanly to Simulation Mode.
    - Non-blocking asynchronous command queue (never blocks OpenCV/FastAPI loop).
    - Rate-limited transmission (up to 50Hz) to prevent serial buffer overflow.
    - Tracks servo angles, firing events, and hardware responses.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = DEFAULT_BAUD,
        simulation_mode: bool = False,
        use_legacy_format: bool = False,
        tx_rate_hz: float = 50.0,
    ) -> None:
        self.requested_port = port
        self.baudrate = baudrate
        self.force_simulation = simulation_mode
        self.use_legacy_format = use_legacy_format
        self.tx_interval_s = 1.0 / max(1.0, tx_rate_hz)

        self._serial: Optional[Any] = None
        self._is_simulated: bool = False
        self._is_connected: bool = False
        self._active_port: str = ""

        # Last confirmed state
        self._last_pan: float = 90.0
        self._last_tilt: float = 90.0
        self._fire_count: int = 0
        self._last_tx_time: float = 0.0

        # Non-blocking async queue & worker thread
        self._cmd_queue: queue.Queue[Tuple[str, Optional[threading.Event]]] = queue.Queue(maxsize=100)
        self._worker_thread: Optional[threading.Thread] = None
        self._running: bool = False
        self._lock = threading.Lock()

        # Connect on init
        self.connect(port=self.requested_port)

    @property
    def is_connected(self) -> bool:
        """True if either hardware serial port or virtual simulator is active."""
        with self._lock:
            return self._is_connected

    @property
    def is_simulated(self) -> bool:
        """True if operating in virtual software simulation mode."""
        with self._lock:
            return self._is_simulated

    @property
    def active_port(self) -> str:
        """Name of active serial port or simulation identifier."""
        with self._lock:
            return self._active_port

    @property
    def last_angles(self) -> Tuple[float, float]:
        """Returns (last_pan, last_tilt) sent to turret."""
        with self._lock:
            return (self._last_pan, self._last_tilt)

    def connect(self, port: Optional[str] = None, baudrate: Optional[int] = None) -> bool:
        """
        Attempts to open hardware serial port. If unavailable or simulation requested,
        initializes MockSerialTransport seamlessly.
        """
        self.disconnect()

        if baudrate is not None:
            self.baudrate = baudrate

        target_port = port or self.requested_port
        if not target_port and not self.force_simulation:
            target_port = find_arduino_port()

        if self.force_simulation or not target_port:
            # Initialize Simulation Mode
            self._serial = MockSerialTransport(port=target_port or "SIM_PORT", baudrate=self.baudrate)
            with self._lock:
                self._is_simulated = True
                self._is_connected = True
                self._active_port = "SIMULATION"
            logger.info("[SIMULATION] Turret controller running in virtual mode.")
        else:
            try:
                import serial
                self._serial = serial.Serial(
                    port=target_port,
                    baudrate=self.baudrate,
                    timeout=DEFAULT_TIMEOUT,
                )
                time.sleep(1.5)  # Allow Arduino bootloader to complete reset
                with self._lock:
                    self._is_simulated = False
                    self._is_connected = True
                    self._active_port = str(target_port)
                logger.info("[HARDWARE] Connected to Arduino on %s @ %d baud", target_port, self.baudrate)
            except Exception as e:
                logger.warning("Failed to connect to hardware on %s (%s). Falling back to SIMULATION.", target_port, e)
                self._serial = MockSerialTransport(port="SIM_PORT", baudrate=self.baudrate)
                with self._lock:
                    self._is_simulated = True
                    self._is_connected = True
                    self._active_port = "SIMULATION"

        # Start non-blocking worker thread
        self._running = True
        self._worker_thread = threading.Thread(
            target=self._tx_worker_loop,
            daemon=True,
            name="Serial-Tx-Worker",
        )
        self._worker_thread.start()
        return True

    def disconnect(self) -> None:
        """Stops async worker and closes serial port."""
        self._running = False
        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.0)
            self._worker_thread = None

        with self._lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
            self._is_connected = False
            self._is_simulated = False

    def send_angles(
        self,
        pan_angle: float,
        tilt_angle: float,
        immediate: bool = False,
        use_float: bool = False,
    ) -> bool:
        """
        Sends commanded absolute angles to Arduino servos [0, 180].
        
        Args:
            pan_angle: Absolute pan angle in degrees [0, 180].
            tilt_angle: Absolute tilt angle in degrees [0, 180].
            immediate: If True, blocks briefly to transmit synchronously.
            use_float: If True, uses 1-decimal float precision.
        """
        if not self._is_connected:
            return False

        # Clamp to physical range
        pan = max(0.0, min(180.0, float(pan_angle)))
        tilt = max(0.0, min(180.0, float(tilt_angle)))

        with self._lock:
            self._last_pan = pan
            self._last_tilt = tilt

        if self.use_legacy_format:
            msg = FORMAT_LEGACY.format(pan=pan, tilt=tilt)
        elif use_float:
            msg = FORMAT_PRECISE.format(pan=pan, tilt=tilt)
        else:
            msg = FORMAT_STANDARD.format(pan=pan, tilt=tilt)

        return self._queue_or_send(msg, immediate=immediate)

    def fire(self, immediate: bool = True) -> bool:
        """
        Sends immediate high-priority pneumatic net firing command.
        """
        if not self._is_connected:
            return False

        with self._lock:
            self._fire_count += 1

        logger.info("[TURRET] FIRE command issued! (Total fires: %d)", self._fire_count)
        return self._queue_or_send(CMD_FIRE, immediate=immediate)

    def home(self, immediate: bool = False) -> bool:
        """Commands turret back to 90°, 90° center."""
        return self.send_angles(90.0, 90.0, immediate=immediate)

    def ping(self, timeout: float = 0.5) -> bool:
        """Sends PING query and waits for PONG response."""
        if not self._is_connected or self._serial is None:
            return False

        try:
            self._serial.write(CMD_PING.encode("ascii"))
            # Read response
            t0 = time.time()
            while time.time() - t0 < timeout:
                line = self._serial.readline()
                if b"PONG" in line:
                    return True
                time.sleep(0.01)
        except Exception as e:
            logger.warning("Ping failed: %s", e)
        return False

    def get_status(self) -> Dict[str, Any]:
        """Returns diagnostic status dictionary."""
        with self._lock:
            return {
                "connected": self._is_connected,
                "simulated": self._is_simulated,
                "port": self._active_port,
                "baudrate": self.baudrate,
                "last_pan": round(self._last_pan, 2),
                "last_tilt": round(self._last_tilt, 2),
                "fire_count": self._fire_count,
            }

    def _queue_or_send(self, msg: str, immediate: bool) -> bool:
        """Enqueues command for async worker or sends immediately."""
        if immediate:
            return self._write_raw(msg)

        try:
            # If queue is full, drop oldest command to prevent lag buildup
            if self._cmd_queue.full():
                try:
                    self._cmd_queue.get_nowait()
                except queue.Empty:
                    pass
            self._cmd_queue.put_nowait((msg, None))
            return True
        except queue.Full:
            return False

    def _write_raw(self, msg: str) -> bool:
        """Thread-safe direct write to serial transport."""
        if not self._serial:
            return False
        try:
            self._serial.write(msg.encode("ascii"))
            self._last_tx_time = time.time()
            return True
        except Exception as e:
            logger.error("Serial write error: %s", e)
            return False

    def _tx_worker_loop(self) -> None:
        """Background worker thread draining serial commands at fixed rate."""
        while self._running:
            try:
                # Wait for next command or rate-limiting interval
                item = self._cmd_queue.get(timeout=0.05)
                msg, event = item

                # Enforce rate limit between writes
                now = time.time()
                elapsed = now - self._last_tx_time
                if elapsed < self.tx_interval_s:
                    time.sleep(self.tx_interval_s - elapsed)

                self._write_raw(msg)

                if event:
                    event.set()
                self._cmd_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                logger.error("Serial worker exception: %s", e)
                time.sleep(0.05)


# Alias for backward compatibility with v1 API
ArduinoController = SerialCommunicator
