import io
import struct
import time

class MockSerialPort:
    """Thread-safe / in-memory mock of serial.Serial for automated testing."""
    def __init__(self, port="COM_MOCK", baudrate=115200, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open = True
        self.rx_buffer = bytearray()
        self.tx_history = []
        
    def write(self, data: bytes) -> int:
        if not self.is_open:
            raise IOError("Port closed")
        self.tx_history.append(data)
        return len(data)
        
    def read(self, size: int = 1) -> bytes:
        if not self.is_open:
            raise IOError("Port closed")
        if not self.rx_buffer:
            return b""
        read_bytes = self.rx_buffer[:size]
        self.rx_buffer = self.rx_buffer[size:]
        return bytes(read_bytes)
        
    def readline(self) -> bytes:
        if not self.is_open:
            raise IOError("Port closed")
        idx = self.rx_buffer.find(b"\n")
        if idx == -1:
            # return all if timeout
            res = bytes(self.rx_buffer)
            self.rx_buffer.clear()
            return res
        res = bytes(self.rx_buffer[:idx+1])
        self.rx_buffer = self.rx_buffer[idx+1:]
        return res
        
    @property
    def in_waiting(self) -> int:
        return len(self.rx_buffer)
        
    def inject_rx_data(self, data: bytes):
        self.rx_buffer.extend(data)
        
    def close(self):
        self.is_open = False

class MockTFMiniLiDAR:
    """Generates TFMini / TF-Luna binary frames: 9 bytes per measurement."""
    @staticmethod
    def create_packet(dist_cm: int, strength: int = 1200, temp_c: float = 25.0) -> bytes:
        dist_l = dist_cm & 0xFF
        dist_h = (dist_cm >> 8) & 0xFF
        str_l = strength & 0xFF
        str_h = (strength >> 8) & 0xFF
        temp_raw = int(temp_c * 10) # 0.1 deg C
        temp_l = temp_raw & 0xFF
        temp_h = (temp_raw >> 8) & 0xFF
        
        payload = [0x59, 0x59, dist_l, dist_h, str_l, str_h, temp_l, temp_h]
        checksum = sum(payload) & 0xFF
        payload.append(checksum)
        return bytes(payload)

class TFMiniParser:
    """Parses binary stream from LiDAR serial port."""
    def __init__(self, serial_dev):
        self.serial = serial_dev
        self.buffer = bytearray()
        
    def read_distance(self):
        """Returns distance in meters, or None if no valid packet available."""
        if self.serial.in_waiting > 0:
            self.buffer.extend(self.serial.read(self.serial.in_waiting))
            
        while len(self.buffer) >= 9:
            # Look for header 0x59 0x59
            if self.buffer[0] == 0x59 and self.buffer[1] == 0x59:
                packet = self.buffer[:9]
                checksum = sum(packet[:8]) & 0xFF
                if checksum == packet[8]:
                    dist_cm = packet[2] | (packet[3] << 8)
                    strength = packet[4] | (packet[5] << 8)
                    temp_c = (packet[6] | (packet[7] << 8)) / 10.0
                    self.buffer = self.buffer[9:]
                    
                    # Check signal validity
                    if strength < 100 or dist_cm == 0xFFFF:
                        return None # Signal too weak or invalid
                    return dist_cm / 100.0 # Return meters
                else:
                    # Checksum mismatch, advance 1 byte
                    self.buffer = self.buffer[1:]
            else:
                self.buffer = self.buffer[1:]
        return None

def test_mock_serial():
    print("--- MOCK SERIAL & LIDAR PROTOCOL TEST ---")
    mock_port = MockSerialPort()
    parser = TFMiniParser(mock_port)
    
    # Test 1: Send valid 25.50m (2550cm) packet
    p1 = MockTFMiniLiDAR.create_packet(2550, strength=2000)
    mock_port.inject_rx_data(p1)
    d1 = parser.read_distance()
    print(f"Test 1 (25.50m nominal): Parsed = {d1} m (Expected: 25.50)")
    assert abs(d1 - 25.50) < 1e-3
    
    # Test 2: Corrupt byte stream followed by valid 12.34m packet
    garbage = b"\x00\xFF\x59\x12\x34"
    p2 = MockTFMiniLiDAR.create_packet(1234, strength=1500)
    mock_port.inject_rx_data(garbage + p2)
    d2 = parser.read_distance()
    print(f"Test 2 (Garbage recovery + 12.34m): Parsed = {d2} m (Expected: 12.34)")
    assert abs(d2 - 12.34) < 1e-3

    # Test 3: Low signal strength packet (strength=50 < 100 threshold)
    p3 = MockTFMiniLiDAR.create_packet(4000, strength=50)
    mock_port.inject_rx_data(p3)
    d3 = parser.read_distance()
    print(f"Test 3 (Weak signal < 100): Parsed = {d3} (Expected: None for optical fallback)")
    assert d3 is None
    
    # Test 4: Arduino command sending
    mock_port.write(b"95,87\n")
    print(f"Test 4 (Arduino TX): History = {mock_port.tx_history}")
    assert mock_port.tx_history == [b"95,87\n"]
    print("All mock serial tests PASSED!")

if __name__ == '__main__':
    test_mock_serial()
