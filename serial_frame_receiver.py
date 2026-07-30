import serial

class SerialFrameReceiver:
    HEADER = b'\xAA\x55'

    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0)
        self.buffer = bytearray()

    def poll(self):
        """
        Poll the serial port.

        Returns:
            payload (bytes) if a complete frame has been received,
            otherwise None.
        """

        self.buffer.extend(self.ser.read(self.ser.in_waiting))

        while True:

            # Need at least header + length
            if len(self.buffer) < 4:
                return None

            # Find the header
            idx = self.buffer.find(self.HEADER)

            if idx == -1:
                self.buffer.clear()
                return None

            if idx > 0:
                del self.buffer[:idx]

            if len(self.buffer) < 4:
                return None

            length = int.from_bytes(self.buffer[2:4], "little")
            packet_size = 4 + length

            if len(self.buffer) < packet_size:
                return None

            payload = bytes(self.buffer[4:packet_size])
            del self.buffer[:packet_size]

            return payload

    def close(self):
        self.ser.close()

class CSVReceiver:

    def __init__(self, port, baudrate=115200, decoder=float):
        self.ser = serial.Serial(port, baudrate, timeout=0)
        self.buffer = ""
        self.decoder = decoder

    def poll(self):
        self.buffer += self.ser.read(self.ser.in_waiting).decode(
            "ascii", errors="ignore"
        )

        latest = None

        while '\n' in self.buffer:
            line, self.buffer = self.buffer.split('\n', 1)
            line = line.strip()

            if not line:
                continue

            try:
                latest = [self.decoder(x) for x in line.split(',')]
            except ValueError:
                continue

        return latest

    def close(self):
        self.ser.close()