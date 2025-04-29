import serial
import struct
import time
from collections import deque
from typing import List, Optional

# CRSF Protocol Constants
CRSF_ADDRESS_CRSF_RECEIVER = 0xEE
CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
CRSF_FRAMETYPE_LINK_STATISTICS = 0x14
CRSF_MAX_PACKET_SIZE = 64
CRSF_MAX_CHANNELS = 16
CRSF_FRAME_HEADER_BYTES = 2
CRSF_FRAME_CRC_BYTES = 1


class CRC8:
    """CRC8 calculation class."""

    def __init__(self):
        self.crc8_dvb_s2_table = [
            0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
            0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
            0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
            0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
            0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
            0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
            0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
            0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
            0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
            0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
            0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
            0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
            0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
            0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
            0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
            0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
        ]

    def calculate(self, data: bytes) -> int:
        crc = 0
        for byte in data:
            crc = self.crc8_dvb_s2_table[crc ^ byte]
        return crc


class MedianFilter:
    """Median filter for smoothing channel data."""

    def __init__(self, window_size: int):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)

    def update(self, value: float) -> float:
        self.values.append(value)
        if len(self.values) < self.window_size:
            return value
        sorted_values = sorted(self.values)
        mid = len(sorted_values) // 2
        if len(sorted_values) % 2 == 0:
            return (sorted_values[mid - 1] + sorted_values[mid]) / 2
        return sorted_values[mid]


class AlfredoCRSF:
    """Python implementation of Alfredo CRSF library."""

    def __init__(self, port: str, baudrate: int = 420000, timeout: float = 0.1):
        self.serial = serial.Serial(port, baudrate, timeout=timeout)
        self.crc8 = CRC8()
        self.channels = [1500] * CRSF_MAX_CHANNELS
        self.median_filters = [MedianFilter(3) for _ in range(CRSF_MAX_CHANNELS)]
        self.buffer = bytearray()
        self.last_packet_time = time.time()

    def begin(self):
        """Initialize serial communication."""
        if not self.serial.is_open:
            self.serial.open()

    def read(self) -> bool:
        """Read and process incoming CRSF packets."""
        if not self.serial.is_open:
            return False

        # Read available bytes
        while self.serial.in_waiting > 0:
            self.buffer.extend(self.serial.read(self.serial.in_waiting))

            # Process packets
            while len(self.buffer) >= CRSF_FRAME_HEADER_BYTES:
                # Check for valid packet start
                if self.buffer[0] != CRSF_ADDRESS_FLIGHT_CONTROLLER:
                    self.buffer.pop(0)
                    continue

                packet_length = self.buffer[1]
                if packet_length < 2 or packet_length > CRSF_MAX_PACKET_SIZE:
                    self.buffer.pop(0)
                    continue

                if len(self.buffer) < packet_length + CRSF_FRAME_HEADER_BYTES:
                    break  # Wait for more data

                # Extract packet
                packet = self.buffer[:packet_length + CRSF_FRAME_HEADER_BYTES]
                self.buffer = self.buffer[packet_length + CRSF_FRAME_HEADER_BYTES:]

                # Verify CRC
                crc_received = packet[-1]
                crc_calculated = self.crc8.calculate(packet[2:-1])
                if crc_received != crc_calculated:
                    continue

                # Process packet
                if packet[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                    self._parse_channels(packet[3:-1])
                    self.last_packet_time = time.time()
                    return True
                # Add handling for other packet types (e.g., link statistics) if needed

        return False

    def _parse_channels(self, data: bytes):
        """Parse RC channels from packed data."""
        # CRSF packs 16 channels into 22 bytes, 11 bits per channel
        if len(data) < 22:
            return

        # Unpack channels (11-bit values)
        channels = [0] * CRSF_MAX_CHANNELS
        bit_buffer = 0
        bit_count = 0
        byte_index = 0
        channel_index = 0

        while byte_index < len(data) and channel_index < CRSF_MAX_CHANNELS:
            bit_buffer |= data[byte_index] << bit_count
            bit_count += 8
            byte_index += 1

            while bit_count >= 11 and channel_index < CRSF_MAX_CHANNELS:
                channels[channel_index] = bit_buffer & 0x7FF
                bit_buffer >>= 11
                bit_count -= 11
                channel_index += 1

        # Convert to microseconds (988-2012 range) and apply median filter
        for i in range(CRSF_MAX_CHANNELS):
            raw_value = channels[i]
            # Map 0-1984 (11-bit) to 988-2012 us
            us_value = 988 + (raw_value * (2012 - 988) // 1984)
            self.channels[i] = self.median_filters[i].update(us_value)

    def get_channel(self, channel: int) -> int:
        """Get the value of a specific channel (1-16)."""
        if 1 <= channel <= CRSF_MAX_CHANNELS:
            return self.channels[channel - 1]
        return 1500  # Default middle value

    def get_packet_interval(self) -> float:
        """Get time since last valid packet."""
        return time.time() - self.last_packet_time

    def close(self):
        """Close serial connection."""
        if self.serial.is_open:
            self.serial.close()


# Example usage
if __name__ == "__main__":
    # Replace '/dev/ttyUSB0' with your serial port
    crsf = AlfredoCRSF(port="COM8", baudrate=420000)
    crsf.begin()

    try:
        while True:
            if crsf.read():
                print("Channels:", [crsf.get_channel(i) for i in range(1, 5)])
            time.sleep(0.01)
    except KeyboardInterrupt:
        crsf.close()
        print("Closed")