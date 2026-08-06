"""Low level CRSF (Crossfire) protocol helpers: framing, CRC8, and RC channel packing.

Most users should use :class:`rpycrsf.Drone` instead of this module directly.
This module only deals with byte framing and does not know about arming,
sticks, or update rates.
"""

from __future__ import annotations

import serial

CRSF_BAUDRATE = 420000
CRSF_SYNC_BYTE = 0xC8
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16

CRSF_CHANNEL_VALUE_MIN = 172
CRSF_CHANNEL_VALUE_MID = 992
CRSF_CHANNEL_VALUE_MAX = 1811

CRSF_NUM_CHANNELS = 16
CRSF_MAX_PAYLOAD_LEN = 62


def crc8(data: bytes) -> int:
    """CRC-8/DVB-S2 (poly 0xD5), the checksum used by every CRSF frame."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def pack_rc_channels(channels: list[int]) -> bytes:
    """Pack 16 channel values (11-bit, 172..1811) into an RC_CHANNELS_PACKED frame."""
    if len(channels) != CRSF_NUM_CHANNELS:
        raise ValueError(f"expected {CRSF_NUM_CHANNELS} channel values, got {len(channels)}")

    bits = 0
    bit_len = 0
    for ch in channels:
        bits |= (int(ch) & 0x7FF) << bit_len
        bit_len += 11
    payload = bits.to_bytes(22, byteorder="little")

    frame = bytearray()
    frame.append(CRSF_SYNC_BYTE)
    frame.append(len(payload) + 2)
    frame.append(CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    frame.extend(payload)
    frame.append(crc8(frame[2:]))
    return bytes(frame)


def unpack_rc_channels(payload: bytes) -> list[int]:
    """Inverse of :func:`pack_rc_channels`: a 22-byte payload -> 16 channel values."""
    if len(payload) != 22:
        raise ValueError("expected a 22-byte RC_CHANNELS_PACKED payload")
    bits = int.from_bytes(payload, byteorder="little")
    return [(bits >> (11 * i)) & 0x7FF for i in range(CRSF_NUM_CHANNELS)]


class CRSFPort:
    """Raw CRSF serial port for sending arbitrary frames and reading telemetry.

    This is the low level building block :class:`rpycrsf.Drone` is built on.
    Use it directly only if you need custom frame types or telemetry frames.
    """

    def __init__(self, device: str, baudrate: int = CRSF_BAUDRATE, timeout: float = 0.1):
        self.ser = serial.Serial(device, baudrate, timeout=timeout, write_timeout=timeout)

    def write_frame(self, frame: bytes) -> None:
        self.ser.write(frame)

    def read_frame(self) -> tuple[int, bytes] | None:
        """Read and CRC-validate a single incoming frame.

        Returns ``(frame_type, payload)`` or ``None`` if no valid frame was
        available within the port timeout.
        """
        sync = self.ser.read(1)
        if len(sync) != 1 or sync[0] != CRSF_SYNC_BYTE:
            return None
        length_byte = self.ser.read(1)
        if len(length_byte) != 1:
            return None
        length = length_byte[0]
        if length < 2 or length > CRSF_MAX_PAYLOAD_LEN:
            return None
        buf = self.ser.read(length)
        if len(buf) != length:
            return None
        if crc8(buf[:-1]) != buf[-1]:
            return None
        return buf[0], buf[1:-1]

    def close(self) -> None:
        self.ser.close()

    def __enter__(self) -> "CRSFPort":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
