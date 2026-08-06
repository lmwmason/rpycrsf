"""rpycrsf: control CRSF (Crossfire/ELRS) based drones from a Raspberry Pi."""

from .drone import Drone
from .protocol import (
    CRSF_BAUDRATE,
    CRSF_CHANNEL_VALUE_MAX,
    CRSF_CHANNEL_VALUE_MID,
    CRSF_CHANNEL_VALUE_MIN,
    CRSFPort,
    crc8,
    pack_rc_channels,
    unpack_rc_channels,
)

__version__ = "0.1.0"

__all__ = [
    "Drone",
    "CRSFPort",
    "CRSF_BAUDRATE",
    "CRSF_CHANNEL_VALUE_MIN",
    "CRSF_CHANNEL_VALUE_MID",
    "CRSF_CHANNEL_VALUE_MAX",
    "crc8",
    "pack_rc_channels",
    "unpack_rc_channels",
]
