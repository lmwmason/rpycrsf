"""High level CRSF drone control.

A CRSF-speaking flight controller expects RC channel frames at a steady
rate (typically well above the ~4 Hz failsafe threshold); if frames stop
arriving it will assume link loss and trigger its failsafe. :class:`Drone`
opens the serial port and runs a background thread that keeps re-sending
the current channel state at ``update_rate_hz``, so callers only need to
update the desired stick/switch values and the link stays alive on its own.
"""

from __future__ import annotations

import logging
import threading
import time

import serial

from .protocol import (
    CRSF_BAUDRATE,
    CRSF_CHANNEL_VALUE_MAX,
    CRSF_CHANNEL_VALUE_MID,
    CRSF_CHANNEL_VALUE_MIN,
    CRSF_NUM_CHANNELS,
    pack_rc_channels,
)

logger = logging.getLogger(__name__)

# Standard AETR channel order used by CRSF/ELRS.
CH_ROLL = 0
CH_PITCH = 1
CH_THROTTLE = 2
CH_YAW = 3
CH_AUX1 = 4  # ARM
CH_AUX2 = 5  # MODE (angle/acro)
CH_AUX3 = 6  # ALTHOLD

DEFAULT_UPDATE_RATE_HZ = 150.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _bipolar_to_crsf(value: float) -> int:
    """Map -1.0..1.0 (roll/pitch/yaw) to CRSF_CHANNEL_VALUE_MIN..MAX."""
    value = _clamp(value, -1.0, 1.0)
    half_span = (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN) / 2
    return round(CRSF_CHANNEL_VALUE_MID + value * half_span)


def _unipolar_to_crsf(value: float) -> int:
    """Map 0.0..1.0 (throttle) to CRSF_CHANNEL_VALUE_MIN..MAX."""
    value = _clamp(value, 0.0, 1.0)
    return round(CRSF_CHANNEL_VALUE_MIN + value * (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN))


class Drone:
    """Control a CRSF/ELRS based drone over a Raspberry Pi serial port.

    Example::

        with Drone("/dev/serial0") as d:
            d.arm(True)
            d.set_mode(True)
            d.set_sticks(throttle=0.5)
            time.sleep(5)
    """

    def __init__(
        self,
        device: str,
        baudrate: int = CRSF_BAUDRATE,
        update_rate_hz: float = DEFAULT_UPDATE_RATE_HZ,
        auto_start: bool = True,
    ):
        self._lock = threading.Lock()
        self._channels = [CRSF_CHANNEL_VALUE_MID] * CRSF_NUM_CHANNELS
        self._channels[CH_THROTTLE] = CRSF_CHANNEL_VALUE_MIN
        self._channels[CH_AUX1] = CRSF_CHANNEL_VALUE_MIN  # start disarmed

        # exclusive=True keeps a second process (or a re-run of this script)
        # from silently stealing the port out from under a live link.
        self.ser = serial.Serial(device, baudrate, timeout=0.1, write_timeout=0.1, exclusive=True)

        self._update_period = 1.0 / update_rate_hz
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        if auto_start:
            self.start()

    # -- lifecycle -----------------------------------------------------

    def start(self) -> None:
        """Start the background thread that keeps the link alive."""
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the background sender thread without closing the port."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def close(self) -> None:
        """Disarm, stop the sender thread, and close the serial port."""
        try:
            self.arm(False)
            self.send()
        except serial.SerialException:
            logger.exception("Failed to send final disarm frame on close")
        self.stop()
        if self.ser.is_open:
            self.ser.close()

    def __enter__(self) -> "Drone":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # -- control ---------------------------------------------------------

    def arm(self, armed: bool) -> None:
        with self._lock:
            self._channels[CH_AUX1] = CRSF_CHANNEL_VALUE_MAX if armed else CRSF_CHANNEL_VALUE_MIN

    def set_mode(self, angle: bool) -> None:
        with self._lock:
            self._channels[CH_AUX2] = CRSF_CHANNEL_VALUE_MAX if angle else CRSF_CHANNEL_VALUE_MIN

    def set_althold(self, enabled: bool) -> None:
        with self._lock:
            self._channels[CH_AUX3] = CRSF_CHANNEL_VALUE_MAX if enabled else CRSF_CHANNEL_VALUE_MIN

    def set_sticks(self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, throttle: float = 0.0) -> None:
        """Set stick positions. roll/pitch/yaw are -1.0..1.0, throttle is 0.0..1.0."""
        with self._lock:
            self._channels[CH_ROLL] = _bipolar_to_crsf(roll)
            self._channels[CH_PITCH] = _bipolar_to_crsf(pitch)
            self._channels[CH_YAW] = _bipolar_to_crsf(yaw)
            self._channels[CH_THROTTLE] = _unipolar_to_crsf(throttle)

    def send(self) -> None:
        """Immediately transmit the current channel state.

        Not required in normal use since the background thread already
        streams the current state at ``update_rate_hz``; call this if you
        want a value change reflected on the wire without waiting for the
        next scheduled tick.
        """
        with self._lock:
            frame = pack_rc_channels(self._channels)
            self._write(frame)

    # -- internals ---------------------------------------------------

    def _write(self, frame: bytes) -> None:
        try:
            self.ser.write(frame)
        except serial.SerialException:
            logger.exception("Failed to write CRSF frame to %s", self.ser.port)

    def _run(self) -> None:
        next_time = time.monotonic()
        while not self._stop_event.is_set():
            self.send()
            next_time += self._update_period
            delay = next_time - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                # We fell behind (e.g. a slow write); resync instead of
                # busy-looping to catch up.
                next_time = time.monotonic()
