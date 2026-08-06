# rpycrsf

Python library to control CRSF (Crossfire/ELRS) based drones from a Raspberry Pi.

`rpycrsf` opens the Pi's UART, packs stick/switch positions into CRSF
`RC_CHANNELS_PACKED` frames, and streams them to the flight controller from
a background thread at a steady rate — so the link doesn't drop into
failsafe just because your script is busy doing something else.

## Install

```bash
pip install rpycrsf
```

Or from a checkout:

```bash
pip install -e .
```

## Raspberry Pi setup

1. Enable the hardware UART: `sudo raspi-config` → *Interface Options* →
   *Serial Port* → disable the login shell over serial, enable the
   hardware port.
2. On boards where Bluetooth uses the primary UART (Pi 3 / Pi 4 / Zero W),
   add `dtoverlay=disable-bt` to `/boot/firmware/config.txt` (or
   `/boot/config.txt`) and `sudo systemctl disable hciuart` so the PL011
   UART is freed up for `/dev/serial0` instead of being routed to
   Bluetooth via the slower mini-UART.
3. Add your user to the `dialout` group so you don't need `sudo` to open
   the port: `sudo usermod -aG dialout $USER` (log out/in to take effect).
4. Wire the CRSF TX module's UART RX/TX to the Pi's TX/GPIO14 and
   RX/GPIO15 pins (cross them: Pi TX → module RX, Pi RX → module TX), plus
   a common ground.
5. Use `/dev/serial0` as the device — it's a symlink to the correct UART
   for your board.

## Usage

```python
import time
from rpycrsf import Drone

with Drone("/dev/serial0") as d:
    d.arm(True)
    d.set_mode(True)  # AUX2 -> angle mode
    d.set_sticks(throttle=0.5, roll=0.0, pitch=0.1, yaw=0.0)
    time.sleep(5)
# `close()` (called automatically by the `with` block) disarms and stops
# the sender thread before the port closes.
```

`Drone` starts a background thread on construction that resends the
current channel state at `update_rate_hz` (150 Hz by default). Calling
`arm()`, `set_mode()`, or `set_sticks()` only updates in-memory state —
you don't need to call `send()` yourself in normal use; it exists for
cases where you want a change reflected on the wire immediately instead
of waiting for the next scheduled tick.

### API

- `Drone(device, baudrate=420000, update_rate_hz=150.0, auto_start=True)`
- `arm(armed: bool)` — AUX1
- `set_mode(angle: bool)` — AUX2
- `set_althold(enabled: bool)` — AUX3 (Betaflight `ALTHOLD` mode range)
- `set_sticks(roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0)` — roll/pitch/yaw
  are `-1.0..1.0`, throttle is `0.0..1.0`
- `send()` — force an immediate transmit
- `start()` / `stop()` — control the background sender thread
- `close()` — disarm, stop the thread, close the port (called by `__exit__`)

See [`examples/basic_arm.py`](examples/basic_arm.py) for a runnable example.

Lower level frame packing/CRC helpers and a raw `CRSFPort` (for sending
custom frame types or reading telemetry) are available under
`rpycrsf.protocol` if you need to go below the `Drone` API.

## Development

```bash
pip install -e ".[dev]"
pytest
```
