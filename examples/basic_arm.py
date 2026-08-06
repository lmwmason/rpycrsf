"""Arm, switch to angle mode, and hold a mid throttle for a few seconds.

AUX1 -> ARM
AUX2 -> MODE -> angle
AUX3 -> ALTHOLD

"""

import time
from rpycrsf import Drone

with Drone("/dev/serial0") as d:
    d.arm(True)
    d.set_mode(True)
    d.set_althold(False)
    d.set_sticks(throttle=0.5, roll=0.0, pitch=0.1, yaw=0.0)
    time.sleep(5)
