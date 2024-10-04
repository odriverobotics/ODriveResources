#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""
import math
import time

import odrive
from odrive.enums import AxisState, ControlMode, InputMode
from odrive.utils import dump_errors, request_state

# Find a connected ODrive (this will block until you connect one)
print("waiting for ODrive...")
odrv0 = odrive.find_sync()
print(f"found ODrive {odrv0._dev.serial_number}")

# Enter closed loop control
odrv0.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
odrv0.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
request_state(odrv0.axis0, AxisState.CLOSED_LOOP_CONTROL)

# Run a sine wave until ODrive reports an error or user hits Ctrl+C
try:
    p0 = odrv0.axis0.controller.input_pos
    t0 = time.monotonic()
    while odrv0.axis0.current_state == AxisState.CLOSED_LOOP_CONTROL:
        setpoint = p0 + 4.0 * math.sin((time.monotonic() - t0) * 2)
        print(f"goto {setpoint}")
        odrv0.axis0.controller.input_pos = setpoint
        time.sleep(0.01)

finally:
    request_state(odrv0.axis0, AxisState.IDLE)

# Show errors
dump_errors(odrv0)




# Some more things you can try:

# Write to a read-only property:
odrv0.vbus_voltage = 11.0  # fails with `AttributeError: This property cannot be written to.`

# Assign an incompatible value:
odrv0.axis0.controller.input_pos = "I like trains"  # fails with `ValueError: could not convert string to float`
