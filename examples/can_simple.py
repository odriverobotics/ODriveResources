"""
Example for controlling an ODrive via the CANSimple protocol with watchdog handling.

Puts the ODrive into closed loop control mode, sends periodic sinusoidal velocity
setpoints, and asynchronously prints the encoder feedback. The regular velocity
setpoint update resets the watchdog, which is a safety feature that disengages
the axis if no valid commands are received within the configured timeout period,
avoiding uncontrolled behavior.

Assumes that the ODrive is already configured for velocity control and has
the watchdog enabled. Additionally, assumes that the heartbeat message rate is
shorter than the watchdog timeout to ensure timely confirmation of the axis
entering closed loop control mode without triggering the watchdog.

See https://docs.odriverobotics.com/v/latest/manual/can-protocol.html for protocol
documentation.
"""

import can
import math
import time
import struct

node_id = 0 # must match `<odrv>.axis0.config.can.node_id`. The default is 0.

bus = can.interface.Bus("can0", interface="socketcan")

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

# Put axis into closed loop control state
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
    data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
    is_extended_id=False
))

# Wait for axis to enter closed loop control by scanning heartbeat messages
for msg in bus:
    if msg.arbitration_id == (node_id << 5 | 0x01): # 0x01: Heartbeat
        error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        if state == 8: # 8: AxisState.CLOSED_LOOP_CONTROL
            break

# Define an asynchronous listener to handle and print encoder feedback
def encoder_listener(msg):
    if msg.arbitration_id == (node_id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
        pos, vel = struct.unpack('<ff', bytes(msg.data))
        print(f"pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")

# Control ODrive while notifier object exist
with can.Notifier(bus, [encoder_listener]):
    f = 0.5 # Hz
    torque_ff = 0.0 # torque feedforward

    while True:
        t = time.time()
        velocity_setpoint = math.sin(2 * math.pi * f * t) # turns / s

        # Update velocity and reset watchdog timer 
        bus.send(can.Message(
            arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
            data=struct.pack('<ff', velocity_setpoint, torque_ff),
            is_extended_id=False
        ))
        time.sleep(0.1)
