"""
Minimal example for controlling an ODrive via the CANSimple protocol.

Puts the ODrive into closed loop control mode, sends periodic sinusoidal velocity
setpoints, and asynchronously prints the encoder feedback.

Assumes that the ODrive is already configured for velocity control.

If the watchdog is enabled on the ODrive, it is fed implicitly by the continuous
velocity setpoint message and the motor will stop when the script is terminated.
The heartbeat interval should be shorter than the watchdog timeout to ensure
timely confirmation of the axis entering closed loop control mode without 
triggering the watchdog.

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

# Handler for incoming CAN messages to print encoder feedback
def on_rx_message(msg: can.Message):
    if msg.arbitration_id == (node_id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
        pos, vel = struct.unpack('<ff', bytes(msg.data))
        print(f"pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")

# Control ODrive while notifier object exist
with can.Notifier(bus, [on_rx_message]):
    while True:
        velocity_setpoint = math.sin(2 * math.pi * 0.5 * time.monotonic()) # turns / s

        # Update velocity and reset watchdog timer 
        bus.send(can.Message(
            arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
            data=struct.pack('<ff', velocity_setpoint, 0.0), # 0.0: torque feedforward
            is_extended_id=False
        ))
        time.sleep(0.1)