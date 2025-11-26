"""
Example for controlling an ODrive via the CANSimple protocol with watchdog handling.

Puts the ODrive into closed loop control mode, sends a velocity setpoint of 1.0
and then prints the encoder feedback while periodically resending the velocity
command to reset the watchdog timer, ensuring continuous operation.

Assumes that the ODrive is already configured for velocity control and has
the watchdog enabled.

See https://docs.odriverobotics.com/v/latest/manual/can-protocol.html for protocol
documentation.
"""

import can
import time
import struct

node_id = 0  # must match `<odrv>.axis0.config.can.node_id`. The default is 0.

bus = can.interface.Bus("can0", bustype="socketcan")

# Flush CAN RX buffer so there are no more old pending messages
while bus.recv(timeout=0) is not None: pass

# Put axis into closed loop control state (this feeds the watchdog once)
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x07),  # 0x07: Set_Axis_State
    data=struct.pack('<I', 8),  # 8: AxisState.CLOSED_LOOP_CONTROL
    is_extended_id=False
))

# Wait for axis to enter closed loop control by scanning heartbeat messages
for msg in bus:
    if msg.arbitration_id == (node_id << 5 | 0x01):  # 0x01: Heartbeat
        error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        if state == 8:  # 8: AxisState.CLOSED_LOOP_CONTROL
            break

# Set velocity to 1.0 turns/s (this feeds the watchdog once)
velocity_setpoint = 1.0  # turns/s
torque_ff = 0.0 # torque feedforward
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x0d),  # 0x0d: Set_Input_Vel
    data=struct.pack('<ff', velocity_setpoint, torque_ff),
    is_extended_id=False
))

# Track last send time for periodic watchdog feeding
last_send_time = time.time()
feed_interval = 0.5

# Print encoder feedback with periodic watchdog feeding
for msg in bus:
    if msg.arbitration_id == (node_id << 5 | 0x09):  # 0x09: Get_Encoder_Estimates
        pos, vel = struct.unpack('<ff', bytes(msg.data))
        print(f"pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")
    
    # Periodically resend Set_Input_Vel to feed the watchdog
    current_time = time.time()
    if current_time - last_send_time >= feed_interval:
        bus.send(can.Message(
            arbitration_id=(node_id << 5 | 0x0d),  # 0x0d: Set_Input_Vel
            data=struct.pack('<ff', velocity_setpoint, torque_ff),
            is_extended_id=False
        ))
        last_send_time = current_time
