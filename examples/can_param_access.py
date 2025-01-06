
# -- start load
import json
with open('flat_endpoints.json', 'r') as f:
    endpoint_data = json.load(f)
    endpoints = endpoint_data['endpoints']
# -- end load

# -- start definitions
OPCODE_READ = 0x00
OPCODE_WRITE = 0x01

# See https://docs.python.org/3/library/struct.html#format-characters
format_lookup = {
    'bool': '?',
    'uint8': 'B', 'int8': 'b',
    'uint16': 'H', 'int16': 'h',
    'uint32': 'I', 'int32': 'i',
    'uint64': 'Q', 'int64': 'q',
    'float': 'f'
}

node_id = 0 # must match the configured node_id on your ODrive (default 0)
# -- end definitions

import can
bus = can.interface.Bus("can0", bustype="socketcan")

# When using ODrive USB-CAN adapter:
# bus = can.interface.Bus(index=0, channel=0, bitrate=1000000, interface="gs_usb")

# Make sure CAN interface is closed when script exits
import atexit
bus.__enter__()
atexit.register(lambda: bus.__exit__(None, None, None))

# -- start version check
# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

# Send read command
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x00), # 0x00: Get_Version
    data=b'',
    is_extended_id=False
))

# Await reply
for msg in bus:
    if msg.is_rx and msg.arbitration_id == (node_id << 5 | 0x00): # 0x00: Get_Version
        break

import struct
_, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', msg.data)

# If one of these asserts fail, you're probably not using the right flat_endpoints.json file
assert endpoint_data['fw_version'] == f"{fw_major}.{fw_minor}.{fw_revision}"
assert endpoint_data['hw_version'] == f"{hw_product_line}.{hw_version}.{hw_variant}"
# -- end version check

# -- start write
import struct

path = 'axis0.controller.config.vel_integrator_limit'
value_to_write = 1.234

# Convert path to endpoint ID
endpoint_id = endpoints[path]['id']
endpoint_type = endpoints[path]['type']

# Send write command
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
    data=struct.pack('<BHB' + format_lookup[endpoint_type], OPCODE_WRITE, endpoint_id, 0, value_to_write),
    is_extended_id=False
))
# -- end write

# On firmware 0.6.11 or newer, the ODrive sends a confirmation for write
# requests, so we insert a small delay so that the response doesn't get confused
# for a response for the read request.
import time
time.sleep(0.1)

# -- start read
import struct

path = 'axis0.controller.config.vel_integrator_limit'

# Convert path to endpoint ID
endpoint_id = endpoints[path]['id']
endpoint_type = endpoints[path]['type']

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

# Send read command
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
    data=struct.pack('<BHB', OPCODE_READ, endpoint_id, 0),
    is_extended_id=False
))

# Await reply
for msg in bus:
    if msg.is_rx and msg.arbitration_id == (node_id << 5 | 0x05): # 0x05: TxSdo
        break

# Unpack and print reply
_, _, _, return_value = struct.unpack_from('<BHB' + format_lookup[endpoint_type], msg.data)
print(f"received: {return_value}")
# -- end read

# -- start function
import struct

path = "save_configuration"

# Convert path to endpoint ID
endpoint_id = endpoints[path]['id']

bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
    data=struct.pack('<BHB', OPCODE_WRITE, endpoint_id, 0),
    is_extended_id=False
))
# -- end function
