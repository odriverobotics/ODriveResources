
import argparse
import asyncio
import can
from dataclasses import dataclass
import json
import math
import struct
from can_simple_utils import CanSimpleNode, REBOOT_ACTION_SAVE # if this import fails, make sure you copy the whole folder from the git repository


_OPCODE_READ = 0x00
_OPCODE_WRITE = 0x01

# See https://docs.python.org/3/library/struct.html#format-characters
_FORMAT_LOOKUP = {
    'bool': '?',
    'uint8': 'B', 'int8': 'b',
    'uint16': 'H', 'int16': 'h',
    'uint32': 'I', 'int32': 'i',
    'uint64': 'Q', 'int64': 'q',
    'float': 'f'
}

_GET_VERSION_CMD = 0x00 # Get_Version
_RX_SDO = 0x04 # RxSdo
_TX_SDO = 0x05 # TxSdo


@dataclass
class EndpointAccess():
    node: CanSimpleNode
    endpoint_data: dict

    async def version_check(self):
        self.node.flush_rx()

        # Send read command
        self.node.bus.send(can.Message(
            arbitration_id=(self.node.node_id << 5 | _GET_VERSION_CMD),
            data=b'',
            is_extended_id=False
        ))

        # Await reply
        msg = await self.node.await_msg(_GET_VERSION_CMD)

        _, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', msg.data)
        hw_version_str = f"{hw_product_line}.{hw_version}.{hw_variant}"
        fw_version_str = f"{fw_major}.{fw_minor}.{fw_revision}"

        # If one of these asserts fail, you're probably not using the right flat_endpoints.json file
        if self.endpoint_data['fw_version'] != fw_version_str:
            raise Exception(f"the file provided in --endpoints-json does not match the firmware version of the ODrive: {self.endpoint_data['fw_version']} != {fw_version_str}")
        if self.endpoint_data['hw_version'] != hw_version_str:
            raise Exception(f"the file provided in --endpoints-json does not match the firmware version of the ODrive: {self.endpoint_data['hw_version']} != {hw_version_str}")

    async def write_and_verify(self, path: str, val):
        endpoint_id = self.endpoint_data['endpoints'][path]['id']
        endpoint_type = self.endpoint_data['endpoints'][path]['type']
        endpoint_fmt = _FORMAT_LOOKUP[endpoint_type]

        self.node.bus.send(can.Message(
            arbitration_id=(self.node.node_id << 5 | _RX_SDO),
            data=struct.pack('<BHB' + endpoint_fmt, _OPCODE_WRITE, endpoint_id, 0, val),
            is_extended_id=False
        ))

        # Since firmware 0.6.11, the device returns a confirmation for the write,
        # so we wait briefly to flush it before doing the read.
        await asyncio.sleep(0.01)
        self.node.flush_rx()

        self.node.bus.send(can.Message(
            arbitration_id=(self.node.node_id << 5 | _RX_SDO),
            data=struct.pack('<BHB', _OPCODE_READ, endpoint_id, 0),
            is_extended_id=False
        ))

        msg = await self.node.await_msg(_TX_SDO)

        # Unpack and cpmpare reply
        _, _, _, return_value = struct.unpack_from('<BHB' + endpoint_fmt, msg.data)
        val_pruned = val if endpoint_type != 'float' else struct.unpack('<f', struct.pack('<f', val))[0]
        matches = math.isnan(return_value) if math.isnan(val_pruned) else return_value == val_pruned
        if not matches:
            raise Exception(f"failed to write {path}: {return_value} != {val_pruned}")


async def restore_config(odrv: EndpointAccess, config: dict):
    print(f"writing {len(config)} variables...")
    for k, v in config.items():
        print(f"  {k} = {v}")
        await odrv.write_and_verify(k, v)

async def main():
    parser = argparse.ArgumentParser(description='Script to configure ODrive over CAN bus.')
    parser.add_argument('-i', '--interface', type=str, default='socketcan', help='Interface type (e.g., socketcan, slcan). Default is socketcan.')
    parser.add_argument('-c', '--channel', type=str, required=True, help='Channel/path/interface name of the device (e.g., can0, /dev/tty.usbmodem11201).')
    parser.add_argument('-b', '--bitrate', type=int, default=250000, help='Bitrate for CAN bus. Default is 250000.')
    parser.add_argument('--node-id', type=int, required=True, help='CAN Node ID of the ODrive.')
    parser.add_argument('--endpoints-json', type=str, required=True, help='Path to flat_endpoints.json corresponding to the given ODrive and firmware version.')
    parser.add_argument('--config', type=str, required=True, help='JSON file with configuration settings.')
    parser.add_argument("--save-config", action='store_true', help="Save the configuration to NVM and reboot ODrive.")
    args = parser.parse_args()

    with open(args.endpoints_json, 'r') as f:
        endpoint_data = json.load(f)

    with open(args.config, 'r') as f:
        config_list = json.load(f)

    print("opening CAN bus...")
    with can.interface.Bus(args.channel, bustype=args.interface, bitrate=args.bitrate) as bus:
        #reader = can.AsyncBufferedReader()
        #notifier = can.Notifier(bus, [reader], loop=asyncio.get_running_loop())
        with CanSimpleNode(bus=bus, node_id=args.node_id) as node:
            odrv = EndpointAccess(node=node, endpoint_data=endpoint_data)

            print("checking version...")
            await odrv.version_check()
            await restore_config(odrv, config_list)

            if args.save_config:
                print(f"saving configuration...")
                node.reboot_msg(REBOOT_ACTION_SAVE)

        await asyncio.sleep(0.1) # needed for last message to get through on SLCAN backend

if __name__ == "__main__":
    asyncio.run(main())
