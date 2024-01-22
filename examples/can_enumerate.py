"""
Interactive ODrive CAN identification script

Run `can_enumerate.py --help` for usage information.

See also:
- General docs: https://docs.odriverobotics.com/v/latest/guides/can-guide.html#setting-up-the-odrive-can-only
- Protocol specification: https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#discovery-addressing
"""

import argparse
import asyncio
import sys
import time
from typing import Dict, List

import can

ADDRESS_CMD = 0x06
REBOOT_CMD = 0x16
CLEAR_ERRORS_CMD = 0x18

BROADCAST_NODE_ID = 0x3f
MAX_NODE_ID = 0x3e

DISCOVERY_MESSAGE_INTERVAL = 0.6
TIMEOUT = 3.0

REBOOT_ACTION_REBOOT = 0
REBOOT_ACTION_SAVE = 1
REBOOT_ACTION_ERASE = 2

def sn_str(sn):
    return f"{sn:012X}"

def get_address_msg(bus: can.Bus):
    msg = can.Message(
        arbitration_id=(BROADCAST_NODE_ID << 5) | ADDRESS_CMD,
        is_extended_id=False,
        is_remote_frame=True
    )
    bus.send(msg)

def set_address_msg(bus, sn, node_id):
    msg = can.Message(
        arbitration_id=(BROADCAST_NODE_ID << 5) | ADDRESS_CMD,
        data=bytes([node_id]) + sn.to_bytes(6, byteorder='little'),
        is_extended_id=False
    )
    bus.send(msg)

def identify_msg(bus: can.Bus, node_id: int, enable: bool):
    msg = can.Message(
        arbitration_id=(node_id << 5) | CLEAR_ERRORS_CMD,
        data=b'\x01' if enable else b'\x00',
        is_extended_id=False
    )
    bus.send(msg)

def reboot_msg(bus: can.Bus, node_id: int, action: int):
    msg = can.Message(
        arbitration_id=(node_id << 5) | REBOOT_CMD,
        data=[action],
        is_extended_id=False
    )
    bus.send(msg)

class Discoverer():
    def __init__(self, bus):
        self.bus = bus
        self.last_received_time = time.monotonic()
        self.discovered_devices = {}  # serial_number: node_id
        self.auto_assign = False

    def __enter__(self):
        self.notifier = can.Notifier(self.bus, [self.on_message_received], loop=asyncio.get_running_loop())
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.notifier.stop()
        pass

    def assign_free_node_id(self, serial_number):
        next_free_node_id = next(i for i in range(len(self.discovered_devices) + 1) if i not in self.discovered_devices.values())
        if next_free_node_id > MAX_NODE_ID:
            print(f"Can't address {sn_str(serial_number)} because there are too many devices on the bus.")

        print(f"Assigning node ID {next_free_node_id} to {sn_str(serial_number)}")

        set_address_msg(self.bus, serial_number, next_free_node_id)
        self.discovered_devices[serial_number] = next_free_node_id

    def on_message_received(self, msg):
        cmd_id = msg.arbitration_id & 0x1F

        if cmd_id == ADDRESS_CMD:
            node_id = msg.data[0]
            serial_number = int.from_bytes(msg.data[1:7], byteorder='little')

            if serial_number not in self.discovered_devices:
                node_id_str = "unaddressed" if node_id == BROADCAST_NODE_ID else f"node ID {node_id}"
                print(f"Discovered ODrive {sn_str(serial_number)} ({node_id_str})")
            #else:
            #    node_id_str = "unaddressed" if node_id == BROADCAST_NODE_ID else f"node ID {node_id}"
            #    print(f"Rediscovered ODrive {sn_str(serial_number)} ({node_id_str})")
            
            self.discovered_devices[serial_number] = node_id if node_id != BROADCAST_NODE_ID else None

            if self.discovered_devices[serial_number] is None:
                self.last_received_time = time.monotonic()

                if self.auto_assign:
                    self.assign_free_node_id(serial_number)

async def scan_for_devices(bus):
    """
    Scans for ODrives on the bus and assigns addresses for all ODrives that are
    not addressed yet. This should discover all ODrives, regardless of their
    configured node ID.
    """
    
    print(f"Scanning for ODrives...")

    with Discoverer(bus) as discoverer:
        iteration_count = 0
        while True:
            iteration_count += 1

            get_address_msg(bus)

            await asyncio.sleep(DISCOVERY_MESSAGE_INTERVAL)

            if iteration_count == 3:
                for serial, node_id in discoverer.discovered_devices.items():
                    if node_id is None:
                        discoverer.assign_free_node_id(serial)
                discoverer.auto_assign = True

            # Check exit condition
            current_time = time.monotonic()
            if current_time - discoverer.last_received_time >= TIMEOUT:
                break

    print(f"Scan complete. Discovered {len(discoverer.discovered_devices)} ODrives.\n")
    return discoverer.discovered_devices

def identify_ui(bus: can.Bus, node_ids: List[int], user_labels: List[str]) -> Dict[str, int]:
    """
    Blinks the LEDs of the specified ODrives one by one and shows interactive
    user prompts to determine which node_id belongs to which user label.

    Parameters
    ----------
    node_ids: A list of node_ids that should be identified.
    user_labels: A list of strings representing descriptive user labels. For
        example "right wheel" or "left shoulder".

    Returns
    -------
    A list (node_id: user_label). Depending on the user responses, some node_ids
    or user_labels may be missing from this map.
    """
    node_to_label = {}

    # Stop blinking all LEDs
    identify_msg(bus, BROADCAST_NODE_ID, False)

    first = True

    node_ids = [node_id for node_id in node_ids if not node_id is None]

    for node_id in node_ids:
        if len(user_labels) == 0:
            break # no more choices

        identify_msg(bus, node_id, True)

        try:
            # Show user prompt
            print(f'{"One of the ODrives" if first else "Another ODrive"} should have a white flashing LED now. Which one is flashing?')
            for i, label in enumerate(user_labels):
                print(f"  {i}: {label}")
            print("  n: none/other/multiple")

            while True:    
                user_response = input("Enter a number from the list above: ")

                if user_response.lower() in ['n', 'none']:
                    num = None
                    break
                else:
                    try:
                        num = int(user_response)
                    except ValueError:
                        pass # not a number
                    if num >= 0 and num < len(user_labels):
                        break

                print("Invalid input")

            if num is None:
                print("Hmm strange.")
            else:
                node_to_label[node_id] = user_labels.pop(num)
        finally:
            identify_msg(bus, node_id, False)

        first = False

    unidentified_node_ids = [node_id for node_id in node_ids if not node_id in node_to_label]
    found_all = len(unidentified_node_ids) == 0 and len(user_labels) == 0
    if not found_all:
        print()
    if len(unidentified_node_ids):
        print(f"Some unidentified ODrives were found: node_ids {unidentified_node_ids}")
    if len(user_labels):
        print(f"Some ODrives were not found: {user_labels}")

    return found_all, node_to_label


async def set_addresses(bus: can.Bus, sn_to_node_id: Dict[int, int]):
    """
    Assigns the specified node IDs to the specified ODrives identified by serial
    number. This works regardless of the previous node ID of the corresponding
    ODrives.
    If other ODrives already occupy the target node IDs, they will vacate those
    node IDs and go into unaddressed state.

    Parameters
    ----------
    sn_to_node_id: dict of the form {serial_number: node_id}
    """
    for sn, node_id in sn_to_node_id:
        set_address_msg(bus, sn, node_id)


async def main():
    parser = argparse.ArgumentParser(description="Interactive ODrive identification script.")
    parser.add_argument('-i', '--interface', type=str, default='socketcan', help='Interface type (e.g., socketcan, slcan). Default is socketcan.')
    parser.add_argument('-c', '--channel', type=str, required=True, help='Channel/path/interface name of the device (e.g., can0, /dev/tty.usbmodem11201).')
    parser.add_argument('-b', '--bitrate', type=int, default=250000, help='Bitrate for CAN bus. Default is 250000.')
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--reboot-all", action='store_true',
                    help="Broadcast reboot command to all ODrives before running discovery. "
                         "This is not required but can be used for increased determinism or testing repeatability.")
    group.add_argument("--erase-all", action='store_true',
                    help="Broadcast erase command to all ODrives before running discovery. "
                         "This is not required but can be used for increased determinism or testing repeatability.")
    parser.add_argument("labels", metavar="L", type=str, nargs="*", help="Pairs of user_label=node_id (e.g., 'left=0' 'right=1')")
    parser.add_argument("--save-config", action='store_true',
            help="For every ODrive for which the user specified a node ID, save the configuration to NVM.")
    args = parser.parse_args()

    # Parse "labels" list
    user_labels = []
    user_addresses = {} # label: address

    for item in args.labels:
        parts = item.split('=')
        user_labels.append(parts[0])
        if len(parts) >= 2:
            try:
                if len(parts) > 2:
                    raise ValueError()
                num = int(parts[1])
            except ValueError:
                print(f"{item} is not a valid item")
                sys.exit(1)
            user_addresses[parts[0]] = num

    with can.interface.Bus(args.channel, bustype=args.interface, bitrate=args.bitrate) as bus:
        if args.reboot_all:
            print("Broadcasting reboot command...")
            reboot_msg(bus, BROADCAST_NODE_ID, REBOOT_ACTION_REBOOT)

        if args.erase_all:
            print("Broadcasting erase command...")
            reboot_msg(bus, BROADCAST_NODE_ID, REBOOT_ACTION_ERASE)
        
        # Run discovery
        discovered_devices = await scan_for_devices(bus)

        if len(user_labels):
            ok, node_to_label = identify_ui(bus, discovered_devices.values(), user_labels)

            sn_to_label = [
                (sn, node_to_label[node_id])
                for sn, node_id in discovered_devices.items()
                if node_id in node_to_label
            ]
            sn_to_target_addr = [
                (sn, user_addresses[label])
                for sn, label in sn_to_label
                if (label in user_addresses)
            ]

            # Filter out the nodes that already are on the target address
            sn_to_new_addr = [(sn, node_id) for sn, node_id in sn_to_target_addr if node_id != discovered_devices[sn]]

            if len(sn_to_new_addr):
                print(f"Assigning new node IDs to {len(sn_to_new_addr)} ODrives")
                await set_addresses(bus, sn_to_new_addr)

            if len(sn_to_target_addr) and args.save_config:
                print(f"Saving configuration on {len(sn_to_target_addr)} ODrives")
                for sn, node_id in sn_to_target_addr:
                    reboot_msg(bus, node_id, REBOOT_ACTION_SAVE)
        else:
            ok = True

        await asyncio.sleep(0.1) # needed for last message to get through on SLCAN backend

    sys.exit(0 if ok else 1)

if __name__ == '__main__':
    asyncio.run(main())

