
import argparse
import asyncio
import can
from dataclasses import dataclass
import json
import struct
from can_simple_utils import CanSimpleNode, REBOOT_ACTION_SAVE # if this import fails, make sure you copy the whole folder from the git repository

async def main():
    parser = argparse.ArgumentParser(description='Script to calibrate the ODrive over CAN bus.')
    parser.add_argument('-i', '--interface', type=str, default='socketcan', help='Interface type (e.g., socketcan, slcan). Default is socketcan.')
    parser.add_argument('-c', '--channel', type=str, required=True, help='Channel/path/interface name of the device (e.g., can0, /dev/tty.usbmodem11201).')
    parser.add_argument('-b', '--bitrate', type=int, default=250000, help='Bitrate for CAN bus. Default is 250000.')
    parser.add_argument('--state', type=int, default=3, help='Which calibration state to run. Defaults to AxisState.FULL_CALIBRATION_SEQUENCE.')
    parser.add_argument('--node-id', type=int, required=True, help='CAN Node ID of the ODrive.')
    parser.add_argument("--save-config", action='store_true', help="Save the configuration to NVM and reboot ODrive.")
    args = parser.parse_args()

    print("opening CAN bus...")
    with can.interface.Bus(args.channel, bustype=args.interface, bitrate=args.bitrate) as bus:
        with CanSimpleNode(bus=bus, node_id=args.node_id) as node:
            node.clear_errors_msg()
            node.set_state_msg(3) # AxisState.FULL_CALIBRATION_SEQUENCE
            print("waiting for calibration to finish...")
            await asyncio.sleep(1.0)
            node.flush_rx()
            while True:
                msg = await node.await_msg(0x01)
                error, state, procedure_result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if state == 1: # 1: AxisState.IDLE
                    break
            if error != 0 or procedure_result != 0:
                raise Exception(f"Calibration resulted in {error=}, {procedure_result=}. See here for error codes:\n"
                    "- https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Error\n"
                    "- https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.ProcedureResult"
                )
            
            print("Calibration successful!")

            if args.save_config:
                print(f"saving configuration...")
                node.reboot_msg(REBOOT_ACTION_SAVE)

        await asyncio.sleep(0.1) # needed for last message to get through on SLCAN backend

if __name__ == "__main__":
    asyncio.run(main())
