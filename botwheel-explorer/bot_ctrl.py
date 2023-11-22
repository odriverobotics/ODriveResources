import argparse
import asyncio
import http.server
import json
import math
import os
import socket
import socketserver
import ssl
import struct
import threading
import time
import can
import qrcode
from websockets.exceptions import ConnectionClosed
from websockets.server import serve


# Connection Parameters

WEBSOCKET_PORT = 8080
HTTP_PORT = 8000

NODE_IDS = {
    'left': 0,
    'right': 1
}

ODRIVE_TIMEOUT = 1.0


# Behavior Parameters

DEFAULT_CONFIG = {
    'vel_gain': 3.0,
    'vel_integrator_gain': 25.0
}

LEFT_DIR = -1
RIGHT_DIR = 1

MAX_VEL = 1.0 # [m/s]
MAX_YAW = 0.5 # [turns/s]

BRAKE_TIMEOUT = 1.0
STATE_TRANSITION_TIMEOUT = 0.5


# Physical Parameters

WHEEL_DIAMETER = 171 # [mm]
WHEEL_SPACING = 419 # [mm]

VEL_COEF = 1000 / WHEEL_DIAMETER
YAW_COEF = WHEEL_SPACING / WHEEL_DIAMETER

AXIS_STATE_UNDEFINED = 0
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

class ODriveCAN():
    def __init__(self, bus, node_id):
        self.bus = bus
        self.node_id = node_id

        self.last_timestamp = 0.0

        # telemetry
        self.error = None
        self.state = AXIS_STATE_UNDEFINED
        self.vel = None
        self.dc_voltage = None
        self.dc_current = None
        self.torque_setpoint = None
        self.torque_estimate = None
        self.fet_temp = None
        self.motor_temp = None

    def set_gains(self, vel_gain, vel_integrator_gain):
        try:
            self.bus.send(can.Message(
                arbitration_id=(self.node_id << 5 | 0x01b), # 0x01b: Set_Vel_Gains
                data=struct.pack('<ff', vel_gain, vel_integrator_gain),
                is_extended_id=False
            ))
        except (OSError, can.exceptions.CanOperationError):
            pass # TX buffer might be full
    
    def request_state(self, state):
        try:
            self.bus.send(can.Message(
                arbitration_id=(self.node_id << 5 | 0x18), # 0x18: Clear_Errors
                data=b'',
                is_extended_id=False
            ))

            self.bus.send(can.Message(
                arbitration_id=(self.node_id << 5 | 0x07), # 0x07: Set_Axis_State
                data=struct.pack('<I', state), # 8: AxisState.CLOSED_LOOP_CONTROL
                is_extended_id=False
            ))
        except (OSError, can.exceptions.CanOperationError):
            pass # TX buffer might be full

    def send_vel(self, vel, torque_feedforward=0.0):
        try:
            self.bus.send(can.Message(
                arbitration_id=(self.node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
                data=struct.pack('<ff', vel, torque_feedforward),
                is_extended_id=False
            ))
        except (OSError, can.exceptions.CanOperationError):
            pass # TX buffer might be full

    def on_can_message(self, msg: can.Message):
        if msg.arbitration_id == (self.node_id << 5 | 0x01): # 0x01: Heartbeat
            self.error, self.state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            self.last_timestamp = time.monotonic()
        elif msg.arbitration_id == (self.node_id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
            pos, self.vel = struct.unpack('<ff', bytes(msg.data))
        elif msg.arbitration_id == (self.node_id << 5 | 0x15): # 0x15: Get_Temperature
            fet_temp, motor_temp = struct.unpack('<ff', bytes(msg.data))
            self.fet_temp = None if math.isnan(fet_temp) else fet_temp
            self.motor_temp = None if math.isnan(motor_temp) else motor_temp
        elif msg.arbitration_id == (self.node_id << 5 | 0x17): # 0x17: Get_Bus_Voltage_Current
            self.dc_voltage, self.dc_current = struct.unpack('<ff', bytes(msg.data))
        elif msg.arbitration_id == (self.node_id << 5 | 0x1C): # 0x1C: Get_Torques
            self.torque_setpoint, self.torque_estimate = struct.unpack('<ff', bytes(msg.data))

    def connected(self, now):
        return now - self.last_timestamp < ODRIVE_TIMEOUT


def user_space_to_axis_space(vel, yaw):
    vel_left = VEL_COEF * vel + YAW_COEF * yaw
    vel_right = VEL_COEF * vel - YAW_COEF * yaw
    return (vel_left * LEFT_DIR, vel_right * RIGHT_DIR)

def axis_space_to_user_space(vel_left, vel_right):
    vel_left *= LEFT_DIR
    vel_right *= RIGHT_DIR
    vel = (vel_left + vel_right) / 2 / VEL_COEF
    yaw = (vel_left - vel_right) / 2 / YAW_COEF
    return (vel, yaw)

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


class ODriveCart():
    def __init__(self, bus, ignore_odrives):
        self.odrives = {k: ODriveCAN(bus, v) for k, v in NODE_IDS.items()}
        can.Notifier(bus, [odrv.on_can_message for odrv in self.odrives.values()])

        self.ignore_odrives = ignore_odrives
        self.enabled_odrives = [odrv for k, odrv in self.odrives.items() if not k in ignore_odrives]

        self.user_commands = {} # key: connection identifier, value: dict {vel, yaw}
        self.user_requested_state = None

        self.state = 'waiting-for-odrives'

        # Load gain settings from persistent config
        if os.path.isfile('config.json'):
            with open('config.json', 'r') as fp:
                self.config = {**DEFAULT_CONFIG, **json.load(fp)}
        else:
            self.config = {**DEFAULT_CONFIG}

    def set_config(self, config: dict):
        new_config = {**self.config, **config}
        if self.config != new_config:
            self.config = new_config
            with open('config.json', 'w') as fp:
                json.dump(self.config, fp)

    @property
    def telemetry(self):
        vel, yaw = axis_space_to_user_space(
            self.odrives['left'].vel or 0.0,
            self.odrives['right'].vel or 0.0
        )
        return {
            'vel': vel,
            'yaw': yaw,
            'state': self.state,
            'odrives': {
                k: {
                    'error': v.error,
                    'state': v.state,
                    'vel': v.vel,
                    'dc_voltage': v.dc_voltage,
                    'dc_current': v.dc_current,
                    'torque_setpoint': v.torque_setpoint,
                    'torque_estimate': v.torque_estimate,
                    'fet_temp': v.fet_temp,
                    'motor_temp': v.motor_temp,
                } for k, v in self.odrives.items()
            },
            'config': self.config
        }

    async def main_loop(self):
        while True:
            now = time.monotonic()
            
            # Implicitly request braking state when all clients are disconnected
            if len(self.user_commands) == 0:
                self.user_requested_state = 'brake'

            # State transitions based on user requests and timing
            state = self.state

            if state != 'drive' and state != 'entering-drive' and self.user_requested_state == 'drive':
                state = 'entering-drive'
            elif state != 'coast' and state != 'entering-coast' and self.user_requested_state == 'coast':
                state = 'entering-coast'
            elif state == 'drive' and self.user_requested_state == 'brake':
                state = 'brake'
            elif state == 'entering-drive' and self.user_requested_state == 'brake':
                state = 'entering-coast'
            elif state == 'brake' and now - self.state_timestamp > BRAKE_TIMEOUT:
                state = 'entering-coast' # brake command times out after 1 second
            elif state == 'entering-drive' and now - self.state_timestamp > STATE_TRANSITION_TIMEOUT:
                state = 'entering-coast' # entering-drive state times out after 1 second
            
            self.user_requested_state = None

            # State transitions based on ODrive feedback

            odrive_states = set(odrv.state for odrv in self.enabled_odrives)
            common_odrive_state = next(iter(odrive_states)) if len(odrive_states) == 1 else None
            if any(not odrv.connected(now) for odrv in self.enabled_odrives):
                state = 'waiting-for-odrives'
            elif state == 'waiting-for-odrives':
                state = 'coast'
            elif state == 'entering-drive' and common_odrive_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                state = 'drive'
            elif state == 'entering-coast' and common_odrive_state == AXIS_STATE_IDLE:
                state = 'coast'
            elif (state == 'drive' or state == 'brake') and common_odrive_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                # In case of an error, one ODrive might disarm. In that case we immediately disarm the other one too
                state = 'entering-coast'

            if self.state != state:
                self.state = state
                self.state_timestamp = now
                print("state: " + state)

            # During the transition states, send periodic requests to each ODrive until confirmed
            if state == 'entering-coast' or state == 'entering-drive':
                target_state = AXIS_STATE_CLOSED_LOOP_CONTROL if state == 'entering-drive' else AXIS_STATE_IDLE
                for odrv in self.enabled_odrives:
                    if odrv.connected(now) and odrv.state != target_state:
                        odrv.request_state(target_state)

            # Calculate velocity setpoints and send to ODrives
            if state == 'drive':
                total_vel = clamp(sum(cmd['vel'] for cmd in self.user_commands.values()), -MAX_VEL, MAX_VEL)
                total_yaw = clamp(sum(cmd['yaw'] for cmd in self.user_commands.values()), -MAX_YAW, MAX_YAW)
            else:
                total_vel = 0.0
                total_yaw = 0.0

            vel_left, vel_right = user_space_to_axis_space(total_vel, total_yaw)

            self.odrives['left'].send_vel(vel_left)
            self.odrives['right'].send_vel(vel_right)

            for odrv in self.enabled_odrives:
                odrv.set_gains(self.config['vel_gain'], self.config['vel_integrator_gain'])

            # Wait for 100ms (10Hz)
            await asyncio.sleep(0.1)


async def client_tx(websocket, cart: ODriveCart):
    try:
        while True:
            #print(latest_telemetry)
            await websocket.send(json.dumps(cart.telemetry))
            await asyncio.sleep(0.1)
    except ConnectionClosed:
        pass

async def client_rx(websocket, cart):
    try:
        while True:
            # Receive message from client
            message = await websocket.recv()
            data = json.loads(message)
            cart.set_config(data.get('config', {}))
            # Store the most recent vel/yaw command
            if 'vel' in data and 'yaw' in data:
                cart.user_commands[websocket] = {'vel': data['vel'], 'yaw': data['yaw']}
            new_state = data.get('state', None)
            if new_state:
                cart.user_requested_state = new_state
                print("client requested " + new_state)
    except ConnectionClosed:
        print("client disconnected")
    finally:
        cart.user_commands.pop(websocket, None)

# Method using socket.getattrinfo()
def get_local_ips_1():
    try:
        hostname = socket.gethostname()
        if not '.' in hostname:
            hostname = hostname + '.local' # without this suffix, sometimes only 127.0.0.1 is returned
        local_ip_addresses = set(ip[4][0] for ip in socket.getaddrinfo(hostname, None))
    except Exception as e:
        print(f"Could not determine IP addresses: {e}")
        return []
    return sorted(local_ip_addresses, key=lambda x: (':' in x, x)) # prioritize IPv4 addresses

# # Method using fib_tree
# def get_local_ips_2():
#     local_ips = {} # key: string, value: priority (lowest first)
#     last_ip = None
#     with open("/proc/net/fib_trie", "r") as f:
#         for line in f:
#             if line.strip().startswith("|-- "):
#                 last_ip = line.strip().split()[1]
#             elif line.strip().startswith("+-- "):
#                 last_ip = None
#             elif last_ip and "/32 host LOCAL" in line:
#                 priority = len(line.lstrip()) - len(line)
#                 local_ips[last_ip] = min(local_ips.get(last_ip, priority), priority)
#     return sorted(set(local_ips.keys()) - {'127.0.0.1'}, key=lambda x: (local_ips[x], x))
# 
# # Method using ip route
# import subprocess
# def get_local_ips_3():
#     local_ip_addresses = []
#     try:
#         result = subprocess.run(['ip', 'route', 'get', '1.1.1.1'], stdout=subprocess.PIPE).stdout.decode('utf-8')
#         # Extract the source IP address from the command output
#         for line in result.splitlines():
#             if 'src' in line:
#                 primary_ip = line.split('src')[-1].strip().split()[0]
#                 local_ip_addresses.add(primary_ip)
#                 break
#     except Exception as e:
#         print(f"Could not determine non-loopback IP address: {e}")
#         return []
#     return sorted(local_ip_addresses, reverse=True) # prioritize higher value addresses


def print_connection_hints(local_ip_addresses, ssl: bool):
    if len(local_ip_addresses) == 0:
        print("No IP address found")

    else:
        print("")
        print("##################################################################")
        print("Scan the QR code below or copy-paste the address into your browser")

        url = f"{'https' if ssl else 'http'}://{local_ip_addresses[0]}:{HTTP_PORT}/bot_ctrl.html#autoconnect"

        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_L,
            box_size=10,
            border=4,
        )
        qr.add_data(url)
        qr.make(fit=True)
        qr.print_ascii()

        print(url)
        print()
        if len(local_ip_addresses) > 1:
            print("Alternative IP addresses if the URL above doesn't work:")
            for ip_addr in local_ip_addresses[1:]:
                print(f"* {ip_addr}")
            print()
        print("##################################################################")
        print()

class MyRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/bot_ctrl.html':
            super().do_GET()
        else:
            self.send_response(404)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(b'File not found')

def run_http_server(httpd: socketserver.TCPServer):
    print(f"Serving on port {HTTP_PORT}")
    httpd.allow_reuse_address = True
    httpd.serve_forever()


async def main():
    parser = argparse.ArgumentParser(description='BotWheel Explorer Control Script\n')
    parser.add_argument('--can', type=str, default='can0')
    parser.add_argument("-s", "--ssl", action="store_true",
                        help="Serve the remote control page and websocket via SSL using a self-signed certificate.")
    parser.add_argument('--ignore', type=str, nargs='+',
                        help=f'Disable the specified ODrives. Allowed values: {NODE_IDS.keys()}. '
                        "Useful for testing when one or more ODrives are disconnected. If ignored ODrives are present, "
                        "their telemetry will still be relayed but they will be set to IDLE.")
    args = parser.parse_args()

    os.chdir(os.path.dirname(os.path.realpath(__file__)))

    local_ip_addresses = get_local_ips_1()
    bus = can.interface.Bus(args.can, bustype="socketcan")

    # Flush CAN RX buffer so there are no more old pending messages
    while not (bus.recv(timeout=0) is None): pass
    
    cart = ODriveCart(bus, set(args.ignore or {}))

    if args.ssl:
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.load_cert_chain(certfile='cert.pem', keyfile='key.pem')
    else:
        ssl_context = None

    # Launch HTTP server
    with socketserver.TCPServer(("", HTTP_PORT), MyRequestHandler) as httpd:
        if not ssl_context is None:
            httpd.socket = ssl_context.wrap_socket(httpd.socket)
        threading.Thread(target=run_http_server, args=(httpd,)).start()

        try:
            # Launch websocket server
            async def handle_client(websocket, path):
                print("client connected")
                await asyncio.gather(
                    asyncio.create_task(client_tx(websocket, cart)),
                    asyncio.create_task(client_rx(websocket, cart))
                )

            async with serve(handle_client, '0.0.0.0', WEBSOCKET_PORT, ssl=ssl_context, ping_timeout=2.5, ping_interval=1):
                print_connection_hints(local_ip_addresses, args.ssl)

                # Launch main loop
                await cart.main_loop()

        finally:
            httpd.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
