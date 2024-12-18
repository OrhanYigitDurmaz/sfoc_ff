import sys
import subprocess
import struct
import socket
import time

serial_port = r'usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_207238674D4D-if00'


def check_can_if_state(can_if):
    result = subprocess.run(["ifconfig", can_if],
                            capture_output=True,
                            text=True)
    print(result.stdout)
    has_error = 'Device not found' in result.stderr
    # if has_error:
        # subprocess.run(['sudo', 'slcand', '-o', '-c', '-s0', serial_port, 'can0'])
    print(f'{can_if} has error: {has_error}')
    is_up = 'UP,RUNNING' in result.stdout
    print(f'{can_if} has is up and running: {is_up}')
    if is_up:
        return True


class VESC():
    PING = 17
    SET_CURRENT_REL = 10
    POLL_ROTOR_POS = 56
    PROCESS_SHORT_BUFFER = 8

    def __init__(self, can_if, can_address=0xFF):
        self.address = can_address
        self.interface = can_if

    def ping(self):
        self.send(self.PING, '')

    def set_current(self, current):
        current = int(float(current)*1e5)
        data = struct.pack('i', current)
        data_str = ''.join([f'{b:02x}' for b in data])
        self.send(self.SET_CURRENT_REL, data_str)

    def get_position(self):
        self.send(self.POLL_ROTOR_POS, '')

    def get_fw_version(self):
        data = '400000'
        self.send(self.PROCESS_SHORT_BUFFER, data)

    def get_telem(self):
        data = '40' + '00' + '32'
        cmd = struct.pack('i', 1 << 8 | 1 << 15)
        cmd_str = ''.join([f'{b:02x}' for b in cmd])
        data = data + cmd_str
        self.send(self.PROCESS_SHORT_BUFFER, data)

    def send(self, command_id, data):
        addr = f'{(int(command_id)<<8 | self.address):08x}'

        subprocess.run(['cansend', self.interface, addr+'#'+str(data)])


is_up = check_can_if_state("can0")
if not is_up:
    print('can0 not up')
    sys.exit(1)

s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
interface = "can0"  # Replace with your CAN interface name
s.bind((interface,))
s.settimeout(1)

vesc = VESC(interface)
vesc.get_fw_version()
vesc.ping()
vesc.set_current(1.23)
vesc.ping()

vesc.get_position()
time.sleep(0.1)
vesc.get_telem()
vesc.get_telem()
time.sleep(0.5)

t0 = time.monotonic()
while True:
    try:
        cf, addr = s.recvfrom(16,)
    except Exception as e:
        print(e)
        break
    # print(cf)
# Unpack the CAN frame
    can_id, cmd, can_dlc, data = struct.unpack_from("BBxxBBxx", cf)
    if cmd == 0x38:
        data = float(struct.unpack('i', cf[8:12])[0])/1e6
    elif cmd == 0x8:
        buffer = cf[8:]
        print([f'0x{b:02x} ' for b in buffer])
        # print([f'{int(b)} ' for b in cf])
        sub_cmd = buffer[2]
        data = ''
        if sub_cmd == 0:
            print(f'FW version: 0x{buffer[3]:02x}:0x{buffer[4]:02x}')
        elif sub_cmd == 50:
            pass
            # mask = int(struct.unpack('i', cf[11:15])[0])
            # print(f'flags: voltage: {mask&(1<<8)==(1<<8)} - status: {mask&(1<<15)==(1<<15)}')
            # if mask & 1 << 8:
            #     volts = struct.unpack('h', cf[15:18])[0]
            #     data += f'Volts: {volts/10.0:.3f}'
            # if mask & 1 << 15:
            #     status = cf[18]
            #     data += ' ' + f'status: {status}'

    else:
        data = cf[8:]
# Print the received message
    print(f"CAN ID: {can_id:02x}, cmd: {cmd:02x}, DLC: {can_dlc}, Data: {data}\n")
    if time.monotonic() > (t0+1):
        break
vesc.set_current(0)
