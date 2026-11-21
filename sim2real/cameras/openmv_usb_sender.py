#!/usr/bin/env python3
"""OpenMV H7 Plus USB VCP image collector -> CAR V11 camera endpoint.

Run on the Ubuntu machine connected to OpenMV over USB, or on Pi5 if OpenMV is
attached there. Device script openmv_camera_usb_main.py should be installed.
"""
import argparse
import os
import struct
import sys
import time
from pathlib import Path
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError


def read_token(user_value):
    if user_value:
        return user_value
    if os.environ.get('CAR_CAMERA_TOKEN'):
        return os.environ['CAR_CAMERA_TOKEN']
    file = Path(__file__).resolve().parents[1] / 'data/camera_token.txt'
    if file.exists():
        return file.read_text('ascii').strip()
    sys.exit('Camera token missing; use --token or export CAR_CAMERA_TOKEN')


def read_exact(port, count):
    data = bytearray()
    while len(data) < count:
        part = port.read(count-len(data))
        if not part:
            raise TimeoutError('OpenMV USB frame timed out')
        data.extend(part)
    return bytes(data)


def send(url, token, jpeg):
    request = Request(url, data=jpeg, headers={'Content-Type':'image/jpeg', 'X-Camera-Token':token},method='POST')
    with urlopen(request, timeout=5) as response:
        if response.status != 200:
            raise RuntimeError(f'Frame rejected: HTTP {response.status}')


def main():
    p=argparse.ArgumentParser(description='OpenMV JPEG USB VCP -> V11 CAR server')
    p.add_argument('--port',default='/dev/ttyACM0',help='USB VCP port (check ls /dev/ttyACM*)')
    p.add_argument('--server',default='http://127.0.0.1:8080',help='Ubuntu CAR HTTP server')
    p.add_argument('--token',default=None,help='Camera publish token; omit when running on CAR server host')
    p.add_argument('--fps',type=float,default=4.,help='Target frame rate, recommended 2..6 fps')
    a=p.parse_args()
    if not 0.5 <= a.fps <= 12: p.error('fps must be 0.5..12')
    try:
        import serial
    except ImportError:
        sys.exit('Missing pyserial. Ubuntu: sudo apt install python3-serial')
    token=read_token(a.token)
    url=a.server.rstrip('/')+'/api/camera/openmv/frame'
    print('Opening OpenMV USB port',a.port,'->',url,'(Ctrl+C to stop)',flush=True)
    while True:
        try:
            with serial.Serial(a.port, 115200, timeout=4, write_timeout=3) as port:
                # OpenMV official USB_VCP example uses DTR for its VCP transport.
                port.setDTR(True)
                port.reset_input_buffer()
                while True:
                    tick=time.monotonic()
                    port.write(b'snap')
                    n=struct.unpack('<I',read_exact(port,4))[0]
                    if not 64 <= n <= 2*1024*1024:
                        raise ValueError('Invalid frame size (%s). Is OpenMV main.py running?' % n)
                    jpeg=read_exact(port,n)
                    if not (jpeg.startswith(b'\xff\xd8') and jpeg.endswith(b'\xff\xd9')):
                        raise ValueError('OpenMV did not return a valid JPEG')
                    send(url,token,jpeg)
                    remaining=1/a.fps-(time.monotonic()-tick)
                    if remaining>0:time.sleep(remaining)
        except KeyboardInterrupt:
            print('\nStopped OpenMV camera.');return
        except (OSError,TimeoutError,ValueError,URLError,HTTPError) as exc:
            print('OpenMV connection lost:',exc,'- retrying in 2 s',flush=True)
            time.sleep(2)


if __name__=='__main__':main()
