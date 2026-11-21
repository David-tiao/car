#!/usr/bin/env python3
"""Raspberry Pi 5 CSI camera (any supported 77-degree lens) -> CAR V11.

Run on Raspberry Pi OS with python3-picamera2 installed (usually preinstalled).
Publishes JPEG to the Ubuntu PC's HTTP server using a camera upload token.
"""
import argparse
import io
import os
import sys
import time
from pathlib import Path
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError


def main():
    p=argparse.ArgumentParser(description='Raspberry Pi 5 CSI camera -> CAR HTTP server')
    p.add_argument('--server',required=True,help='Ubuntu address, e.g. http://10.50.88.249:8080')
    p.add_argument('--token',default=os.environ.get('CAR_CAMERA_TOKEN',''),help='Camera token from Ubuntu data/camera_token.txt')
    p.add_argument('--fps',type=float,default=6.,help='Target fps, recommended 4..10')
    p.add_argument('--width',type=int,default=640)
    p.add_argument('--height',type=int,default=480)
    p.add_argument('--camera',type=int,default=0,help='Picamera2 camera index')
    a=p.parse_args()
    if not a.token:p.error('Missing camera token (--token or CAR_CAMERA_TOKEN env variable)')
    if not 0.5<=a.fps<=20:p.error('fps must be 0.5..20')
    if not (160<=a.width<=1920 and 120<=a.height<=1080):p.error('width or height outside supported preview range')
    try:
        from picamera2 import Picamera2
    except ImportError:
        sys.exit('Picamera2 not installed. Raspberry Pi OS: sudo apt install python3-picamera2')
    url=a.server.rstrip('/')+'/api/camera/pi5/frame'
    camera=Picamera2(a.camera)
    camera.configure(camera.create_video_configuration(main={'size':(a.width,a.height)}))
    camera.start()
    time.sleep(.7)
    print('Pi5 CSI camera online ->',url,'(Ctrl+C to stop)',flush=True)
    try:
        while True:
            tick=time.monotonic()
            out=io.BytesIO()
            camera.capture_file(out,format='jpeg')
            jpeg=out.getvalue()
            req=Request(url,data=jpeg,headers={'Content-Type':'image/jpeg','X-Camera-Token':a.token},method='POST')
            try:
                with urlopen(req,timeout=5) as response:
                    if response.status!=200:raise RuntimeError('Server rejected frame')
            except (HTTPError,URLError,TimeoutError) as exc:
                print('Frame delivery failed:',exc,flush=True)
                time.sleep(1)
            remaining=1/a.fps-(time.monotonic()-tick)
            if remaining>0:time.sleep(remaining)
    except KeyboardInterrupt:
        print('\nStopped Pi camera sender.')
    finally:
        camera.stop()
        camera.close()


if __name__=='__main__':main()
