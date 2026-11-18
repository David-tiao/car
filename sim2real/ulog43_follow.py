#!/usr/bin/env python3
"""Follow a GROWING ULog43 CSV file and forward new rows to the live website.

Requires a separate logging application to append each row in real time. An old
static log file will produce NO live updates. This never commands the chassis.
"""
import argparse
import csv
import json
import math
import os
import time
from pathlib import Path
from urllib.request import Request, urlopen


def decode(line):
    if not line.strip() or line.startswith('#'):
        return None
    try:
        row=next(csv.reader([line]))
        if len(row)<43:
            return None
        ch=[float(v) for v in row]
        x,y,yaw=ch[32],ch[33],math.radians(ch[16])
        k=2*math.pi*.04/440
        a,b,c,d=[ch[i]*k for i in (28,29,30,31)]
        value={'x':x,'y':y,'yaw':yaw,'vx':(a+b+c+d)/4,
               'vy':(-a+b+c-d)/4,'wz':(-a+b-c+d)/(4*((.186+.22)/2)),
               'source':'ulog43_growing_file','frame':'log_odom'}
        return value if all(math.isfinite(value[k]) for k in ('x','y','yaw','vx','vy','wz')) else None
    except (ValueError,IndexError,StopIteration):
        return None


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('csv_path',help='A CSV file being appended in real time by your logger')
    ap.add_argument('--server',default='http://127.0.0.1:8080')
    args=ap.parse_args()
    filename=Path(args.csv_path).expanduser()
    if not filename.exists():ap.error(f'File does not exist: {filename}')
    print('Following new ULog43 CSV rows only; existing content is skipped.')
    print('This requires an ACTIVE logger which appends one ULog43 CSV row per sample.')
    last_sent=0.0
    with filename.open('r',encoding='utf-8-sig',errors='replace') as f:
        f.seek(0,os.SEEK_END)
        while True:
            start=f.tell()
            line=f.readline()
            if not line:
                time.sleep(.05)
                if filename.stat().st_size < start:
                    f.seek(0)
                continue
            if not line.endswith('\n'):
                f.seek(start)
                time.sleep(.03)
                continue
            data=decode(line)
            if not data or time.monotonic()-last_sent<.05:
                continue
            try:
                body=json.dumps(data).encode()
                req=Request(args.server.rstrip('/')+'/api/telemetry',body,{'Content-Type':'application/json'},method='POST')
                with urlopen(req,timeout=.5) as res:res.read()
                last_sent=time.monotonic()
            except OSError as exc:
                print('Send error:',exc)
                time.sleep(.5)

if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:pass
