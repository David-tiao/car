#!/usr/bin/env python3
"""OFFLINE REPLAY of user's ULog43 CSV into the website. NOT live robot data.

Values match kart_log_view_mecanum_v4(1).html mapping:
  time ch5 ms, x/y ch32/33 meters, yaw ch16 degrees,
  actual wheel pulses/s FL/FR/RL/RR ch28/29/30/31.
"""
import argparse
import csv
import json
import math
import time
from urllib.request import urlopen, Request


def main():
    ap=argparse.ArgumentParser(description='Test website live data pipeline with OFFLINE CSV REPLAY')
    ap.add_argument('csv_path',help='ULog43 CSV path (e.g. ~/Downloads/log0_kart.csv)')
    ap.add_argument('--server',default='http://127.0.0.1:8080')
    ap.add_argument('--speed',type=float,default=1.0)
    ap.add_argument('--max-seconds',type=float,default=30.0)
    args=ap.parse_args()
    if not 0.1<=args.speed<=20:ap.error('speed should be 0.1..20')
    radius=.04;ppr=440;lever=(.186+.22)/2;pulse_to_m=2*math.pi*radius/ppr
    print('OFFLINE ULog43 REPLAY ONLY, not a live bottom-chassis feed.')
    first_ms=None;start=time.monotonic();sent=0
    with open(args.csv_path,encoding='utf-8-sig',newline='') as f:
        for row in csv.reader(f):
            if len(row)<43 or (row[0].strip().startswith('#')):continue
            if not all(v.strip() for v in row): continue
            ch=[float(v) for v in row]
            t=ch[5]
            if first_ms is None:first_ms=t
            elapsed=(t-first_ms)/1000
            if elapsed<0 or elapsed>args.max_seconds:break
            x,y=ch[32],ch[33]
            yaw=math.radians(ch[16])
            wheel=[ch[i]*pulse_to_m for i in [28,29,30,31]]
            a,b,c,d=wheel
            vx=(a+b+c+d)/4
            vy=(-a+b+c-d)/4
            wz=(-a+b-c+d)/(4*lever)
            if not all(math.isfinite(z) for z in [x,y,yaw,vx,vy,wz]):continue
            delay=elapsed/args.speed-(time.monotonic()-start)
            if delay>0:time.sleep(delay)
            obj={'x':x,'y':y,'yaw':yaw,'vx':vx,'vy':vy,'wz':wz,
                 'source':'ulog43_replay_OFFLINE','frame':'log_odom'}
            req=Request(args.server.rstrip('/')+'/api/telemetry',json.dumps(obj).encode(),{'Content-Type':'application/json'},method='POST')
            with urlopen(req,timeout=2) as res:res.read()
            sent+=1
    print(f'Finished {sent} ULog43 samples; website live mode will show offline when replay stops.')

if __name__=='__main__':main()
