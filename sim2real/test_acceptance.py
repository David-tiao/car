#!/usr/bin/env python3
"""Explicit live TEST acceptance: exits nonzero unless ROS2->HTTP stream is working.

Run while `bash algorithm/car_ws/bash/web.sh test` is active. Read-only.
"""
from __future__ import annotations
import argparse
import json
import sys
import time
from urllib.request import urlopen


def get(url):
    with urlopen(url, timeout=1.5) as res:
        return json.load(res)


def checks(tel, inspector, previous_seq=None):
    problems = []
    if tel.get('source_type') != 'TEST' or not tel.get('online'):
        problems.append('TEST /odom 通道未在线')
    elif not tel.get('samples'):
        problems.append('TEST /odom 尚无里程计样本')
    if inspector.get('source_type') != 'MOCK' or not inspector.get('online'):
        problems.append('模拟诊断桥接未在线')
        return problems
    snap = inspector.get('snapshot') or {}
    topics = snap.get('topics') or {}
    samples = snap.get('samples') or {}
    for name in ('chassis','wheels','link','mission','vision','feedback','mechanism','power'):
        if not (topics.get(name) or {}).get('fresh'):
            problems.append('ROS2 话题未收到有效新帧：'+name)
    link=samples.get('link') or {}
    if not (link.get('connected') and link.get('heartbeat_fresh')):
        problems.append('模拟串口连接或 HEARTBEAT 不正常')
    if link.get('received_frames',0) < 20:
        problems.append('串口有效帧数不足 20')
    wheels=samples.get('wheels') or {}
    if len(wheels.get('actual_pulse_per_s') or [])!=4:
        problems.append('四轮实际速度缺少通道')
    if previous_seq is not None and tel.get('seq',0) <= previous_seq:
        problems.append('里程计序列号未增长')
    if snap.get('control_enabled') is not False:
        problems.append('测试集意外出现控制权限')
    return problems


def main():
    parser=argparse.ArgumentParser(description='TEST mock MCU -> PTY -> ROS2 -> HTTP acceptance')
    parser.add_argument('--server', default='http://127.0.0.1:8080')
    parser.add_argument('--timeout', type=float, default=15)
    args=parser.parse_args()
    if not args.server.startswith(('http://127.0.0.1:', 'http://localhost:', 'http://[::1]:')):
        parser.error('TEST acceptance only allows loopback server')
    if not 1<=args.timeout<=120:
        parser.error('Choose timeout 1..120 seconds')
    print('[CHECK] CAR-LINK V1 TEST chain (no motor control)', flush=True)
    end=time.monotonic()+args.timeout
    first_seq=None
    latest=[]
    while time.monotonic()<end:
        try:
            t=get(args.server+'/api/test/telemetry')
            i=get(args.server+'/api/test/inspection')
            if first_seq is None:first_seq=t.get('seq',0)
            latest=checks(t,i,first_seq)
            if not latest:
                samples=i['snapshot']['samples'];link=samples['link']
                print('[PASS] MOCK PTY -> CAR-LINK -> ROS2: '+str(link['received_frames'])+' frames',flush=True)
                print('[PASS] /odom -> HTTP TEST seq: '+str(first_seq)+' -> '+str(t['seq']),flush=True)
                print('[PASS] Four wheels, link heartbeat, mission, vision, feedback, mechanism, power fresh',flush=True)
                print('[PASS] TEST channel isolated from REAL API; no command transmitter launched',flush=True)
                print('[SUCCESS] /test browser acceptance can continue.',flush=True)
                return 0
        except (OSError,ValueError,TypeError,KeyError, json.JSONDecodeError) as exc:
            latest=[str(exc)]
        time.sleep(.55)
    print('[FAIL] TEST end-to-end acceptance not satisfied:',file=sys.stderr)
    for p in latest:print('  - '+p,file=sys.stderr)
    print('Check sim2real/data/web_modes/test_ros.log, test_web_bridge.log and test_inspection_bridge.log',file=sys.stderr)
    return 2

if __name__=='__main__':raise SystemExit(main())
