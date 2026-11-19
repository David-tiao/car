"""OpenMV H7 Plus · 2027 智能搬运初赛入口。

同目录文件为 main.py、config.py、tools.py、serial.py。在 IDE 中运行本文件。
上电独立自启时，把这四个文件一同放进 OpenMV 的启动盘根目录。
现有 V5 主控在首批三件后停车。余下工位的到站、动作取用和完成回执见 serial.py。
"""
import gc
import math
import os
import sys
import time


def _add_project_directory():
    # IDE 直接执行脚本时可能不提供 __file__，板内同级模块仍需可导入。
    filename = globals().get('__file__') or ''
    folders = ['openmv', '/sd/openmv', '/flash/openmv']
    if '/' in filename:
        folders.insert(0, filename.rsplit('/', 1)[0] or '/')
    for directory in folders:
        try:
            os.stat(directory + '/tools.py')
            os.stat(directory + '/serial.py')
            os.stat(directory + '/config.py')
        except OSError:
            continue
        sys.path.insert(0, directory)
        return


_add_project_directory()
import algorithm.openmv.openmv_all.config as C
from serial import SerialLink
from algorithm.openmv.openmv_all.tools import Camera, MaterialDetector, Mission, Panel, SourceStation, StablePoint, StationVision, near


class App:
    def __init__(self, link, panel):
        self.link = link
        self.panel = panel
        self.materials = MaterialDetector()
        self.stations = StationVision(self.materials)
        self.token = 0
        self.reset('READY')

    def reset(self, state):
        if getattr(self, 'disk_visible', False):
            self.link.send('DISK', 0)
        self.job = Mission()
        self.state = state
        self.repeat_seq = False
        self.source = None
        self.disk_visible = False
        self.slot_tx = None
        self.armed = False
        self.obj_track = None
        self.retry_obj = False
        self.stable = StablePoint()
        self.offer = None
        self.used = None
        self.last_result = None
        self.report_at = None
        self.panel.update(self.job, self.state)

    def send_seq(self, now):
        self.link.send('SEQ', *''.join(self.job.groups))
        self.seq_at = now

    def scan(self, img, now):
        if self.job.scan(img):
            self.source = SourceStation(self.job.groups[0], self.materials)
            self.state = 'RAW'
            self.repeat_seq = True
            self.send_seq(now)

    def legacy_command(self, command, now):
        name = command[0]
        if command == ('ARM', 1):
            self.repeat_seq = False
            self.armed = bool(self.state == 'RAW' and self.disk_visible
                              and self.slot_tx is not None
                              and time.ticks_diff(now, self.slot_tx) <= C.MAX_FRAME_GAP_MS)
            self.link.send('ARM_OK', int(self.armed))
        elif command == ('ARM', 0):
            self.armed = self.retry_obj = False
            self.link.send('ARM_OK', 0)
        elif name == 'TAKE' and self.state in ('OFFER', 'TAKING'):
            self.state = 'TAKING'
            self.link.send('TAKE_OK')
        elif name == 'K' and self.state == 'TAKING':
            self.job.confirm()
            self.armed = False
            self.obj_track = None
            self.state = 'RAW' if self.job.step <= 3 else 'WAIT'
            if self.state == 'WAIT':
                self.link.send('BATCH1_DONE')
                self.disk_visible = False
                self.source = None
        elif name == 'B' and self.state in ('OFFER', 'TAKING'):
            self.state = 'RAW'
            self.armed = self.retry_obj = False
            self.obj_track = None
            self.link.send('ARM_OK', 0)
        # GROUP_DONE 只通知批次结束。每件已由完成回执 K 计数。

    def source_sample(self, img, now, legacy):
        color = self.job.current()[2]
        candidate = self.source.observe(img, color, now)
        slot = self.source.slot
        visible = slot is not None
        if legacy and visible != self.disk_visible:
            self.link.send('DISK', int(visible))
            if not visible:
                self.armed = False
            self.slot_tx = None
        if not legacy and self.disk_visible and not visible:
            self.link.send('RAW_LOST', self.job.step)
            self.slot_tx = None
        self.disk_visible = visible
        if visible and (self.slot_tx is None or time.ticks_diff(now, self.slot_tx) >= C.REPORT_MS):
            if legacy:
                self.link.send('SLOT', slot[0], slot[1], self.source.radius)
            else:
                self.link.send('RAW_SLOT', self.job.step, slot[0], slot[1], self.source.radius)
            self.slot_tx = now
        if not visible or not near(slot, (C.PICK_TARGET_X, C.PICK_TARGET_Y), C.PICK_SLOT_TOL_PX):
            return None
        return candidate

    def legacy_frame(self, img, now):
        if self.repeat_seq and time.ticks_diff(now, self.seq_at) >= C.SEQ_REPEAT_MS:
            self.send_seq(now)
        if self.state == 'TAKING':
            return  # Occlusion during a confirmed pickup is expected.
        face = self.source_sample(img, now, True)
        if self.state == 'OFFER':
            if time.ticks_diff(now, self.obj_since) >= C.OBJ_TAKE_TIMEOUT_MS:
                self.state = 'RAW'
                self.armed = self.retry_obj = False
                self.link.send('ARM_OK', 0)
                return
            if (not self.retry_obj or not face or face[3] != self.obj_track or
                    time.ticks_diff(now, self.obj_tx) < C.OBJ_RETRY_MS):
                return
        elif not (face and self.armed):
            return
        else:
            self.state = 'OFFER'
            self.armed = False
            self.retry_obj = True
            self.obj_since = now
            self.obj_track = face[3]
        x, y, r, _ = face
        self.link.send('OBJ', self.job.current()[2], x, y, 2 * r, 2 * r, int(round(math.pi * r * r)), 0)
        self.obj_tx = now

    def at_station(self, step):
        if not (4 <= step <= 25 and step == self.job.step and self.job.current()):
            self.link.send('ERR', 'STEP')
            return
        if self.state == 'WAIT':
            if step == 13 and self.source is None:
                self.source = SourceStation(self.job.groups[2], self.materials)
            self.state = 'LOOK'
            self.clear_offer()
        self.link.send('AT_OK', step)

    def clear_offer(self):
        self.stable.reset()
        self.offer = None
        self.report_at = None

    def station_frame(self, img, now):
        op, station, color, ring = self.job.current()
        if station == 'RAW':
            face = self.source_sample(img, now, False)
            point = face[:2] if face else None
        elif op == 'PLACE':
            point = self.stations.empty_ring(img, ring)
        elif op in ('PICK', 'STACK'):
            point = self.stations.occupied_ring(img, station, ring, color)
        else:
            point = self.stations.blue_home(img)
        point = self.stable.update(point, now)
        if point is None:
            if self.offer is not None:
                self.link.send('LOST', self.job.step, self.offer)
            self.offer = None
            return
        if self.offer is None:
            self.token += 1
            self.offer = self.token
            self.report_at = None
        self.offer_at = now
        if self.report_at is None or time.ticks_diff(now, self.report_at) >= C.REPORT_MS:
            self.link.send('OBS', self.job.step, self.offer, op, station, color, ring, point[0], point[1])
            self.report_at = now

    def use_offer(self, step, token, now):
        if self.state == 'ACT' and self.used == (step, token):
            self.link.send('USE_OK', step, token)
        elif (self.state == 'LOOK' and step == self.job.step and token == self.offer
              and time.ticks_diff(now, self.offer_at) <= C.MAX_FRAME_GAP_MS):
            self.used = (step, token)
            self.state = 'ACT'
            self.link.send('USE_OK', step, token)
        else:
            self.link.send('ERR', 'STALE')

    def result(self, step, token, outcome):
        key = (step, token, outcome)
        if key != self.last_result:
            if self.state != 'ACT' or self.used != (step, token) or outcome not in ('OK','FAIL'):
                self.link.send('ERR', 'RESULT')
                return
            if outcome == 'OK':
                self.job.confirm()
            self.state = 'DONE' if not self.job.current() else 'WAIT'
            self.used = None
            self.clear_offer()
            self.last_result = key
        self.link.send('RESULT_OK', step, token, outcome, self.job.picks, self.job.places)

    def commands(self, now):
        for command in self.link.poll():
            name = command[0]
            if name == 'RESET':
                self.reset('QR')
            elif name in ('K', 'B', 'TAKE', 'ARM', 'GROUP_DONE'):
                self.legacy_command(command, now)
            elif name == 'STATUS':
                self.link.send('STATE', self.job.step, self.state, self.job.picks, self.job.places)
            elif name == 'AT':
                self.at_station(command[1])
            elif name == 'USE':
                self.use_offer(command[1], command[2], now)
            elif name == 'RESULT':
                self.result(command[1], command[2], command[3])
            else:
                self.link.send('ERR', 'CMD')

    def frame(self, img, now):
        if self.state == 'QR':
            self.scan(img, now)
        elif self.state in ('RAW', 'OFFER', 'TAKING'):
            self.legacy_frame(img, now)
        elif self.state == 'LOOK':
            self.station_frame(img, now)
        self.panel.update(self.job, self.state)


def run():
    link = SerialLink()
    camera = Camera()
    app = App(link, Panel())
    frames = 0
    while True:
        app.commands(time.ticks_ms())
        img = camera.snapshot()
        app.frame(img, time.ticks_ms())
        frames += 1
        if frames % C.GC_EVERY_FRAMES == 0:
            gc.collect()


if __name__ == '__main__':
    run()
