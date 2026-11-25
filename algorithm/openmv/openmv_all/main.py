"""OpenMV H7 Plus · 2027 智能搬运初赛入口。

同目录文件为 main.py、config.py、tools.py、serial.py。在 IDE 中运行本文件。
上电独立自启时，把这四个文件一同放进 OpenMV 的启动盘根目录。
默认对接 architecture-v2 电控。旧主控可在 config.py 选择 V5。
"""
import gc
import math
import os
import sys
import time


def _add_project_directory():
    # IDE 直接执行脚本时可能不提供 __file__，板内同级模块仍需可导入。
    filename = globals().get('__file__') or ''
    folders = ['.', '/flash', '/sd', '/', 'openmv_all', 'openmv', '/sd/openmv', '/flash/openmv']
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
import config as C
from serial import SerialLink, V4Link
from tools import Camera, MaterialDetector, Mission, Panel, SourceStation, StablePoint, StationVision, near


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


class ElectricalApp:
    """按 STM32 的会话工作。同一份相机识别器供两种主控使用。"""
    def __init__(self, link, panel):
        self.link = link
        self.panel = panel
        self.materials = MaterialDetector()
        self.stations = StationVision(self.materials)
        self.reset()
        self.link.send('READY', 'IDLE')

    def reset(self):
        self.board = Mission()
        self.task_session = 0
        self.source_batch = None
        self.message = None
        self.last_done = None
        self.prepared = None
        self.home_zone = None
        self.source = None
        self.idle()

    def idle(self):
        self.state = 'IDLE'
        self.request = None
        self.session = 0
        self.step = 0
        self.wait_step = False
        self.last_report = None
        self.qr = None
        self.stable = StablePoint()

    @staticmethod
    def number(s, low, high):
        if not s.isdigit() or len(s) > 5 or not low <= int(s) <= high:
            raise ValueError('FORMAT')
        return int(s)

    def start(self, command, scan_prefix):
        if self.request == command and not scan_prefix:
            self.link.send(*self.accept)
            return
        if self.state != 'IDLE' and not scan_prefix:
            raise ValueError('BUSY')
        if command[0] == 'ALIGN':
            if len(command) != 3:
                raise ValueError('FORMAT')
            ring = self.number(command[1], 1, 3)
            session = self.number(command[2], 1, 65535)
            self.idle()
            self.target, self.session = ring, session
            self.state = 'RING'
            self.accept = ('ACK', 'ALIGN', ring, session)
        else:
            if len(command) not in (2, 3, 4):
                raise ValueError('FORMAT')
            job = self.number(command[1], 1, 255)
            if job not in (1, 40, 41):
                raise ValueError('UNSUPPORTED_JOB')
            session = 0
            tx = ty = C.PICK_SLOT_TOL_PX
            if len(command) == 3:
                if job != 1:
                    raise ValueError('FORMAT')
                session = self.number(command[2], 0, 65535)
            elif len(command) == 4:
                tx = self.number(command[2], 1, 100)
                ty = self.number(command[3], 1, 100)
            if job == 41 and (self.source is None or not self.source.disk.locked):
                raise ValueError('AREA_REQUIRED')
            self.idle()
            self.session = session
            self.tol_x, self.tol_y = tx, ty
            if job == 1:
                self.reset()
                self.session = self.task_session = session
                self.state = 'QR'
            elif job == 40:
                self.state = 'AREA'
                colors = self.board.groups[0] if self.board.groups else range(1, 7)
                self.source = SourceStation(colors, self.materials)
            else:
                self.state = 'MATERIAL'
                self.source.tracker.clear()
                self.source.track_id = None
            self.accept = ('ACK', 'JOB', job)
        self.request = command
        self.link.send(*self.accept)

    def command(self, cmd):
        name = cmd[0]
        if name == 'CHECK':
            self.task_check(cmd)
        elif name in ('JOB', 'SCAN_BEGIN', 'ALIGN'):
            request = ('JOB',) + cmd[1:] if name == 'SCAN_BEGIN' else cmd
            self.start(request, name == 'SCAN_BEGIN')
        elif cmd == ('RESET',):
            self.reset()
            self.link.send('IDLE')
        elif cmd == ('CANCEL',):
            self.task_session = 0
            self.last_done = None
            self.prepared = None
            self.idle()
            self.source = None
            self.link.send('IDLE')
        elif cmd == ('STATUS',):
            if self.state == 'IDLE':
                self.link.send('READY', 'IDLE')
            else:
                self.link.send('STATUS', self.state)
        elif name == 'ACK' and len(cmd) == 3 and cmd[1] == 'QR':
            if (self.state == 'QR' and self.qr is not None and
                    self.number(cmd[2], 0, 65535) == self.session):
                self.link.finish('DONE', 'QR')
                self.idle()
        elif name == 'STEP_DONE' and len(cmd) == 3:
            step = self.number(cmd[1], 0, 65535)
            if self.state == 'RING' and self.wait_step and step == self.step and cmd[2] in ('0', '1'):
                self.step = (self.step + 1) & 65535
                self.wait_step = False
                self.last_report = None
                self.stable.reset()
        elif name == 'ARRIVED' and len(cmd) == 7 and cmd[1] == 'RING':
            if (self.state == 'RING' and self.wait_step and
                    self.number(cmd[2], 1, 3) == self.target and
                    self.number(cmd[5], 1, 65535) == self.session and
                    self.number(cmd[6], 1, 255) > 0 and
                    (int(cmd[3]), int(cmd[4])) == self.last_error_px):
                self.idle()
                self.link.send('IDLE')
        else:
            raise ValueError('UNKNOWN_CMD')

    def task_check(self, cmd):
        # CHECK,scan session,manipulation step,B(efore)/A(fter),colour,ring
        if len(cmd) != 6 or cmd[3] not in ('B', 'A'):
            raise ValueError('FORMAT')
        session = self.number(cmd[1], 1, 65535)
        step = self.number(cmd[2], 1, 25)
        color = self.number(cmd[4], 0, 6)
        ring = self.number(cmd[5], 0, 3)
        if session != self.task_session:
            raise ValueError('SESSION')
        if cmd == self.last_done:
            self.link.send('DID', session, step)
            return
        item = self.board.current()
        if not item or step != self.board.step or color != item[2]:
            raise ValueError('STEP')
        if item[0] == 'HOME':
            if ring not in (1, 2) or C.HOME_WINDOWS[ring] is None:
                raise ValueError('CAL_HOME')
            if cmd[3] == 'A' and self.home_zone != ring:
                raise ValueError('ZONE')
            self.home_zone = ring
        elif ring != item[3]:
            raise ValueError('STEP')
        if self.request == cmd:
            return  # Keep accumulating fresh frames across MCU retries.
        if self.state not in ('IDLE', 'CHECK') or (cmd[3] == 'A' and self.prepared != step):
            raise ValueError('PHASE')
        if self.state == 'CHECK' and self.request[3] == 'A':
            raise ValueError('PHASE')
        if item[1] == 'RAW' and cmd[3] == 'B':
            batch = 0 if step < 13 else 1
            if self.source is None or self.source_batch != batch:
                self.source = SourceStation(self.board.groups[batch*2], self.materials)
                self.source_batch = batch
            else:
                # Keep the verified disk anchor when only one or two pieces remain.
                self.source.tracker.clear()
                self.source.track_id = None
                self.source.last_frame = None
        if item[0] != 'HOME':
            slot = (step-1) % 3
            key = 'PICK' if item[0] == 'PICK' else 'PUT'
            windows = [C.TRAY_STEP_WINDOWS.get((step, ph), C.TRAY_WINDOWS[key, ph][slot])
                       for ph in ('B', 'A')]
            self.tray_window = windows[0 if cmd[3] == 'B' else 1]
            if (None in windows
                    or C.TRAY_EMPTY_LAB is None):
                raise ValueError('CAL_TRAY')
        if item[0] == 'STACK':
            lower, upper = C.STACK_WINDOWS[ring]
            if (not lower or not upper or max(abs(lower[0]-upper[0]), abs(lower[1]-upper[1]))
                    <= lower[2]+upper[2]):
                raise ValueError('CAL_STACK')
        self.request = cmd
        self.message = None
        self.state = 'CHECK'
        self.stable.reset()
        self.last_report = None

    def task_frame(self, img, now):
        op, station, color, ring = self.board.current()
        before = self.request[3] == 'B'
        point = None
        if op == 'HOME':
            point = self.stations.home_patch(img, C.HOME_WINDOWS[self.home_zone])
        elif self.stations.tray_evidence(img, self.tray_window,
                color if (op == 'PICK') != before else 0):
            if station == 'RAW':
                if before:
                    face = self.source.observe(img, color, now)
                    if face and near(face[:2], (C.PICK_TARGET_X,C.PICK_TARGET_Y), C.PICK_SLOT_TOL_PX):
                        point = face[:2]
                else:
                    point = self.tray_window[:2]
            elif op == 'STACK':
                window = C.STACK_WINDOWS[ring][0 if before else 1]
                point = self.stations.face_in_window(img, window, color)
            else:
                empty = (op == 'PLACE') == before
                point = (self.stations.empty_ring(img, ring) if empty else
                         self.stations.occupied_ring(img, station, ring, color))
                if point and not near(point, (C.RING_TARGET_X, C.RING_TARGET_Y), C.PICK_SLOT_TOL_PX):
                    point = None
        if self.stable.update(point, now) is None or not self.due(now):
            return
        step = self.board.step
        if before:
            self.prepared = step
            self.link.send('CAN', self.task_session, step)
            self.last_report = now
        else:
            self.last_done = self.request
            self.board.confirm()
            self.prepared = None
            self.idle()
            self.link.send('DID', self.task_session, step)

    def commands(self, now):
        for cmd in self.link.poll():
            try:
                self.command(cmd)
            except ValueError as exc:
                self.message = str(exc)
                self.link.send('ERR', self.message)

    def due(self, now):
        return self.last_report is None or time.ticks_diff(now, self.last_report) >= C.REPORT_MS

    def qr_frame(self, img, now):
        if self.qr is None and self.board.scan(img):
            self.qr = '+'.join(self.board.groups)
        if self.qr and self.due(now):
            if self.session:
                self.link.send('QR', self.qr, self.session)
            else:
                self.link.send('QR', self.qr)
            self.last_report = now

    def ring_frame(self, img, now):
        if self.wait_step:
            return
        point = self.stable.update(self.stations.empty_ring(img, self.target), now)
        if point is None:
            if self.due(now):
                self.link.send('LOST', 'RING', self.target)
                self.last_report = now
            return
        cx, cy = point
        dx, dy = cx-C.RING_TARGET_X, cy-C.RING_TARGET_Y
        self.last_error_px = (dx, dy)
        self.link.send('POS', 'RING', self.target, dx, dy, cx, cy, self.session, self.step)
        self.wait_step = True

    def material_frame(self, img, now):
        # Return a real tracking measurement, never a projected slot or stale centroid.
        face = self.source.observe(img, None, now)
        if face is None:
            if self.due(now):
                self.link.send('LOST', 'MATERIAL', 0)
                self.last_report = now
            return
        cx, cy = face[:2]
        dx, dy = cx-C.PICK_TARGET_X, cy-C.PICK_TARGET_Y
        if abs(dx) <= self.tol_x and abs(dy) <= self.tol_y:
            self.link.finish('ARRIVED', 'MATERIAL', 0, dx, dy)
            self.idle()
        elif self.due(now):
            self.link.send('POS', 'MATERIAL', 0, dx, dy, cx, cy)
            self.last_report = now

    def frame(self, img, now):
        if self.state == 'QR':
            self.qr_frame(img, now)
        elif self.state == 'RING':
            self.ring_frame(img, now)
        elif self.state == 'CHECK':
            self.task_frame(img, now)
        elif self.state == 'AREA':
            disk, _, _ = self.source.disk_sample(img)
            if disk:
                cx, cy, radius = [int(round(value)) for value in disk]
                self.link.finish('AREA_OK', cx, cy, radius)
                self.idle()
            elif self.due(now):
                self.link.send('LOST', 'AREA')
                self.last_report = now
        elif self.state == 'MATERIAL':
            self.material_frame(img, now)
        label = self.state
        if self.state == 'CHECK':
            label = 'LOOK' if self.request[3] == 'B' else 'VERIFY'
        elif self.board.groups and self.board.current() is None:
            label = 'DONE'
        self.panel.update(self.board, self.message or label)


def run():
    if C.UART_PROTOCOL == 'V4':
        link, app_type = V4Link(), ElectricalApp
    elif C.UART_PROTOCOL == 'V5':
        link, app_type = SerialLink(), App
    else:
        raise ValueError('UART_PROTOCOL must be V4 or V5')
    camera = Camera()
    app = app_type(link, Panel())
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
