# OpenMV H7 Plus / Firmware 5.x
# Minimal first-batch source station: QR -> disk -> fixed slot -> ordered pick.
# This build intentionally DOES NOT execute ring/edge/blue or other groups.
import csi
import time
import math
import gc
import config as C
import utils as U


class App:
    def __init__(self):
        self.cam = csi.CSI()
        self.cam.reset()
        self.cam.pixformat(csi.GRAYSCALE)
        self.cam.framesize(csi.QVGA)
        self.cam.snapshot(time=1500)
        self.board = U.TaskBoard()
        self.lcd = U.TaskLCD()
        self.uart = U.VisionUART()
        self.lcd.update(self.board)
        self.state = 'QR'           # QR / MATERIAL / WAIT_ACK / DONE
        self.color_mode = False
        self.qr_last = None
        self.qr_hits = 0
        self.disk = U.DiskGate()
        self.material = U.MaterialTracker()
        self.frame = 0
        self.item = 0
        self.pick_slot = None
        self.disk_prev = False
        self.armed = False
        self.taking = False
        self.target_id = None
        self.slot_since = 0
        self.last_slot_tx = 0
        self.pending = None
        self.pending_ms = 0
        self.retries = 0
        self.last_status = time.ticks_ms()

    def camera_color(self, enabled):
        if self.color_mode == enabled:
            return
        self.cam.pixformat(csi.RGB565 if enabled else csi.GRAYSCALE)
        self.cam.framesize(csi.QVGA)
        self.cam.snapshot(time=350)
        if enabled:
            self.cam.auto_gain(False)
            self.cam.auto_whitebal(False)
        else:
            self.cam.auto_gain(True)
        self.color_mode = enabled

    def reset(self):
        if self.disk_prev:
            self.uart.send('#DISK,0\n')
        self.board.reset()
        self.lcd.update(self.board)
        self.state = 'QR'
        self.qr_last, self.qr_hits = None, 0
        self.item = 0
        self.frame = 0
        self.disk = U.DiskGate()
        self.material.clear()
        self.pick_slot = None
        self.disk_prev = False
        self.armed = False
        self.taking = False
        self.target_id = None
        self.slot_since = 0
        self.pending = None
        self.retries = 0
        self.camera_color(False)

    def process_commands(self):
        for cmd in self.uart.poll():
            if cmd in ('R', '#RESET'):
                self.reset()
            elif cmd == '#ARM,0':
                self.armed = False
                self.target_id = None
                self.slot_since = 0
                self.uart.send('#ARM_OK,0\n')
            elif cmd == '#ARM,1':
                good = (self.state == 'MATERIAL' and self.disk.locked
                        and self.pick_slot is not None)
                self.armed = bool(good)
                self.target_id = None
                self.slot_since = 0
                self.uart.send('#ARM_OK,%d\n' % (1 if good else 0))
            elif cmd == '#TAKE':
                if self.state == 'WAIT_ACK':
                    self.taking = True
                    self.uart.send('#TAKE_OK\n')
            elif cmd == 'K' and self.state == 'WAIT_ACK' and self.taking:
                self.pending = None
                self.taking = False
                self.armed = False
                self.target_id = None
                self.item += 1
                if self.item == 3:
                    self.state = 'DONE'
                    self.uart.send('#BATCH1_DONE\n')
                else:
                    self.state = 'MATERIAL'
            elif cmd == 'B' and self.state == 'WAIT_ACK':
                self.pending = None
                self.taking = False
                self.armed = False
                self.target_id = None
                self.state = 'MATERIAL'
                self.uart.send('#ARM_OK,0\n')
            elif cmd == '#GROUP_DONE,1':
                if self.item == 3 and self.board.finish_group(1):
                    self.lcd.update(self.board)

    def scan_qr(self, img):
        code = None
        for qr in img.find_qrcodes():
            raw = U.normalize_code(U.val(qr, 'payload', ''))
            if U.parse_code(raw) is not None:
                code = raw
                break
        self.qr_hits = self.qr_hits + 1 if code and code == self.qr_last else (1 if code else 0)
        self.qr_last = code
        if code and self.qr_hits >= C.QR_CONFIRM_FRAMES and self.board.accept(code):
            self.lcd.update(self.board)
            self.uart.sequence(self.board)
            self.state = 'MATERIAL'
            self.camera_color(True)
            print('QR accepted / material stage')

    def update_disk(self, img, gray, blobs, now):
        disk = self.disk.update(gray, blobs, self.frame)
        if self.disk.locked != self.disk_prev:
            self.uart.send('#DISK,%d\n' % int(self.disk.locked))
            self.disk_prev = self.disk.locked
            if not self.disk.locked:
                self.material.clear()
                self.pick_slot = None
                self.armed = False
                self.target_id = None
                self.slot_since = 0
        if disk is None:
            img.draw_string((2, 2), 'SEARCH DISK', color=(255, 255, 0))
            return None
        x, y, r = disk
        img.draw_circle((int(x), int(y), int(r)), color=(0, 255, 0), thickness=2)
        slot = U.nearest_pick_slot(disk)
        self.pick_slot = slot
        if slot is not None:
            img.draw_cross(slot, color=(255, 255, 0), size=10)
            img.draw_circle((C.PICK_TARGET_X, C.PICK_TARGET_Y),
                            C.PICK_SLOT_TOL_PX, color=(0, 255, 255))
            if time.ticks_diff(now, self.last_slot_tx) >= C.PICK_SLOT_SEND_MS:
                self.uart.uart.write(('#SLOT,%d,%d,%d\n' %
                                      (slot[0], slot[1], int(round(r)))).encode('ascii'))
                self.last_slot_tx = now
        return disk

    def update_material(self, img, now):
        blobs = U.color_blobs(img)
        gray = img.to_grayscale(copy=True)
        try:
            disk = self.update_disk(img, gray, blobs, now)
            if disk is None:
                return False
            detected = U.material_candidates(img, gray, disk, blobs)
            self.material.update(detected, now)
            needed = int(self.board.groups[0][self.item])
            chosen = None
            active_track = False
            for t in self.material.items:
                if not t.seen:
                    continue
                tx, ty = int(round(t.kx.x)), int(round(t.ky.x))
                color = U.COLORS[t.label - 1]
                img.draw_circle((tx, ty, int(round(t.r))), color=color)
                img.draw_string((max(0, tx-15), max(0, ty-20)),
                                '%d %s' % (t.label, t.state), color=color)
                eligible = (t.label == needed and t.hits >= 3
                            and t.state == 'S' and U.in_pick_slot(t, self.pick_slot))
                if eligible:
                    active_track = True
                    if self.armed and self.state == 'MATERIAL':
                        if self.target_id != t.id:
                            self.target_id = t.id
                            self.slot_since = now
                        elif time.ticks_diff(now, self.slot_since) >= C.PICK_SLOT_CONFIRM_MS:
                            chosen = t
            if not active_track and self.state == 'MATERIAL':
                self.target_id = None
                self.slot_since = 0
            if chosen is not None:
                x, y, r = int(round(chosen.kx.x)), int(round(chosen.ky.x)), int(round(chosen.r))
                self.pending = '#OBJ,%d,%d,%d,%d,%d,%d,0\n' % (
                    needed, x, y, 2*r, 2*r, int(round(math.pi * r * r)))
                self.uart.send(self.pending)
                self.state = 'WAIT_ACK'
                self.armed = False
                self.taking = False
                self.pending_ms = now
                self.retries = 0
            return True
        finally:
            del gray

    def poll_ack(self, now, disk_valid):
        if self.state != 'WAIT_ACK' or not self.pending or self.taking:
            return
        if time.ticks_diff(now, self.pending_ms) >= C.OBJ_TAKE_TIMEOUT_MS:
            # Lost/rejected target: require a new ARM after visual re-acquisition.
            self.pending = None
            self.armed = False
            self.target_id = None
            self.state = 'MATERIAL'
            self.uart.send('#ARM_OK,0\n')
            return
        if not disk_valid or self.retries >= C.MAX_RESENDS:
            return
        fresh = any(t.seen and t.label == int(self.board.groups[0][self.item])
                    and t.hits >= 3 and t.state == 'S'
                    and U.in_pick_slot(t, self.pick_slot) for t in self.material.items)
        if fresh and time.ticks_diff(now, self.pending_ms) >= C.ACK_TIMEOUT_MS:
            self.uart.send(self.pending)
            self.pending_ms = now
            self.retries += 1

    def run(self):
        print('V5 minimal: QR / DISK / ORDERED PICK (first batch only)')
        while True:
            self.process_commands()
            img = self.cam.snapshot()
            now = time.ticks_ms()
            self.frame += 1
            if self.state == 'QR':
                self.scan_qr(img)
            elif self.state in ('MATERIAL', 'WAIT_ACK'):
                valid = self.update_material(img, now)
                self.poll_ack(now, valid)
            else:
                img.draw_string((5, 5), 'BATCH1 DONE', color=(0, 255, 0), scale=2)
            if time.ticks_diff(now, self.last_status) >= C.STATUS_PERIOD_MS:
                print('STATE=%s ITEM=%d DISK=%d' %
                      (self.state, self.item, int(self.disk.locked)))
                self.last_status = now
            if self.frame % 30 == 0:
                gc.collect()


if __name__ == '__main__':
    App().run()
