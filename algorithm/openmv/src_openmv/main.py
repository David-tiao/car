# OpenMV H7 Plus / firmware 5.x. UART3 -> STM32F4.
# Source algorithms: demo_scan.py, demo_circle.py, demo_material.py and src_openmv.
# IMPORTANT: vision controls reporting only, NEVER motors or gripper.
import csi
import time
import gc
import math
import config as C
import utils as U


def camera_create():
    cam = csi.CSI()
    cam.reset()
    cam.pixformat(csi.GRAYSCALE)
    cam.framesize(csi.QVGA)
    cam.snapshot(time=1500)
    return cam


def camera_mode(cam, mode):
    # Colors/white turntable and blue outline require RGB565;
    # numbered rings/QR/black boundary are processed in grayscale.
    color = mode in ('MATERIAL', 'BLUE', 'RING')
    cam.pixformat(csi.RGB565 if color else csi.GRAYSCALE)
    cam.framesize(csi.QVGA)
    cam.snapshot(time=350)
    if color:
        cam.auto_gain(False)
        cam.auto_whitebal(False)
    else:
        cam.auto_gain(True)


def send_target(link, t):
    x, y, z, dist, valid = U.material_metric(t)
    # id, color, pixel_x, pixel_y, cap_r, Xmm, Ymm, Zmm,
    # slant_mm, tracker_state(U/M/S), quality, metric_valid
    link.send('TARGET', t.id, t.label,
              int(round(t.kx.x)), int(round(t.ky.x)), int(round(t.r)),
              x, y, z, dist, t.state, t.quality, valid)


def report_material(link, tracker, color_filter):
    visible = [t for t in tracker.items if t.seen and t.hits >= 3]
    for t in visible:
        x, y, z, dist, valid = U.material_metric(t)
        link.send('MAT', t.id, t.label, int(round(t.kx.x)),
                  int(round(t.ky.x)), int(round(t.r)), x, y, z,
                  dist, t.state, t.quality, valid)
    filtered = [t for t in visible if color_filter == 0 or t.label == color_filter]
    if filtered:
        # Physically nearest vertex if calibrated. Pixel-centre is only
        # a provisional alignment cue when metric validity is false.
        def score(t):
            x, y, z, dist, valid = U.material_metric(t)
            if valid:
                return (0, dist)
            return (1, (t.kx.x-C.CX_PX)**2 + (t.ky.x-C.CY_PX)**2)
        t = min(filtered, key=score)
        send_target(link, t)
        return t
    link.send('NONE', 'TARGET')
    return None


def main():
    cam = camera_create()
    serial = U.PacketSerial()
    board = U.TaskBoard()
    try:
        lcd = U.TaskLCD()
        lcd.update(board)
    except Exception as err:
        lcd = None
        print('LCD unavailable:', err)

    mode = 'QR'
    disk_gate = U.PersistentDiskGate()
    material_tracker = U.MaterialTracker()
    rings = U.RingFinder()
    frame = 0
    selected_color = 0  # 0=nearest of any color, 1..6 restrict selection
    selected_ring = 0   # 0=all, 1..3 desired numbered ring
    last_report = time.ticks_ms()
    last_stat = time.ticks_ms()
    last_disk_tx = time.ticks_ms()
    last_qr_tx = time.ticks_ms()
    last_qr = ''
    qr_count = 0
    last_qr_rect = None
    qr_rect_seen = 0
    was_locked = False
    edge_near = 0
    edge_clear = 0
    edge_latch = False

    print('OpenMV to STM32 framed UART3 active')
    print('Commands: MODE, COLOR, RINGID, DONE, RESET, PING')
    while True:
        now = time.ticks_ms()
        frame += 1

        # STM32 decides the mission step. Vision MUST NOT advance its own step
        # simply because an object is seen, or because a packet is delivered.
        for packet in serial.poll():
            command = packet[0]
            try:
                if command == 'MODE' and len(packet) == 2 and packet[1] in (
                        'QR', 'MATERIAL', 'RING', 'EDGE', 'BLUE'):
                    new_mode = packet[1]
                    if new_mode != mode:
                        if mode == 'MATERIAL':
                            serial.send('DISK', 0, 0, 0, 0)
                            serial.send('NONE', 'TARGET')
                        camera_mode(cam, new_mode)
                        mode = new_mode
                        disk_gate = U.PersistentDiskGate()
                        material_tracker.clear()
                        rings.reset()
                        frame = 0
                        was_locked = False
                        qr_count = 0
                        last_qr_rect = None
                        edge_near = edge_clear = 0
                        edge_latch = False
                    serial.send('ACK', 'MODE', mode)
                elif command == 'COLOR' and len(packet) == 2:
                    x = int(packet[1])
                    if not 0 <= x <= 6:
                        raise ValueError('COLOR range')
                    selected_color = x
                    serial.send('ACK', 'COLOR', x)
                elif command == 'RINGID' and len(packet) == 2:
                    x = int(packet[1])
                    if not 0 <= x <= 3:
                        raise ValueError('RING range')
                    selected_ring = x
                    serial.send('ACK', 'RINGID', x)
                elif command == 'DONE' and len(packet) == 2:
                    x = int(packet[1])
                    if board.finish_group(x):
                        if lcd:
                            lcd.update(board)
                        serial.send('ACK', 'DONE', x)
                    else:
                        serial.send('ERR', 'DONE_ORDER')
                elif command == 'RESET' and len(packet) == 1:
                    board.reset()
                    if lcd:
                        lcd.update(board)
                    disk_gate = U.PersistentDiskGate()
                    material_tracker.clear()
                    rings.reset()
                    selected_color = selected_ring = 0
                    last_qr = ''
                    qr_count = 0
                    last_qr_rect = None
                    was_locked = False
                    mode = 'QR'
                    camera_mode(cam, mode)
                    serial.send('ACK', 'RESET')
                elif command == 'PING' and len(packet) == 1:
                    serial.send('PONG')
                else:
                    serial.send('ERR', 'BAD_COMMAND')
            except (ValueError, TypeError):
                serial.send('ERR', 'BAD_ARGUMENT')

        img = cam.snapshot()
        now = time.ticks_ms()

        if mode == 'QR':
            seen = None
            for qr in img.find_qrcodes():
                raw = U.normalize_code(U.val(qr, 'payload', ''))
                if U.parse_code(raw) is None:
                    continue
                seen = raw
                last_qr_rect = U.val(qr, 'rect', None)
                qr_rect_seen = now
                break
            if seen is None:
                qr_count = 0
            elif seen == last_qr:
                qr_count += 1
            else:
                last_qr = seen
                qr_count = 1
            if seen and qr_count >= C.QR_CONFIRM_FRAMES and board.accept(seen):
                if lcd:
                    lcd.update(board)
                serial.send('QR', board.code())
                last_qr_tx = now
            # Repeat QR after lock until STM32 knows it; doesn't affect groups.
            if board.locked() and time.ticks_diff(now, last_qr_tx) >= 1000:
                serial.send('QR', board.code())
                last_qr_tx = now
            if last_qr_rect is not None and time.ticks_diff(now, qr_rect_seen) < C.QR_RECT_HOLD_MS:
                img.draw_rectangle(tuple(last_qr_rect), color=255, thickness=2)

        elif mode == 'MATERIAL':
            # The only path to material outputs is the strict white turntable
            # gate, not arbitrary color blobs elsewhere in the image.
            blobs = U.color_blobs(img)
            gray = img.to_grayscale(copy=True)
            disk = disk_gate.update(gray, blobs, frame)
            if bool(disk) != was_locked:
                serial.send('DISK', 1 if disk else 0,
                            int(disk[0]) if disk else 0,
                            int(disk[1]) if disk else 0,
                            int(disk[2]) if disk else 0)
                if not disk:
                    serial.send('NONE', 'TARGET')
                    material_tracker.clear()
                was_locked = bool(disk)
                last_disk_tx = now
            if disk is not None and time.ticks_diff(now, last_disk_tx) >= C.STATUS_PERIOD_MS:
                serial.send('DISK', 1, int(disk[0]), int(disk[1]), int(disk[2]))
                last_disk_tx = now
            if disk is not None:
                xc, yc, rr = (int(round(a)) for a in disk)
                img.draw_circle((xc, yc, rr), color=(0, 255, 0), thickness=2)
                detections = U.material_candidates(img, gray, disk)
                material_tracker.update(detections, now)
                for t in material_tracker.items:
                    if t.seen:
                        x, y, r = int(round(t.kx.x)), int(round(t.ky.x)), int(round(t.r))
                        img.draw_circle((x, y, r), color=U.COLORS[t.label - 1], thickness=2)
                        img.draw_cross((x, y), color=(255, 0, 0), size=8)
                        img.draw_string((max(0, x - 12), max(0, y - r - 15)),
                                        '%d%s' % (t.label, t.state),
                                        color=U.COLORS[t.label - 1])
                if time.ticks_diff(now, last_report) >= C.MAT_REPORT_PERIOD_MS:
                    report_material(serial, material_tracker, selected_color)
                    last_report = now
            else:
                img.draw_string((3, 3), 'NO DISK / MATERIAL BLOCKED',
                                color=(255, 255, 0))
                if time.ticks_diff(now, last_report) >= C.MAT_REPORT_PERIOD_MS:
                    serial.send('NONE', 'TARGET')
                    last_report = now
            del gray

        elif mode == 'RING':
            rings.process(img)
            rings.draw(img)
            if time.ticks_diff(now, last_report) >= 250:
                found = False
                for t in rings.tracks:
                    digit = t.voted_digit(now)
                    if digit is None or not 1 <= digit <= 3:
                        continue
                    if selected_ring and digit != selected_ring:
                        continue
                    x, y, z, d, valid = U.ring_metric(t)
                    serial.send('RING', digit, int(round(t.x.x)), int(round(t.y.x)),
                                x, y, z, d, int(round(t.score)), valid)
                    found = True
                if not found:
                    serial.send('NONE', 'RING')
                last_report = now

        elif mode == 'EDGE':
            line = U.detect_edge(img)
            if line is not None:
                img.draw_line(line[:4], color=255, thickness=2)
                if time.ticks_diff(now, last_report) >= 250:
                    serial.send('EDGE', *line)
                    last_report = now
            if line and line[4] >= C.EDGE_STOP_ROW:
                edge_near += 1
                edge_clear = 0
            else:
                edge_clear += 1
                edge_near = 0
            if edge_near >= C.EDGE_CONFIRM and not edge_latch:
                serial.send('STOP', 'EDGE')
                edge_latch = True
            if edge_clear >= C.EDGE_RELEASE:
                edge_latch = False

        elif mode == 'BLUE':
            b = U.detect_blue(img)
            if b is not None:
                x, y, w, h, cx, cy, kind = b
                img.draw_rectangle((x, y, w, h), color=(255, 255, 0), thickness=2)
                img.draw_cross((cx, cy), color=(255, 0, 0), size=9)
                if time.ticks_diff(now, last_report) >= 250:
                    serial.send('BLUE', *b)
                    last_report = now

        if time.ticks_diff(now, last_stat) >= C.STATUS_PERIOD_MS:
            serial.send('STATUS', mode, int(was_locked), selected_color,
                        int(board.locked()), board.completed)
            last_stat = now
        if frame % 24 == 0:
            gc.collect()


if __name__ == '__main__':
    main()
