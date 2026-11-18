# main.py - Integrated OpenMV H7 Plus vision state machine, firmware 5.x
# QR -> material group -> black-numbered ring group -> material -> ring.
# LCD remains the confirmed QR task board, never the camera picture.
# STM32 must implement #GROUP_DONE,1..4 to remove each group on the LCD.
import csi
import time
import gc
import config as C
import utils as U


def make_camera():
    cam = csi.CSI()
    cam.reset()
    cam.pixformat(csi.GRAYSCALE)
    cam.framesize(csi.QVGA)
    cam.snapshot(time=1500)
    return cam


def set_camera(cam, color):
    cam.pixformat(csi.RGB565 if color else csi.GRAYSCALE)
    cam.framesize(csi.QVGA)
    cam.snapshot(time=350)
    if color:
        cam.auto_gain(False)
        cam.auto_whitebal(False)
    else:
        cam.auto_gain(True)


def main():
    cam = make_camera()
    lcd = U.TaskLCD()
    link = U.VisionUART()
    board = U.TaskBoard()
    lcd.update(board)

    # States: QR, MATERIAL, WAIT_OBJ_ACK, RING, WAIT_RING_ACK, DONE.
    state = 'QR'
    mode = 'AUTO'             # AUTO / EDGE / BLUE
    g_index, item_index = 0, 0
    previous_color = False
    last_qr, qr_count = '', 0
    qr_box, qr_box_time = None, 0
    frame_no = 0
    history, stable_count = [], 0
    rings = U.RingFinder()
    last_packet, resend_count = '', 0
    last_send_ms = 0
    last_status_ms = time.ticks_ms()
    last_aux_ms = time.ticks_ms()
    edge_near, edge_clear, edge_latched = 0, 0, False

    def reset_all():
        nonlocal state, mode, g_index, item_index, previous_color
        nonlocal last_qr, qr_count, qr_box, history, stable_count
        nonlocal last_packet, resend_count, edge_near, edge_clear, edge_latched
        board.reset()
        mode, state = 'AUTO', 'QR'
        g_index, item_index = 0, 0
        last_qr, qr_count, qr_box = '', 0, None
        history, stable_count = [], 0
        last_packet, resend_count = '', 0
        edge_near, edge_clear, edge_latched = 0, 0, False
        rings.reset()
        set_camera(cam, False)
        previous_color = False
        lcd.update(board)
        print('RESET -> SCAN QR')

    def next_item():
        nonlocal g_index, item_index, state, history, stable_count
        nonlocal last_packet, resend_count, previous_color
        item_index += 1
        history, stable_count = [], 0
        last_packet, resend_count = '', 0
        if item_index >= 3:
            g_index += 1
            item_index = 0
            if g_index >= 4:
                state = 'DONE'
                link.send('#DONE\n')
                print('AUTO FLOW DONE; LCD waits for GROUP_DONE confirmations')
                return
            rings.reset()
            print('NEXT GROUP %d: %s' % (g_index+1, board.groups[g_index]))
        state = 'MATERIAL' if (g_index % 2 == 0) else 'RING'
        need_color = True  # Both materials and IDE red ring crosses are RGB565.
        if previous_color != need_color:
            set_camera(cam, need_color)
            previous_color = need_color

    print('OpenMV integrated V1 ready')
    print('STM32 replies K/B/R; LCD updates ONLY on #GROUP_DONE,n')
    print('Aux commands: #MODE,EDGE  #MODE,BLUE  #MODE,AUTO  #RESET')

    while True:
        frame_no += 1
        now = time.ticks_ms()

        # ----- Nonblocking UART input -----
        for cmd in link.poll():
            if cmd in ('R', '#RESET'):
                reset_all()
                continue
            if cmd.startswith('#GROUP_DONE,'):
                part = cmd[len('#GROUP_DONE,'):]
                if len(part) == 1 and part in '1234' and board.finish_group(int(part)):
                    lcd.update(board)
                    print('GROUP DONE ->', board.remaining_text() or 'DONE')
                else:
                    print('IGNORED GROUP_DONE:', cmd)
                continue
            if cmd in ('#MODE,EDGE', '#MODE,BLUE', '#MODE,AUTO'):
                requested = cmd[6:]
                if requested == 'AUTO':
                    mode = 'AUTO'
                    need_color = state != 'QR'
                else:
                    mode = requested
                    need_color = requested == 'BLUE'
                    edge_near, edge_clear, edge_latched = 0, 0, False
                if need_color != previous_color:
                    set_camera(cam, need_color)
                    previous_color = need_color
                link.send('#MODE_OK,%s\n' % mode)
                continue
            if mode != 'AUTO':
                continue  # Do not progress cargo handling during auxiliary mode.
            if cmd == 'K' and state in ('WAIT_OBJ_ACK', 'WAIT_RING_ACK'):
                print('ACK K:', g_index+1, item_index+1)
                next_item()
            elif cmd == 'B' and state in ('WAIT_OBJ_ACK', 'WAIT_RING_ACK'):
                print('ACK B: retry item', g_index+1, item_index+1)
                history, stable_count = [], 0
                state = 'MATERIAL' if state == 'WAIT_OBJ_ACK' else 'RING'
                last_packet = ''

        img = cam.snapshot()
        now = time.ticks_ms()

        # ----- Auxiliary edge mode (only when commanded) -----
        if mode == 'EDGE':
            line = U.detect_edge(img)
            img.draw_line((0, C.EDGE_STOP_ROW, img.width()-1,
                           C.EDGE_STOP_ROW), color=128)
            if line is not None:
                x1,y1,x2,y2,row = line
                img.draw_line((x1,y1,x2,y2), color=255, thickness=2)
                if time.ticks_diff(now,last_aux_ms) >= 250:
                    link.send('#EDGE,%d,%d,%d,%d,%d\n' % line)
                    last_aux_ms = now
            if line is not None and line[4] >= C.EDGE_STOP_ROW:
                edge_near += 1
                edge_clear = 0
            else:
                edge_near = 0
                edge_clear += 1
            if edge_near >= C.EDGE_CONFIRM and not edge_latched:
                link.send('#STOP\n')  # STM32 must implement this action!
                edge_latched = True
            if edge_clear >= C.EDGE_RELEASE:
                edge_latched = False

        # ----- Auxiliary blue outline mode -----
        elif mode == 'BLUE':
            b = U.detect_blue(img)
            if b:
                x,y,w,h,cx,cy,kind = b
                img.draw_rectangle((x,y,w,h), color=(255,255,0), thickness=2)
                img.draw_cross((cx,cy), color=(255,0,0), size=9)
                if time.ticks_diff(now,last_aux_ms) >= 250:
                    link.send('#BLUE,%d,%d,%d,%d,%d,%d,%d\n' % b)
                    last_aux_ms = now

        # ----- QR scan is unchanged in its core acceptance/LCD behavior -----
        elif state == 'QR':
            seen = None
            for qr in img.find_qrcodes():
                raw = U.normalize_code(U.val(qr,'payload',''))
                if U.parse_code(raw) is None:
                    continue
                seen = raw
                qr_box = tuple(U.val(qr,'rect',(0,0,0,0)))
                qr_box_time = now
                break
            if seen is None:
                last_qr, qr_count = '', 0
            elif seen == last_qr:
                qr_count += 1
            else:
                last_qr, qr_count = seen, 1
            if seen and qr_count >= C.QR_CONFIRM_FRAMES and board.accept(seen):
                print('QR CONFIRMED:', board.code())
                lcd.update(board)
                link.sequence(board)
                g_index, item_index = 0, 0
                state = 'MATERIAL'
                history, stable_count = [], 0
                rings.reset()
                qr_box = None  # Old grayscale frame must not be drawn after reconfigure.
                set_camera(cam, True)
                previous_color = True
                print('GROUP 1 MATERIAL')
            if qr_box is not None and time.ticks_diff(now,qr_box_time) <= C.QR_RECT_HOLD_MS:
                img.draw_rectangle(qr_box, color=255, thickness=2)

        # ----- Material: choose requested color, wait for measured stability -----
        elif state == 'MATERIAL':
            label = int(board.groups[g_index][item_index])
            b = U.material(img, label)
            if b is None:
                history, stable_count = [], 0
            else:
                x,y,w,h = U.val(b,'rect',(0,0,0,0))
                cx,cy = U.val(b,'cx'), U.val(b,'cy')
                history.append((w,cx,cy))
                if len(history) > C.MATERIAL_STILL_FRAMES:
                    history.pop(0)
                stable_count = stable_count+1 if U.material_still(history) else 0
                img.draw_rectangle((x,y,w,h), color=(0,255,0), thickness=2)
                img.draw_cross((cx,cy), color=(255,0,0), size=10)
                img.draw_string((max(0,x),max(0,y-16)), 'M%d' % label,
                                color=(255,255,255), scale=2)
                if stable_count >= C.MATERIAL_STABLE_FRAMES:
                    d = U.material_distance(label,w)
                    last_packet = '#OBJ,%d,%d,%d,%d,%d,%d,%d\n' % (
                        label,cx,cy,w,h,U.val(b,'pixels'),d)
                    link.send(last_packet)
                    state = 'WAIT_OBJ_ACK'
                    last_send_ms, resend_count = now, 0

        # ----- Numbered black rings: OCR labels, not color labels -----
        elif state == 'RING':
            label = int(board.groups[g_index][item_index])
            rings.process(img)
            rings.draw(img)
            track = rings.confirmed(label)
            if track is not None:
                cx, cy = int(round(track.x.x)), int(round(track.y.x))
                r = int(round(track.r.x))
                d = U.ring_distance(track)
                quality = min(100, int(round(track.score)))
                last_packet = '#RING,%d,%d,%d,%d,%d,%d\n' % (
                    label,cx,cy,r,d,quality)
                link.send(last_packet)
                state = 'WAIT_RING_ACK'
                last_send_ms, resend_count = now, 0

        # ----- Wait ACK: no new target means no duplicate reports -----
        elif state in ('WAIT_OBJ_ACK', 'WAIT_RING_ACK'):
            img.draw_string((4,4), 'WAIT STM32', color=(255,255,0), scale=2)
            if time.ticks_diff(now,last_send_ms) >= C.ACK_TIMEOUT_MS:
                if last_packet and resend_count < C.MAX_RESENDS:
                    link.send(last_packet)
                    resend_count += 1
                    last_send_ms = now
                elif resend_count == C.MAX_RESENDS:
                    print('ACK TIMEOUT; waiting for K/B/R or #RESET')
                    resend_count += 1  # Print once; do not silently advance.

        elif state == 'DONE':
            img.draw_string((4,4), 'FLOW DONE', color=(0,255,0), scale=2)

        if time.ticks_diff(now,last_status_ms) >= C.STATUS_PERIOD_MS:
            print('STATE=%s MODE=%s GROUP=%d ITEM=%d LCD=%s' % (
                state,mode,g_index+1,item_index+1,
                board.remaining_text() if board.locked() else 'SCAN QR'))
            last_status_ms = now
        if frame_no % 30 == 0:
            gc.collect()


if __name__ == '__main__':
    main()
