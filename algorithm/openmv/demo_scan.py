
# demo_scan.py
# OpenMV H7 Plus / Firmware 5.x
#
# QR scanning + LCD task progress
#
# QR example:
# 156+123+516+231
#
# STM32 -> OpenMV:
# #GROUP_DONE,1\n
# #GROUP_DONE,2\n
# #GROUP_DONE,3\n
# #GROUP_DONE,4\n
# #RESET\n

import csi
import image
import display
import time
from machine import UART


# ============ Configuration ============

LCD_WIDTH = 128
LCD_HEIGHT = 160
LCD_BACKLIGHT = 100

UART_PORT = 3
UART_BAUD = 115200
ENABLE_UART = True

# Confirm after 3 identical scans
QR_CONFIRM_FRAMES = 3

# Continue tracking the QR after lock
LOCKED_SCAN_DIV = 3
RECT_HOLD_MS = 600

STATUS_INTERVAL_MS = 2000


# ============ Basic helpers ============

def field(obj, name, default=None):

    value = getattr(obj, name, None)

    if value is None:
        return default

    return value() if callable(value) else value


def normalize_code(raw):

    if isinstance(raw, bytes):

        try:
            raw = raw.decode('ascii')
        except Exception:
            return None

    if not isinstance(raw, str):
        return None

    return (
        raw.strip()
        .replace(' ', '')
        .replace('\r', '')
        .replace('\n', '')
    )


# ============ QR task validation ============

def parse_code(raw):

    code = normalize_code(raw)

    if code is None:
        return None

    groups = code.split('+')

    if len(groups) != 4:
        return None

    for i, group in enumerate(groups):

        if len(group) != 3 or not group.isdigit():
            return None

        # Group 1/3: material numbers 1~6
        # Group 2/4: ring positions 1~3
        alphabet = (
            '123456' if i % 2 == 0
            else '123'
        )

        if any(d not in alphabet for d in group):
            return None

        if len(set(group)) != 3:
            return None

    return groups


# ============ Task management ============

class TaskBoard:

    def __init__(self):

        self.groups = None
        self.completed = 0

    def locked(self):

        return self.groups is not None

    def code(self):

        if not self.locked():
            return ''

        return '+'.join(self.groups)

    def remaining(self):

        if not self.locked():
            return []

        return self.groups[self.completed:]

    def remaining_text(self):

        return '+'.join(self.remaining())

    def accept(self, raw):

        # Never overwrite a task in progress.
        if self.locked():
            return False

        groups = parse_code(raw)

        if groups is None:
            return False

        self.groups = groups
        self.completed = 0

        return True

    def finish_group(self, group_num):

        if not self.locked():
            return False

        if self.completed >= 4:
            return False

        # Enforce 1 -> 2 -> 3 -> 4
        if group_num != self.completed + 1:
            return False

        self.completed += 1

        return True

    def reset(self):

        self.groups = None
        self.completed = 0


# ============ LCD display ============

class TaskLCD:

    def __init__(self):

        # Same initialization as the working LCD test
        self.lcd = display.SPIDisplay(
            width=LCD_WIDTH,
            height=LCD_HEIGHT
        )

        # Enable the backlight explicitly
        self.lcd.backlight(LCD_BACKLIGHT)

        self.canvas = image.Image(
            LCD_WIDTH,
            LCD_HEIGHT,
            image.RGB565
        )

        print('LCD INIT OK')

    def update(self, board):

        # Clear the old text
        self.canvas.clear()

        if not board.locked():

            self.canvas.draw_string(
                (4, 55),
                'SCAN QR',
                color=(255, 255, 255),
                scale=2
            )

        elif board.completed == 4:

            # All groups completed
            self.canvas.draw_string(
                (14, 45),
                'DONE',
                color=(0, 255, 0),
                scale=3
            )

            self.canvas.draw_string(
                (30, 95),
                '4/4',
                color=(255, 255, 255),
                scale=2
            )

        else:

            # Remaining task groups
            remaining = board.remaining()

            for i, group in enumerate(remaining):

                # Include '+' only if there is
                # another group after this one
                line = group

                if i < len(remaining) - 1:
                    line += '+'

                self.canvas.draw_string(
                    (5, 6 + i * 38),
                    line,
                    color=(255, 255, 255),
                    scale=3
                )

        # Send the completed image to LCD
        self.lcd.write(self.canvas)

        print(
            'LCD REFRESH:',
            board.remaining_text()
            or (
                'DONE' if board.locked()
                else 'SCAN QR'
            )
        )


# ============ UART communication ============

class TaskSerial:

    def __init__(self):

        self.uart = None
        self.rx = bytearray()

        if ENABLE_UART:

            self.uart = UART(
                UART_PORT,
                UART_BAUD,
                timeout=0
            )

            print(
                'UART%d READY %d' % (
                    UART_PORT,
                    UART_BAUD
                )
            )

    def send_sequence(self, board):

        if self.uart is None:
            return

        # Keep the original #SEQ format
        msg = (
            '#SEQ,'
            + ','.join(
                ch
                for g in board.groups
                for ch in g
            )
            + '\n'
        )

        self.uart.write(
            msg.encode('ascii')
        )

        print('TX:', msg.strip())

    def command(self, line, board):

        line = line.strip()

        # Reset
        if line in ('#RESET', 'R'):

            board.reset()

            print('RX RESET')

            return True

        # Group completion
        if line.startswith('#GROUP_DONE,'):

            num = line[len('#GROUP_DONE,'):]

            if (
                len(num) == 1
                and num in '1234'
                and board.finish_group(int(num))
            ):

                print(
                    'RX GROUP_DONE %s -> %s' % (
                        num,
                        board.remaining_text()
                        or 'DONE'
                    )
                )

                return True

            # Duplicate or out-of-order command
            print('IGNORED:', line)

        return False

    def poll(self, board):

        if self.uart is None:
            return False

        amount = self.uart.any()

        if not amount:
            return False

        data = self.uart.read(amount)

        if not data:
            return False

        # Support an old single-byte reset
        if data == b'R' and not self.rx:

            return self.command('R', board)

        changed = False

        for char in data:

            # End of one command
            if char in (10, 13):

                if self.rx:

                    try:
                        line = self.rx.decode('ascii')
                    except Exception:
                        line = ''

                    self.rx = bytearray()

                    if line:

                        if self.command(line, board):
                            changed = True

            elif len(self.rx) < 64:

                self.rx.append(char)

            else:

                # Discard oversized command
                self.rx = bytearray()

        return changed


# ============ Main program ============

def run():

    # ----- Camera initialization -----

    cam = csi.CSI()

    cam.reset()

    cam.pixformat(csi.GRAYSCALE)
    cam.framesize(csi.QVGA)

    cam.snapshot(time=1500)

    # ----- Modules -----

    board = TaskBoard()

    screen = TaskLCD()
    port = TaskSerial()

    # Display standby status immediately
    screen.update(board)

    # ----- QR confirmation state -----

    last_seen_code = ''
    same_count = 0

    qr_rect = None
    qr_seen_at = 0

    last_status = time.ticks_ms()
    frame = 0

    print('demo_scan.py READY')

    print(
        'STM32 CMD: #GROUP_DONE,1 to 4; #RESET'
    )

    # ============ Main loop ============

    while True:

        img = cam.snapshot()

        now = time.ticks_ms()
        frame += 1

        # ----- 1. Read STM32 commands -----

        if port.poll(board):

            # Refresh after completion or reset
            screen.update(board)

            if not board.locked():

                last_seen_code = ''
                same_count = 0
                qr_rect = None

        # ----- 2. QR scanning -----

        should_scan = (
            not board.locked()
            or frame % LOCKED_SCAN_DIV == 0
        )

        if should_scan:

            found = None

            qrcodes = img.find_qrcodes()

            for qr in qrcodes:

                code = normalize_code(
                    field(qr, 'payload', '')
                )

                if board.locked():

                    # Do not overwrite locked tasks
                    valid = (code == board.code())

                else:

                    valid = (
                        parse_code(code) is not None
                    )

                if valid:

                    rect = field(
                        qr,
                        'rect',
                        None
                    )

                    if rect is not None:

                        qr_rect = tuple(rect)
                        qr_seen_at = now

                    found = code

                    break

            # ----- 3. Confirm QR content -----

            if not board.locked():

                if found is None:

                    last_seen_code = ''
                    same_count = 0

                else:

                    if found == last_seen_code:

                        same_count += 1

                    else:

                        last_seen_code = found
                        same_count = 1

                    if same_count >= QR_CONFIRM_FRAMES:

                        if board.accept(found):

                            print(
                                'QR CONFIRMED:',
                                board.code()
                            )

                            # LCD: show all task groups
                            screen.update(board)

                            # Send the QR sequence to STM32
                            port.send_sequence(board)

        # ----- 4. QR rectangle for IDE -----

        if qr_rect is not None:

            if time.ticks_diff(
                now,
                qr_seen_at
            ) <= RECT_HOLD_MS:

                img.draw_rectangle(
                    qr_rect,
                    color=255,
                    thickness=2
                )

            else:

                qr_rect = None

        # ----- 5. Status output -----

        if time.ticks_diff(
            now,
            last_status
        ) >= STATUS_INTERVAL_MS:

            if board.locked():

                print(
                    'PROGRESS %d/4: %s' % (
                        board.completed,
                        board.remaining_text()
                        or 'DONE'
                    )
                )

            else:

                print('WAITING FOR QR')

            last_status = now


if __name__ == '__main__':
    run()

