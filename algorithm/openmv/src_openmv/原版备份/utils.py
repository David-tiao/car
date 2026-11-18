# utils.py - OpenMV 5.x helpers: QR, LCD, UART, materials, rings, edge and blue.
import image
import display
import math
import time
from machine import UART
import config as C


def val(obj, name, default=0):
    v = getattr(obj, name, None)
    return default if v is None else (v() if callable(v) else v)


def normalize_code(raw):
    if isinstance(raw, bytes):
        try:
            raw = raw.decode('ascii')
        except Exception:
            return None
    if not isinstance(raw, str):
        return None
    return raw.strip().replace(' ', '').replace('\r', '').replace('\n', '')


def parse_code(raw):
    code = normalize_code(raw)
    if code is None:
        return None
    groups = code.split('+')
    if len(groups) != 4:
        return None
    for index, group in enumerate(groups):
        if len(group) != 3 or not group.isdigit():
            return None
        chars = '123456' if index % 2 == 0 else '123'
        if any(c not in chars for c in group) or len(set(group)) != 3:
            return None
    return groups


class TaskBoard:
    def __init__(self):
        self.groups = None
        self.completed = 0

    def locked(self):
        return self.groups is not None

    def code(self):
        return '+'.join(self.groups) if self.locked() else ''

    def remaining(self):
        return self.groups[self.completed:] if self.locked() else []

    def remaining_text(self):
        return '+'.join(self.remaining())

    def accept(self, raw):
        if self.locked():
            return False
        groups = parse_code(raw)
        if groups is None:
            return False
        self.groups = groups
        self.completed = 0
        return True

    def finish_group(self, num):
        if not self.locked() or num != self.completed + 1 or num > 4:
            return False
        self.completed = num
        return True

    def reset(self):
        self.groups = None
        self.completed = 0


class TaskLCD:
    # SAME LCD configuration that passed standalone and QR demo testing.
    def __init__(self):
        self.lcd = display.SPIDisplay(width=C.LCD_WIDTH, height=C.LCD_HEIGHT)
        self.lcd.backlight(C.LCD_BACKLIGHT)
        self.canvas = image.Image(C.LCD_WIDTH, C.LCD_HEIGHT, image.RGB565)
        self.signature = None
        print('LCD INIT OK')

    def update(self, board):
        signature = (board.code(), board.completed)
        if signature == self.signature:
            return
        self.canvas.clear()
        if not board.locked():
            self.canvas.draw_string((4, 55), 'SCAN QR',
                                    color=(255, 255, 255), scale=2)
        elif board.completed >= 4:
            self.canvas.draw_string((14, 45), 'DONE',
                                    color=(0, 255, 0), scale=3)
            self.canvas.draw_string((30, 95), '4/4',
                                    color=(255, 255, 255), scale=2)
        else:
            parts = board.remaining()
            for i, part in enumerate(parts):
                line = part + ('+' if i < len(parts) - 1 else '')
                self.canvas.draw_string((5, 6 + 38 * i), line,
                                        color=(255, 255, 255), scale=3)
        self.lcd.write(self.canvas)
        self.signature = signature
        print('LCD REFRESH:', board.remaining_text() or
              ('DONE' if board.locked() else 'SCAN QR'))


class VisionUART:
    # Nonblocking: legacy single-byte K/B/R; framed #...\n commands.
    def __init__(self):
        self.uart = UART(C.UART_ID, C.UART_BAUD, timeout=0)
        self.rx = bytearray()
        print('UART%d READY %d' % (C.UART_ID, C.UART_BAUD))

    def send(self, msg):
        self.uart.write(msg.encode('ascii'))
        print('TX:', msg.strip())

    def sequence(self, board):
        self.send('#SEQ,' + ','.join(ch for g in board.groups for ch in g) + '\n')

    def poll(self):
        out = []
        n = self.uart.any()
        if not n:
            return out
        data = self.uart.read(n)
        if not data:
            return out
        for b in data:
            if not self.rx and b in (75, 66, 82):  # K/B/R as single bytes
                out.append(chr(b))
            elif b in (10, 13):
                if self.rx:
                    try:
                        out.append(self.rx.decode('ascii').strip())
                    except Exception:
                        pass
                    self.rx = bytearray()
            elif len(self.rx) < 80:
                self.rx.append(b)
            else:
                self.rx = bytearray()
        return [e for e in out if e]


# Material detection: same LAB thresholds and selection rule as demo_material.
def material(img, label):
    thr = C.MATERIAL_THRESHOLDS.get(label)
    if thr is None:
        return None
    blobs = img.find_blobs([thr], pixels_threshold=C.MATERIAL_MIN_PIXELS,
                           area_threshold=C.MATERIAL_MIN_PIXELS, merge=True)
    best = None
    for b in blobs:
        w, h = val(b, 'w'), val(b, 'h')
        if w <= 0 or h <= 0:
            continue
        if val(b, 'pixels') / float(w * h) < C.MATERIAL_MIN_DENSITY:
            continue
        if best is None or val(b, 'pixels') > val(best, 'pixels'):
            best = b
    return best


def material_still(history):
    if len(history) < C.MATERIAL_STILL_FRAMES:
        return False
    sizes = [s[0] for s in history]
    xs = [s[1] for s in history]
    ys = [s[2] for s in history]
    smallest = min(sizes)
    return (smallest > 0 and
            100.0 * (max(sizes) - smallest) / smallest <= C.MATERIAL_SIZE_TOL_PCT
            and max(xs) - min(xs) <= C.MATERIAL_POS_TOL_PX
            and max(ys) - min(ys) <= C.MATERIAL_POS_TOL_PX)


def material_distance(label, pixels):
    if not C.DISTANCE_CALIBRATED or C.FOCAL_PX <= 0 or pixels <= 0:
        return 0
    return int(round(C.FOCAL_PX * C.MATERIAL_DIAMETER_MM[label] / pixels))


# Black ring candidates use connected dark shapes plus radial multi-peak checks.
N = 24
UNIT_CIRCLE = [(math.cos(2*math.pi*i/N), math.sin(2*math.pi*i/N))
               for i in range(N)]


def black_threshold(gray):
    if C.RING_BLACK_THRESHOLD >= 0:
        return C.RING_BLACK_THRESHOLD
    st = gray.get_statistics()
    low = val(st, 'min')
    mid = val(st, 'median')
    return max(45, min(195, int(0.18 * low + 0.82 * mid)))


def radial_candidate(gray, x, y, r0, threshold):
    w, h = gray.width(), gray.height()
    x, y = int(round(x)), int(round(y))
    r0 = max(6.0, float(r0))
    lo = max(6, int(0.43 * r0))
    hi = min(C.RING_MAX_RADIUS, int(1.85*r0) + 3,
             x-2, y-2, w-3-x, h-3-y)
    if hi - lo < 5:
        return None
    peaks = []
    limit = int(N * C.RING_RADIAL_MIN_FRACTION + 0.999)
    for r in range(lo, hi+1):
        dark = 0
        for ca, sa in UNIT_CIRCLE:
            if gray.get_pixel((int(round(x+r*ca)), int(round(y+r*sa)))) < threshold:
                dark += 1
        if dark >= limit:
            peaks.append((r, dark))
    if not peaks:
        return None
    order = sorted(peaks, key=lambda v: v[1], reverse=True)
    distinct = []
    for r, hits in order:
        if all(abs(r - v[0]) >= 3 for v in distinct):
            distinct.append((r, hits))
        if len(distinct) == 6:
            break
    if len(distinct) < 2:
        return None
    rr = sorted(p[0] for p in distinct)
    if rr[-1] - rr[0] < max(3, int(rr[-1] * .12)):
        return None
    outer = max(r for r, dark in peaks)
    if not (C.RING_MIN_RADIUS <= outer <= C.RING_MAX_RADIUS):
        return None
    return {'x': float(x), 'y': float(y), 'r': float(outer),
            'score': len(distinct)*10 + sum(v[1] for v in distinct)/N}


def circle_candidates(gray):
    threshold = black_threshold(gray)
    blobs = gray.find_blobs([(0, threshold)], x_stride=1, y_stride=1,
                             pixels_threshold=25, area_threshold=90, merge=False)
    seeds = []
    for b in blobs:
        x, y, w, h = val(b, 'rect', (0, 0, 0, 0))
        if not (17 <= w <= 2*C.RING_MAX_RADIUS+6 and
                17 <= h <= 2*C.RING_MAX_RADIUS+6):
            continue
        aspect = w / float(h)
        fill = val(b, 'pixels') / float(w*h)
        if .72 <= aspect <= 1.28 and .045 <= fill <= .64:
            seeds.append((x+w/2.0, y+h/2.0, (w+h)/4.0, w*h))
    seeds.sort(key=lambda v: v[3], reverse=True)
    candidates = []
    for x, y, r, area in seeds[:18]:
        cand = radial_candidate(gray, x, y, r, threshold)
        if cand is None:
            continue
        for k, old in enumerate(candidates):
            dx = cand['x'] - old['x']
            dy = cand['y'] - old['y']
            if dx*dx + dy*dy < max(7, .36*max(cand['r'], old['r']))**2:
                if cand['score'] > old['score']:
                    candidates[k] = cand
                break
        else:
            candidates.append(cand)
    candidates.sort(key=lambda v: v['score'], reverse=True)
    selected = []
    for c in candidates:
        if all((c['x']-v['x'])**2+(c['y']-v['y'])**2 >
               (.72*(c['r']+v['r']))**2 for v in selected):
            selected.append(c)
        if len(selected) == 3:
            break
    return selected, threshold


# OCR templates: geometric recognition only, not color. Rotation can be fixed.
FONT = {
    0: ('01110','10001','10011','10101','11001','10001','01110'),
    1: ('00100','01100','00100','00100','00100','00100','01110'),
    2: ('01110','10001','00001','00010','00100','01000','11111'),
    3: ('11110','00001','00001','01110','00001','00001','11110'),
    4: ('00010','00110','01010','10010','11111','00010','00010'),
    5: ('11111','10000','10000','11110','00001','00001','11110'),
    6: ('00110','01000','10000','11110','10001','10001','01110'),
    7: ('11111','00001','00010','00100','01000','01000','01000'),
    8: ('01110','10001','10001','01110','10001','10001','01110'),
    9: ('01110','10001','10001','01111','00001','00010','11100')
}
FONT_EXTRA = {
    1: (('00010','00010','00010','00010','11111','11111','11111'),),
    2: (('01110','11011','00001','00011','01100','11000','11111'),),
    3: (('01110','10001','00011','00110','00001','10001','01110'),)
}


def text_bitmap(gray, box, threshold, rotation):
    x, y, w, h = box
    rows = []
    for j in range(7):
        row = ''
        for i in range(5):
            dark = 0
            for oy in (.25, .75):
                for ox in (.25, .75):
                    u, v = (i+ox)/5.0, (j+oy)/7.0
                    if rotation == 0:
                        a, b = u, v
                    elif rotation == 90:
                        a, b = v, 1-u
                    elif rotation == 180:
                        a, b = 1-u, 1-v
                    else:
                        a, b = 1-v, u
                    px = min(gray.width()-1, int(x+a*w))
                    py = min(gray.height()-1, int(y+b*h))
                    if gray.get_pixel((px, py)) < threshold:
                        dark += 1
            row += '1' if dark >= 2 else '0'
        rows.append(row)
    return rows


def read_ring_digit(gray, x, y, radius):
    x, y = int(round(x)), int(round(y))
    half = int(radius * .50) + 1
    x0, y0 = max(0, x-half), max(0, y-half)
    x1, y1 = min(gray.width(), x+half+1), min(gray.height(), y+half+1)
    w, h = x1-x0, y1-y0
    if w < 15 or h < 15:
        return None
    st = gray.get_statistics(roi=(x0, y0, w, h))
    thr = max(35, min(200, int(.20*val(st, 'min')+.80*val(st, 'median'))))
    blobs = gray.find_blobs([(0, thr)], roi=(x0, y0, w, h),
                             x_stride=1, y_stride=1, pixels_threshold=6,
                             area_threshold=10, merge=False)
    chosen, best_rank = None, -100000
    for b in blobs:
        bx, by, bw, bh = val(b, 'rect', (0, 0, 0, 0))
        if bw < 2 or bh < 6 or bw > .84*w or bh > .94*h:
            continue
        if bx <= x0 or by <= y0 or bx+bw >= x1 or by+bh >= y1:
            continue
        delta = abs(bx+bw/2.0-x) + abs(by+bh/2.0-y)
        if delta > .46*(w+h):
            continue
        rank = val(b, 'pixels')-2.7*delta
        if rank > best_rank:
            chosen, best_rank = (bx, by, bw, bh), rank
    if chosen is None:
        return None
    angles = (C.RING_TEXT_ROTATION,) if C.RING_TEXT_ROTATION in (0,90,180,270) \
             else (0,90,180,270)
    ranking = []
    for rotation in angles:
        bits = text_bitmap(gray, chosen, thr, rotation)
        for digit in range(10):
            variants = (FONT[digit],) + FONT_EXTRA.get(digit, ())
            errors = min(sum(bits[j][i] != font[j][i]
                             for j in range(7) for i in range(5))
                         for font in variants)
            ranking.append((1-errors/35.0, digit))
    ranking.sort(reverse=True)
    best_score, best_digit = ranking[0]
    runner = max((s for s,d in ranking if d != best_digit), default=0.0)
    if best_score < C.RING_OCR_MIN_SCORE or best_score-runner < C.RING_OCR_MIN_GAP:
        return None
    if C.RING_TEXT_ROTATION is None and best_digit in (6,9):
        return None  # ambiguous if orientation is unknown
    return best_digit


class Kalman1D:
    def __init__(self, z, R=8.0, Q=3200.0):
        self.x, self.v = float(z), 0.0
        self.p00, self.p01, self.p11 = 30.0, 0.0, 400.0
        self.R, self.Q = R, Q

    def predict(self, dt):
        dt = max(.001, min(.5, dt))
        a = self.p00 + 2*dt*self.p01 + dt*dt*self.p11
        b = self.p01 + dt*self.p11
        self.p00 = a + self.Q*dt**4/4
        self.p01 = b + self.Q*dt**3/2
        self.p11 += self.Q*dt*dt
        self.x += self.v*dt

    def distance(self, z):
        return (z-self.x)**2 / max(.001, self.p00+self.R)

    def correct(self, z):
        den = self.p00 + self.R
        err = z-self.x
        kx, kv = self.p00/den, self.p01/den
        old01 = self.p01
        self.x += kx*err
        self.v += kv*err
        self.p00 = max(.01, (1-kx)*self.p00)
        self.p01 = (1-kx)*old01
        self.p11 = max(.01, self.p11-kv*old01)


class RingTrack:
    def __init__(self, c, now):
        self.x, self.y = Kalman1D(c['x']), Kalman1D(c['y'])
        self.r = Kalman1D(c['r'], R=6.0, Q=900.0)
        self.hits, self.miss = 1, 0
        self.last_predict, self.last_measured = now, now
        self.votes, self.last_ocr = [], now
        self.score = c['score']
        self.matched = True

    def predict(self, now):
        dt = time.ticks_diff(now, self.last_predict)/1000.0
        for f in (self.x, self.y, self.r):
            f.predict(dt)
        self.last_predict = now

    def gate(self, c):
        dx, dy = c['x']-self.x.x, c['y']-self.y.x
        if dx*dx+dy*dy > max(14, .60*self.r.x)**2:
            return None
        if abs(c['r']-self.r.x) > max(8, .38*self.r.x):
            return None
        d = (self.x.distance(c['x'])+self.y.distance(c['y'])+
             self.r.distance(c['r']))
        return d if d < 16 else None

    def update(self, c, now):
        self.x.correct(c['x']); self.y.correct(c['y']); self.r.correct(c['r'])
        self.score = c['score']
        self.hits += 1
        self.miss = 0
        self.matched = True
        self.last_measured = now

    def voted_digit(self, now):
        if self.hits < C.RING_MIN_HITS or self.miss or \
           time.ticks_diff(now, self.last_measured) > 700 or \
           time.ticks_diff(now, self.last_ocr) > 1200:
            return None
        ranking = sorted(((self.votes.count(i), i) for i in range(10)),
                         reverse=True)
        return ranking[0][1] if (ranking[0][0] >= C.RING_OCR_VOTES and
                                ranking[0][0] - ranking[1][0] >= 2) else None


class RingFinder:
    def __init__(self):
        self.tracks = []
        self.cycle = 0

    def reset(self):
        self.tracks = []
        self.cycle = 0

    def process(self, img):
        self.cycle += 1
        now = time.ticks_ms()
        if self.cycle % C.RING_DETECT_INTERVAL:
            return
        gray = img.to_grayscale(copy=True)
        try:
            candidates, threshold = circle_candidates(gray)
            for t in self.tracks:
                t.predict(now)
                t.matched = False
            pairs = []
            for i, t in enumerate(self.tracks):
                for j, c in enumerate(candidates):
                    distance = t.gate(c)
                    if distance is not None:
                        pairs.append((distance, i, j))
            pairs.sort()
            used_t, used_c = set(), set()
            for dist, i, j in pairs:
                if i not in used_t and j not in used_c:
                    self.tracks[i].update(candidates[j], now)
                    used_t.add(i); used_c.add(j)
            for i, t in enumerate(self.tracks):
                if i not in used_t:
                    t.miss += 1
                    t.hits = 0
                    t.votes = []
            self.tracks = [t for t in self.tracks if t.miss <= 2]
            for j, c in enumerate(candidates):
                if j not in used_c and len(self.tracks) < 3:
                    self.tracks.append(RingTrack(c, now))
            if (self.cycle // C.RING_DETECT_INTERVAL) % 2 == 0:
                for t in self.tracks:
                    if t.matched and t.hits >= 2:
                        digit = read_ring_digit(gray, t.x.x, t.y.x, t.r.x)
                        t.votes.append(digit)
                        if len(t.votes) > C.RING_OCR_WINDOW:
                            t.votes.pop(0)
                        if digit is not None:
                            t.last_ocr = now
        finally:
            del gray

    def confirmed(self, label):
        now = time.ticks_ms()
        matches = [t for t in self.tracks if t.voted_digit(now) == label]
        return matches[0] if len(matches) == 1 else None

    def draw(self, img):
        now = time.ticks_ms()
        for t in self.tracks:
            if t.hits >= C.RING_MIN_HITS and t.miss == 0 and \
               time.ticks_diff(now, t.last_measured) < 700:
                cx, cy = int(round(t.x.x)), int(round(t.y.x))
                r = max(1, int(round(t.r.x)))
                img.draw_cross((cx, cy), color=(255,0,0), size=10, thickness=2)
                img.draw_circle((cx, cy, r), color=(255,0,0), thickness=1)
                d = t.voted_digit(now)
                if d is not None:
                    img.draw_string((max(0,cx-5), max(0,cy-r-15)),
                                    str(d), color=(255,0,0), scale=2)


def ring_distance(t):
    if not C.DISTANCE_CALIBRATED or C.FOCAL_PX <= 0 or \
       C.RING_DIAMETER_MM <= 0 or t.r.x <= 0:
        return 0
    z = C.FOCAL_PX*C.RING_DIAMETER_MM/(2*t.r.x)
    dx = (t.x.x-C.PRINCIPAL_X_PX)/C.FOCAL_PX
    dy = (t.y.x-C.PRINCIPAL_Y_PX)/C.FOCAL_PX
    return int(round(z*math.sqrt(1+dx*dx+dy*dy)))


# Auxiliary blue-outline mode; cropped/corner regions remain valid observations.
def detect_blue(img):
    blobs = img.find_blobs([C.BLUE_THRESHOLD],
                           pixels_threshold=C.BLUE_MIN_PIXELS,
                           area_threshold=C.BLUE_MIN_PIXELS, merge=True)
    best = None
    for b in blobs:
        w, h = val(b,'w'), val(b,'h')
        if w < 3 or h < 3 or max(w,h) < 20:
            continue
        pixels = val(b,'pixels')
        if pixels/float(w*h) < C.BLUE_MIN_DENSITY:
            continue
        if best is None or pixels > val(best,'pixels'):
            best = b
    if best is None:
        return None
    x, y, w, h = val(best, 'rect')
    mode = 1 if w >= 3*h else (2 if h >= 3*w else 3)
    return (x,y,w,h,val(best,'cx'),val(best,'cy'),mode)


# Auxiliary black line: validate a THIN dark stripe on brighter background.
# STOP_ROW is an image-row trigger; does not pretend to be metric distance.
def _dark_stripe(img, x, y):
    x, y = int(x), int(y)
    if x < 0 or x >= img.width() or y < 10 or y >= img.height()-10:
        return False
    dark = min(img.get_pixel((x, y+d)) for d in (-2,-1,0,1,2))
    top = img.get_pixel((x,y-7))
    bottom = img.get_pixel((x,y+7))
    if dark > C.EDGE_DARK_MAX or top-dark < C.EDGE_CONTRAST or \
       bottom-dark < C.EDGE_CONTRAST:
        return False
    level = min(top,bottom)-C.EDGE_CONTRAST//2
    width = sum(img.get_pixel((x,y+d)) <= level for d in range(-6,7))
    return width <= 9


def detect_edge(img):
    w, h = img.width(), img.height()
    top = int(h*.33)
    # Native line candidates from the original edge demo, then dark-stripe tests.
    lines = img.find_lines(roi=(0,top,w,h-top), threshold=600,
                           x_stride=1, y_stride=1, theta_margin=10,
                           rho_margin=8)
    best, quality = None, -1
    for l in lines:
        x1, y1, x2, y2 = (val(l,'x1'),val(l,'y1'),val(l,'x2'),val(l,'y2'))
        dx, dy = x2-x1, y2-y1
        if abs(dx) < .55*w or abs(dy) > .45*abs(dx):
            continue
        hits = 0
        for k in range(1,10):
            xx = k*w//10
            if xx < min(x1,x2) or xx > max(x1,x2):
                continue
            yy = int(round(y1+(xx-x1)*dy/float(dx)))
            if yy >= top+8 and yy < h-8 and _dark_stripe(img,xx,yy):
                hits += 1
        if hits >= 6 and hits*1000+val(l,'magnitude') > quality:
            quality = hits*1000+val(l,'magnitude')
            row = int(round(y1+(w//2-x1)*dy/float(dx)))
            best = (x1,y1,x2,y2,row)
    if best is not None:
        return best

    # Original demo's second strategy: scan several columns and fit
    # a nearly horizontal thin stripe when Hough does not return a line.
    columns = []
    for x in range(w//2-60, w//2+61, 12):
        points = []
        for y in range(max(top+10, 10), h-10):
            if _dark_stripe(img, x, y):
                points.append(y)
        columns.append((x, points))
    best_support = 0
    best = None
    for seed_y in range(top+10, h-10, 3):
        matches = []
        for x, rows in columns:
            nearby = [yy for yy in rows if abs(yy-seed_y) <= 6]
            if nearby:
                yy = min(nearby, key=lambda z:abs(z-seed_y))
                matches.append((x, yy))
        if len(matches) < 8 or matches[-1][0]-matches[0][0] < 85:
            continue
        n = len(matches)
        mean_x = sum(x for x,y in matches)/float(n)
        mean_y = sum(y for x,y in matches)/float(n)
        denom = sum((x-mean_x)**2 for x,y in matches)
        if denom <= 0:
            continue
        slope = sum((x-mean_x)*(y-mean_y) for x,y in matches)/denom
        if abs(slope) > .22:
            continue
        if max(abs(y-mean_y-slope*(x-mean_x)) for x,y in matches) > 3:
            continue
        if n > best_support:
            x1, x2 = matches[0][0], matches[-1][0]
            y1 = int(round(mean_y+slope*(x1-mean_x)))
            y2 = int(round(mean_y+slope*(x2-mean_x)))
            best = (x1,y1,x2,y2,int(round(mean_y+slope*(w/2-mean_x))))
            best_support = n
    return best
