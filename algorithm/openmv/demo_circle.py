
# demo_circle.py
# OpenMV H7 Plus / Firmware 5.x
# Black concentric rings + Kalman + OCR 0~9
# LCD outputs confirmed results only.

import csi
import image
import time
import math
import gc

# ============ Configuration ============

DISPLAY = True
DEBUG = True

W = 320
H = 240
MAX_TARGETS = 3

DETECT_EVERY = 2
HOUGH_EVERY = 12

R_MIN = 9
R_MAX = 65

HOUGH_THRESHOLD = 1100
BLACK_THR_OVERRIDE = -1

SAMPLES = 32
RING_FRAC = 0.41

CONFIRM_FRAMES = 3
LOST_LIMIT = 2

OCR_EVERY = 2
OCR_MIN_SCORE = 0.74
OCR_SCORE_GAP = 0.05
OCR_VOTES = 3
OCR_WINDOW = 5

FIXED_TEXT_ROTATION = None
VALID_DIGITS = tuple(range(10))

LCD_PERIOD_MS = 300
STATUS_PERIOD_MS = 1000

# Physical distance calibration
RING_DIAMETER_MM = 0.0
FOCAL_PX = 0.0
DISTANCE_CALIBRATED = False

PRINCIPAL_X = 160.0
PRINCIPAL_Y = 120.0

# ============ OCR templates ============

FONT = {
    0: (
        "01110", "10001", "10011",
        "10101", "11001", "10001", "01110"
    ),
    1: (
        "00100", "01100", "00100",
        "00100", "00100", "00100", "01110"
    ),
    2: (
        "01110", "10001", "00001",
        "00010", "00100", "01000", "11111"
    ),
    3: (
        "11110", "00001", "00001",
        "01110", "00001", "00001", "11110"
    ),
    4: (
        "00010", "00110", "01010",
        "10010", "11111", "00010", "00010"
    ),
    5: (
        "11111", "10000", "10000",
        "11110", "00001", "00001", "11110"
    ),
    6: (
        "00110", "01000", "10000",
        "11110", "10001", "10001", "01110"
    ),
    7: (
        "11111", "00001", "00010",
        "00100", "01000", "01000", "01000"
    ),
    8: (
        "01110", "10001", "10001",
        "01110", "10001", "10001", "01110"
    ),
    9: (
        "01110", "10001", "10001",
        "01111", "00001", "00010", "11100"
    )
}

# Additional templates derived from your printed targets.
FONT_EXTRA = {
    1: ((
        "00010", "00010", "00010",
        "00010", "11111", "11111", "11111"
    ),),
    2: ((
        "01110", "11011", "00001",
        "00011", "01100", "11000", "11111"
    ),),
    3: ((
        "01110", "10001", "00011",
        "00110", "00001", "10001", "01110"
    ),)
}

ANGLES = [
    (
        math.cos(i * 2.0 * math.pi / SAMPLES),
        math.sin(i * 2.0 * math.pi / SAMPLES)
    )
    for i in range(SAMPLES)
]


def attr(obj, key):
    v = getattr(obj, key)
    return v() if callable(v) else v


# ============ Black segmentation ============

def auto_black_threshold(gray):
    if BLACK_THR_OVERRIDE >= 0:
        return BLACK_THR_OVERRIDE

    st = gray.get_statistics()
    lo = attr(st, "min")
    med = attr(st, "median")

    return max(
        45,
        min(195, int(lo * 0.18 + med * 0.82))
    )


# ============ Radial circle validation ============

def ring_test(gray, x, y, initial_r, threshold):

    x = int(round(x))
    y = int(round(y))

    low_r = max(6, int(initial_r * 0.43))

    high_r = min(
        R_MAX,
        int(initial_r * 1.85) + 3,
        x - 2,
        y - 2,
        W - 3 - x,
        H - 3 - y
    )

    if high_r - low_r < 5:
        return None

    results = []

    for r in range(low_r, high_r + 1):

        dark = 0

        for ca, sa in ANGLES:

            xx = int(round(x + r * ca))
            yy = int(round(y + r * sa))

            if gray.get_pixel((xx, yy)) < threshold:
                dark += 1

        if dark >= int(SAMPLES * RING_FRAC + 0.999):
            results.append((r, dark))

    if not results:
        return None

    results.sort(
        key=lambda e: e[1],
        reverse=True
    )

    peaks = []

    for r, dark in results:

        if all(abs(r - p[0]) >= 3 for p in peaks):
            peaks.append((r, dark))

        if len(peaks) >= 6:
            break

    if len(peaks) < 2:
        return None

    radii = sorted(p[0] for p in peaks)

    if radii[-1] - radii[0] < max(
        3,
        radii[-1] * 0.12
    ):
        return None

    outer = max(
        r for r, count in results
        if count >= int(SAMPLES * RING_FRAC + 0.999)
    )

    if outer < R_MIN or outer > R_MAX:
        return None

    quality = (
        len(peaks) * 10.0
        + sum(p[1] for p in peaks) / SAMPLES
    )

    return {
        "x": float(x),
        "y": float(y),
        "r": float(outer),
        "score": quality
    }


def add_candidate(candidates, candidate):

    if candidate is None:
        return

    for k, old in enumerate(candidates):

        dx = old["x"] - candidate["x"]
        dy = old["y"] - candidate["y"]

        tol = max(
            7.0,
            0.36 * max(old["r"], candidate["r"])
        )

        if dx * dx + dy * dy < tol * tol:

            if candidate["score"] > old["score"]:
                candidates[k] = candidate

            return

    candidates.append(candidate)


# ============ Three ring detection ============

def detect_targets(gray, threshold, cycle):

    # Native blob detector operates in C.
    blobs = gray.find_blobs(
        [(0, threshold)],
        x_stride=1,
        y_stride=1,
        pixels_threshold=25,
        area_threshold=90,
        merge=False
    )

    seeds = []

    for b in blobs:

        x = attr(b, "x")
        y = attr(b, "y")
        w = attr(b, "w")
        h = attr(b, "h")

        if not (
            17 <= w <= 2 * R_MAX + 6
            and 17 <= h <= 2 * R_MAX + 6
        ):
            continue

        ratio = w / float(h)
        fill = attr(b, "pixels") / float(w * h)

        if not (
            0.72 <= ratio <= 1.28
            and 0.045 <= fill <= 0.64
        ):
            continue

        radius = (w + h) * 0.25

        seeds.append((
            x + w * 0.5,
            y + h * 0.5,
            radius,
            w * h
        ))

    seeds.sort(
        key=lambda v: v[3],
        reverse=True
    )

    candidates = []

    # Test up to 18 strongest shape candidates.
    for x, y, r, area in seeds[:18]:

        c = ring_test(
            gray,
            x,
            y,
            r,
            threshold
        )

        add_candidate(candidates, c)

    # Occasional Hough fallback, not every frame.
    if (
        cycle % HOUGH_EVERY == 0
        and len(candidates) < MAX_TARGETS
    ):

        try:

            circles = gray.find_circles(
                threshold=HOUGH_THRESHOLD,
                x_stride=2,
                y_stride=2,
                x_margin=7,
                y_margin=7,
                r_margin=2,
                r_min=R_MIN,
                r_max=R_MAX,
                r_step=2
            )

            for circle in circles[:20]:

                c = ring_test(
                    gray,
                    attr(circle, "x"),
                    attr(circle, "y"),
                    attr(circle, "r"),
                    threshold
                )

                add_candidate(candidates, c)

        except Exception as err:

            if DEBUG:
                print("HOUGH_ERR", err)

    candidates.sort(
        key=lambda c: c["score"],
        reverse=True
    )

    selected = []

    for c in candidates:

        separate = all(
            (
                (c["x"] - q["x"]) ** 2
                + (c["y"] - q["y"]) ** 2
            ) > (
                0.72 * (c["r"] + q["r"])
            ) ** 2
            for q in selected
        )

        if separate:
            selected.append(c)

        if len(selected) >= MAX_TARGETS:
            break

    return selected, len(blobs)


# ============ Kalman filter ============

class KF:

    def __init__(self, z, R=8.0, Q=3200.0):

        self.x = float(z)
        self.v = 0.0

        self.p00 = 30.0
        self.p01 = 0.0
        self.p11 = 400.0

        self.R = R
        self.Q = Q

    def predict(self, dt):

        dt = min(0.5, max(0.001, dt))

        a = (
            self.p00
            + 2 * dt * self.p01
            + dt * dt * self.p11
        )

        b = self.p01 + dt * self.p11

        self.p00 = (
            a + 0.25 * self.Q * dt ** 4
        )

        self.p01 = (
            b + 0.5 * self.Q * dt ** 3
        )

        self.p11 += self.Q * dt * dt

        self.x += self.v * dt

    def distance(self, z):

        e = z - self.x

        return (
            e * e
            / max(0.001, self.p00 + self.R)
        )

    def correct(self, z):

        den = self.p00 + self.R
        e = z - self.x

        kx = self.p00 / den
        kv = self.p01 / den

        old01 = self.p01

        self.x += kx * e
        self.v += kv * e

        self.p00 = max(
            0.01,
            (1 - kx) * self.p00
        )

        self.p01 = (1 - kx) * old01

        self.p11 = max(
            0.01,
            self.p11 - kv * old01
        )


# ============ Multi-target tracking ============

class Track:

    def __init__(self, c, now):

        self.x = KF(c["x"])
        self.y = KF(c["y"])

        self.r = KF(
            c["r"],
            R=6.0,
            Q=900.0
        )

        self.hits = 1
        self.miss = 0

        self.stamp = now
        self.last_measured = now

        self.votes = []
        self.last_ocr = now

    def predict(self, now):

        dt = time.ticks_diff(
            now,
            self.stamp
        ) / 1000.0

        for filt in (self.x, self.y, self.r):
            filt.predict(dt)

        self.stamp = now

    def gate(self, c):

        dx = c["x"] - self.x.x
        dy = c["y"] - self.y.x

        maximum = max(
            14.0,
            self.r.x * 0.60
        )

        if dx * dx + dy * dy > maximum ** 2:
            return None

        if abs(c["r"] - self.r.x) > max(
            8.0,
            self.r.x * 0.38
        ):
            return None

        score = (
            self.x.distance(c["x"])
            + self.y.distance(c["y"])
            + self.r.distance(c["r"])
        )

        return score if score < 16.0 else None

    def update(self, c, now):

        self.x.correct(c["x"])
        self.y.correct(c["y"])
        self.r.correct(c["r"])

        self.hits += 1
        self.miss = 0

        self.last_measured = now

    def add_ocr(self, digit, now):

        self.votes.append(digit)

        if len(self.votes) > OCR_WINDOW:
            self.votes.pop(0)

        if digit is not None:
            self.last_ocr = now

    def confirmed_center(self, now):

        return (
            self.hits >= CONFIRM_FRAMES
            and self.miss == 0
            and time.ticks_diff(
                now,
                self.last_measured
            ) < 700
        )

    def confirmed_digit(self, now):

        if not self.confirmed_center(now):
            return None

        if time.ticks_diff(
            now,
            self.last_ocr
        ) > 1200:
            return None

        ranks = sorted(
            [
                (self.votes.count(d), d)
                for d in VALID_DIGITS
            ],
            reverse=True
        )

        if (
            ranks[0][0] >= OCR_VOTES
            and ranks[0][0] - ranks[1][0] >= 2
        ):
            return ranks[0][1]

        return None


def update_tracks(tracks, candidates, now):

    for t in tracks:
        t.predict(now)

    pairs = []

    for i, t in enumerate(tracks):

        for j, c in enumerate(candidates):

            score = t.gate(c)

            if score is not None:
                pairs.append((score, i, j))

    pairs.sort()

    used_t = set()
    used_c = set()
    matched = []

    for score, i, j in pairs:

        if i in used_t or j in used_c:
            continue

        tracks[i].update(candidates[j], now)

        used_t.add(i)
        used_c.add(j)

        matched.append(tracks[i])

    for i, t in enumerate(tracks):

        if i not in used_t:
            t.miss += 1
            t.hits = 0
            t.votes = []

    tracks[:] = [
        t for t in tracks
        if t.miss <= LOST_LIMIT
    ]

    for j, c in enumerate(candidates):

        if (
            j not in used_c
            and len(tracks) < MAX_TARGETS
        ):
            t = Track(c, now)
            tracks.append(t)
            matched.append(t)

    return matched


# ============ Rotation-aware OCR ============

def bitmap_from_blob(
    gray,
    b,
    threshold,
    rotation
):

    x, y, w, h = b
    out = []

    for j in range(7):

        row = ""

        for i in range(5):

            dark = 0

            for oy in (0.25, 0.75):

                for ox in (0.25, 0.75):

                    u = (i + ox) / 5.0
                    v = (j + oy) / 7.0

                    if rotation == 0:
                        a, bb = u, v

                    elif rotation == 90:
                        a, bb = v, 1.0 - u

                    elif rotation == 180:
                        a, bb = 1.0 - u, 1.0 - v

                    else:
                        a, bb = 1.0 - v, u

                    xx = min(
                        gray.width() - 1,
                        int(x + a * w)
                    )

                    yy = min(
                        gray.height() - 1,
                        int(y + bb * h)
                    )

                    if gray.get_pixel((xx, yy)) < threshold:
                        dark += 1

            row += "1" if dark >= 2 else "0"

        out.append(row)

    return out


def count_holes(bits):

    seen = set()
    holes = 0

    for yy in range(7):

        for xx in range(5):

            if (
                bits[yy][xx] != "0"
                or (xx, yy) in seen
            ):
                continue

            todo = [(xx, yy)]
            seen.add((xx, yy))

            outside = False

            while todo:

                x, y = todo.pop()

                if (
                    x == 0 or x == 4
                    or y == 0 or y == 6
                ):
                    outside = True

                for nx, ny in (
                    (x - 1, y),
                    (x + 1, y),
                    (x, y - 1),
                    (x, y + 1)
                ):

                    if (
                        0 <= nx < 5
                        and 0 <= ny < 7
                        and bits[ny][nx] == "0"
                        and (nx, ny) not in seen
                    ):
                        seen.add((nx, ny))
                        todo.append((nx, ny))

            if not outside:
                holes += 1

    return holes


def read_digit(gray, t, threshold):

    outer = max(1.0, t.r.x)
    half = int(0.50 * outer) + 1

    x = int(round(t.x.x))
    y = int(round(t.y.x))

    x0 = max(0, x - half)
    y0 = max(0, y - half)

    x1 = min(W, x + half + 1)
    y1 = min(H, y + half + 1)

    w = x1 - x0
    h = y1 - y0

    if w < 15 or h < 15:
        return None

    roi = (x0, y0, w, h)

    st = gray.get_statistics(roi=roi)

    thr = max(
        35,
        min(
            200,
            int(
                attr(st, "min") * 0.20
                + attr(st, "median") * 0.80
            )
        )
    )

    blobs = gray.find_blobs(
        [(0, thr)],
        roi=roi,
        x_stride=1,
        y_stride=1,
        pixels_threshold=6,
        area_threshold=10,
        merge=False
    )

    chosen = None
    rank = -1e9

    for b in blobs:

        bx = attr(b, "x")
        by = attr(b, "y")

        bw = attr(b, "w")
        bh = attr(b, "h")

        if (
            bw < 2
            or bh < 6
            or bw > 0.84 * w
            or bh > 0.94 * h
        ):
            continue

        if (
            bx <= x0
            or by <= y0
            or bx + bw >= x1
            or by + bh >= y1
        ):
            continue

        delta = (
            abs(bx + bw * 0.5 - x)
            + abs(by + bh * 0.5 - y)
        )

        if delta > 0.46 * (w + h):
            continue

        score = attr(b, "pixels") - 2.7 * delta

        if score > rank:

            chosen = (bx, by, bw, bh)
            rank = score

    if chosen is None:
        return None

    if FIXED_TEXT_ROTATION in (0, 90, 180, 270):
        orientations = (FIXED_TEXT_ROTATION,)
    else:
        orientations = (0, 90, 180, 270)

    scored = []

    for orient in orientations:

        bits = bitmap_from_blob(
            gray,
            chosen,
            thr,
            orient
        )

        holes = count_holes(bits)

        for digit, pattern in FONT.items():

            errors = min(
                sum(
                    bits[j][i] != p[j][i]
                    for j in range(7)
                    for i in range(5)
                )
                for p in (
                    pattern,
                ) + FONT_EXTRA.get(digit, ())
            )

            score = 1.0 - errors / 35.0

            expected = (
                2 if digit == 8
                else (
                    1 if digit in (0, 6, 9)
                    else 0
                )
            )

            score -= 0.055 * abs(
                holes - expected
            )

            scored.append((
                score,
                digit,
                orient
            ))

    scored.sort(reverse=True)

    best = scored[0]

    runner = 0.0

    for s, d, a in scored:

        if d != best[1] and s > runner:
            runner = s

    if (
        best[0] < OCR_MIN_SCORE
        or best[0] - runner < OCR_SCORE_GAP
    ):
        return None

    # 6 and 9 may interchange under 180-degree rotation.
    if (
        best[1] in (6, 9)
        and FIXED_TEXT_ROTATION is None
    ):
        return None

    return best[1]


# ============ Confirmed result ============

def report(tracks, now):

    possible = []

    for t in tracks:

        digit = t.confirmed_digit(now)

        if digit is not None:
            possible.append((digit, t))

    out = []

    for digit, t in possible:

        # Reject duplicated number assignments.
        if sum(
            d == digit
            for d, _ in possible
        ) != 1:
            continue

        x = int(round(t.x.x))
        y = int(round(t.y.x))

        data = {
            "digit": digit,
            "cx": x,
            "cy": y
        }

        if (
            DISTANCE_CALIBRATED
            and RING_DIAMETER_MM > 0
            and FOCAL_PX > 0
            and t.r.x > 0
        ):

            z = (
                FOCAL_PX * RING_DIAMETER_MM
                / (2.0 * t.r.x)
            )

            dist = z * math.sqrt(
                1
                + ((x - PRINCIPAL_X) / FOCAL_PX) ** 2
                + ((y - PRINCIPAL_Y) / FOCAL_PX) ** 2
            )

            data["distance_mm"] = int(round(dist))

        out.append(data)

    out.sort(
        key=lambda v: v["digit"]
    )

    return out


# ============ SSD1351 LCD ============

def lcd_init():

    if not DISPLAY:
        return None, None

    try:

        import display

        dev = display.SPIDisplay(
            controller=display.SSD1351(),
            width=128,
            height=160
        )

        canvas = image.Image(
            128,
            160,
            image.RGB565
        )

        dev.clear()

        return dev, canvas

    except Exception as e:

        print("LCD unavailable:", e)

        return None, None


def lcd_write(dev, canvas, items):

    if dev is None:
        return

    canvas.clear()

    for k, item in enumerate(items[:3]):

        y = 4 + k * 52

        canvas.draw_string(
            (2, y),
            "N:%d" % item["digit"],
            color=(255, 255, 255),
            scale=1
        )

        canvas.draw_string(
            (2, y + 15),
            "X:%d Y:%d" % (
                item["cx"],
                item["cy"]
            ),
            color=(255, 255, 255),
            scale=1
        )

        if "distance_mm" in item:

            canvas.draw_string(
                (2, y + 30),
                "D:%dmm" % item["distance_mm"],
                color=(0, 255, 0),
                scale=1
            )

    dev.write(canvas)


# ============ Main loop ============

def main():

    cam = csi.CSI()

    cam.reset()
    cam.pixformat(csi.RGB565)
    cam.framesize(csi.QVGA)

    cam.snapshot(time=1500)

    cam.auto_gain(False)
    cam.auto_whitebal(False)

    dev, canvas = lcd_init()

    tracks = []

    frame = 0
    cycle = 0

    last_lcd = time.ticks_ms()
    last_status = time.ticks_ms()

    previous = None

    detections = 0
    num_blobs = 0
    thr = 0

    clock = time.clock()

    print(
        "BLACK MULTI-RINGS: "
        "BLOB + RADIAL + KF + OCR"
    )

    if not DISTANCE_CALIBRATED:
        print("Distance OFF until calibrated")

    while True:

        clock.tick()

        img = cam.snapshot()

        now = time.ticks_ms()
        frame += 1

        if frame % DETECT_EVERY == 0:

            cycle += 1

            gray = img.to_grayscale(copy=True)

            try:

                thr = auto_black_threshold(gray)

                found, num_blobs = detect_targets(
                    gray,
                    thr,
                    cycle
                )

                detections = len(found)

                matched = update_tracks(
                    tracks,
                    found,
                    now
                )

                if cycle % OCR_EVERY == 0:

                    for t in matched:

                        if t.hits >= 2:

                            digit = read_digit(
                                gray,
                                t,
                                thr
                            )

                            t.add_ocr(digit, now)

            finally:
                del gray

            if cycle % 20 == 0:
                gc.collect()

        # IDE: red marks for confirmed geometric centers.
        for t in tracks:

            if t.confirmed_center(now):

                x = int(round(t.x.x))
                y = int(round(t.y.x))
                r = int(round(t.r.x))

                img.draw_cross(
                    (x, y),
                    color=(255, 0, 0),
                    size=10,
                    thickness=2
                )

                img.draw_circle(
                    (x, y, r),
                    color=(255, 0, 0),
                    thickness=1
                )

        # LCD receives only confirmed OCR + centers.
        items = report(tracks, now)

        if time.ticks_diff(
            now,
            last_lcd
        ) >= LCD_PERIOD_MS:

            signature = tuple(
                (
                    a["digit"],
                    a["cx"],
                    a["cy"],
                    a.get("distance_mm")
                )
                for a in items
            )

            if signature != previous:

                lcd_write(
                    dev,
                    canvas,
                    items
                )

                previous = signature

            last_lcd = now

        if (
            DEBUG
            and time.ticks_diff(
                now,
                last_status
            ) >= STATUS_PERIOD_MS
        ):

            print(
                "DBG fps=%.1f thr=%d "
                "blobs=%d rings=%d "
                "tracked=%d confirmed=%s" % (
                    clock.fps(),
                    thr,
                    num_blobs,
                    detections,
                    len(tracks),
                    str(items)
                )
            )

            last_status = now


main()

