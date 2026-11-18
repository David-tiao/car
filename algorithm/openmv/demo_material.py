
# demo_material.py
# OpenMV H7 Plus / Firmware 5.0.x
#
# 无附加标记的白色转盘识别
# 第一阶段：圆盘边缘 + 三物料120度布局验证
# 第二阶段：顶部圆面检测 + 卡尔曼实时跟踪
#
# UART3:
# #DISK,1       转盘已确认
# #DISK,0       转盘丢失
# #TRK,label,id,cx,cy,r,vx,vy,speed,state,quality
# #LOST,label,id
#
# 本程序不发送抓取动作命令，不修改LCD任务队列。

import csi
import image
import time
import math
import gc
from machine import UART

# ==================== 基础配置 ====================

SEND_UART = True
UART_ID = 3
UART_BAUD = 115200

# 六种比赛物料的LAB颜色阈值
# 需要根据实际光照进一步标定
THRESHOLDS = [
    (15, 100, 25, 100, 10, 90),       # 1 红
    (55, 100, -40, 25, 35, 110),     # 2 黄
    (10, 100, 0, 80, -128, -30),     # 3 蓝
    (20, 100, -90, -20, 10, 70),     # 4 绿
    (0, 25, -20, 20, -20, 20),       # 5 黑
    (50, 100, -30, 10, -80, -20)    # 6 浅蓝
]

COLORS = [
    (255, 0, 0),
    (255, 255, 0),
    (0, 80, 255),
    (0, 255, 0),
    (255, 255, 255),
    (0, 255, 255)
]

# ==================== 圆盘验证参数 ====================

# 规则：白色转盘直径300mm
# 三物料中心分布圆直径200mm
#
# 以下像素范围依据当前未装车时的画面设置
# 正式安装后可能需要重新标定

DISK_R_MIN = 95
DISK_R_MAX = 165

DISK_SCAN_EVERY = 4
DISK_CONFIRM_SCANS = 5
DISK_LOST_FRAMES = 3

# 圆盘内侧亮度和内外对比度
RIM_IN_BRIGHT_MIN = 150
RIM_CONTRAST_MIN = 42

# 外缘必须有多处有效采样点
RIM_MIN_HITS = 8
RIM_MIN_SECTORS = 2
RIM_WHITE_FRACTION = 0.72

# 严格模式：最初锁定圆盘前，
# 必须同时观察到三个不同颜色的物料
REQUIRE_THREE_MATERIALS = True

# ==================== 物料参数 ====================

BLOB_PIXELS_MIN = 65
BLOB_SIDE_MIN = 13
BLOB_SIDE_MAX = 105

# 顶部圆面霍夫检测
MATERIAL_HOUGH_THRESHOLD = 1200
CAP_R_MIN = 7
CAP_R_MAX = 31

# ==================== 跟踪参数 ====================

MOVE_PX_S = 8.0
STILL_PX_S = 3.0

MOVE_MS = 150
STILL_MS = 550

TRACK_MISSES = 4

SEND_MS = 90
DEBUG_MS = 1100

TWOPI = 2.0 * math.pi

# 圆盘外缘48点采样
RIM_ANGLES = [
    (
        math.cos(TWOPI * k / 48),
        math.sin(TWOPI * k / 48)
    )
    for k in range(48)
]

# 圆盘内部亮度采样
CORE_ANGLES = [
    (
        math.cos(TWOPI * k / 12),
        math.sin(TWOPI * k / 12)
    )
    for k in range(12)
]


# ==================== 工具函数 ====================

def a(obj, name, default=0):
    v = getattr(obj, name, None)

    if v is None:
        return default

    return v() if callable(v) else v


def inside_image(x, y, w=320, h=240):
    return 0 <= x < w and 0 <= y < h


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ==================== 颜色区域初筛 ====================

def color_blobs(img):

    # 这里只提取场景特征。
    # 没有圆盘确认时，绝不把色块作为物料结果发送。

    result = []

    blobs = img.find_blobs(
        THRESHOLDS,
        x_stride=2,
        y_stride=2,
        pixels_threshold=BLOB_PIXELS_MIN,
        area_threshold=100,
        merge=False
    )

    for b in blobs:

        code = a(b, 'code')

        # 同时匹配多种颜色的区域不确认
        if code <= 0 or code & (code - 1):
            continue

        label = 1

        while code > 1:
            code >>= 1
            label += 1

        x, y, w, h = a(
            b, 'rect', (0, 0, 0, 0)
        )

        if (
            w < BLOB_SIDE_MIN
            or h < BLOB_SIDE_MIN
            or w > BLOB_SIDE_MAX
            or h > BLOB_SIDE_MAX
        ):
            continue

        ratio = min(w, h) / float(max(w, h))
        pixels = a(b, 'pixels')

        if ratio < 0.36 or pixels < BLOB_PIXELS_MIN:
            continue

        density = pixels / float(max(1, w * h))

        if density < 0.20 or density > 0.99:
            continue

        result.append((
            label,
            float(a(b, 'cx')),
            float(a(b, 'cy')),
            x, y, w, h, pixels
        ))

    return result


# ==================== 三物料布局检验 ====================

def triangle_hint(blobs):

    # 按比赛几何结构寻找三个物料：
    # 三个不同颜色
    # 围绕同一个中心
    # 相邻方向间隔约120度
    #
    # 这只负责推测圆盘位置。
    # 最终必须通过真实外缘亮度验证。

    best = None

    for i in range(len(blobs)):
        for j in range(i + 1, len(blobs)):
            for k in range(j + 1, len(blobs)):

                bs = (
                    blobs[i],
                    blobs[j],
                    blobs[k]
                )

                labels = (
                    bs[0][0],
                    bs[1][0],
                    bs[2][0]
                )

                if len(set(labels)) != 3:
                    continue

                cx = sum(b[1] for b in bs) / 3.0
                cy = sum(b[2] for b in bs) / 3.0

                ds = [
                    math.sqrt(
                        (b[1] - cx) ** 2
                        + (b[2] - cy) ** 2
                    )
                    for b in bs
                ]

                if min(ds) < 25:
                    continue

                if max(ds) > min(ds) * 1.55:
                    continue

                angles = sorted([
                    math.atan2(
                        b[2] - cy,
                        b[1] - cx
                    ) % TWOPI
                    for b in bs
                ])

                gaps = [
                    angles[1] - angles[0],
                    angles[2] - angles[1],
                    angles[0] + TWOPI - angles[2]
                ]

                # 容许透视和检测误差
                if any(
                    abs(g - TWOPI / 3) > 0.65
                    for g in gaps
                ):
                    continue

                # 200/300 = 0.667
                # 考虑斜视和圆台色块重心偏移
                r = sum(ds) / 3.0 / 0.76

                if not DISK_R_MIN <= r <= DISK_R_MAX:
                    continue

                score = sum(b[7] for b in bs)

                if best is None or score > best[0]:
                    best = (
                        score,
                        cx,
                        cy,
                        r
                    )

    return best


# ==================== 圆盘外缘验证 ====================

def disk_rim_score(gray, cx, cy, r):

    # 真正检查圆周两侧的亮度差。
    #
    # 圆盘内侧采样半径：0.91r
    # 圆盘外侧采样半径：1.07r
    #
    # 白墙没有这种持续的圆周明暗跃迁，
    # 因此不能仅凭颜色通过。

    hits = 0
    sectors = 0

    for k, (ca, sa) in enumerate(RIM_ANGLES):

        xi = int(round(cx + 0.91 * r * ca))
        yi = int(round(cy + 0.91 * r * sa))

        xo = int(round(cx + 1.07 * r * ca))
        yo = int(round(cy + 1.07 * r * sa))

        if not (
            inside_image(xi, yi)
            and inside_image(xo, yo)
        ):
            # 圆周被画面裁切时，
            # 这些点不能当作验证成功
            continue

        vin = gray.get_pixel((xi, yi))
        vout = gray.get_pixel((xo, yo))

        if (
            vin >= RIM_IN_BRIGHT_MIN
            and vin - vout >= RIM_CONTRAST_MIN
        ):
            hits += 1

            # 分成8个方向区域
            sectors |= 1 << (k // 6)

    if hits < RIM_MIN_HITS:
        return -1

    active_sectors = 0

    while sectors:
        active_sectors += sectors & 1
        sectors >>= 1

    if active_sectors < RIM_MIN_SECTORS:
        return -1

    return hits


# ==================== 圆盘内部验证 ====================

def disk_core_ok(gray, cx, cy, r):

    total = 0
    white = 0

    for fraction in (0.20, 0.42, 0.60):

        for ca, sa in CORE_ANGLES:

            x = int(round(
                cx + r * fraction * ca
            ))

            y = int(round(
                cy + r * fraction * sa
            ))

            if inside_image(x, y):

                total += 1

                if gray.get_pixel(
                    (x, y)
                ) >= RIM_IN_BRIGHT_MIN:

                    white += 1

    return (
        total >= 23
        and white >= total * RIM_WHITE_FRACTION
    )


# ==================== 检测大圆盘 ====================

def detect_disk(gray, blobs):

    hint = triangle_hint(blobs)

    if hint is None:
        return None

    _, cx, cy, r0 = hint

    best = None

    # 小范围局部搜索，提高圆盘定位精度
    # 不进行每帧全图大半径霍夫搜索

    for dx in (-6, 0, 6):
        for dy in (-6, 0, 6):
            for dr in (
                -18, -12, -6,
                0, 6, 12, 18
            ):

                r = r0 + dr

                if (
                    r < DISK_R_MIN
                    or r > DISK_R_MAX
                ):
                    continue

                xx = cx + dx
                yy = cy + dy

                score = disk_rim_score(
                    gray,
                    xx,
                    yy,
                    r
                )

                if score < 0:
                    continue

                if (
                    best is None
                    or score > best[0]
                ):
                    best = (
                        score,
                        xx,
                        yy,
                        r
                    )

    if best is None:
        return None

    _, cx, cy, r = best

    # 必须同时满足圆盘内部亮度条件
    if not disk_core_ok(
        gray,
        cx,
        cy,
        r
    ):
        return None

    return (cx, cy, r)


# ==================== 圆盘状态门控 ====================

class DiskGate:

    def __init__(self):

        self.disk = None
        self.pending = None

        self.hits = 0
        self.misses = 0

        self.locked = False

    def reset(self):
        self.__init__()

    def update(self, gray, blobs, frame):

        # 已经锁定圆盘
        if self.locked:

            cx, cy, r = self.disk

            # 每帧继续确认圆盘是否存在
            valid = (
                disk_rim_score(
                    gray, cx, cy, r
                ) >= 0
                and disk_core_ok(
                    gray, cx, cy, r
                )
            )

            if valid:
                self.misses = 0
                return self.disk

            self.misses += 1

            if self.misses >= DISK_LOST_FRAMES:
                self.reset()

            # 验证失败的帧不能识别物料
            return None

        # 未锁定时降低检测频率
        if frame % DISK_SCAN_EVERY != 0:
            return None

        cand = detect_disk(
            gray,
            blobs
        )

        if cand is None:

            self.hits = 0
            self.pending = None

            return None

        if self.pending is not None:

            dx = cand[0] - self.pending[0]
            dy = cand[1] - self.pending[1]

            similar = (
                dx * dx + dy * dy < 12 * 12
                and abs(
                    cand[2] - self.pending[2]
                ) < 15
            )

            if similar:

                self.hits += 1

                self.pending = tuple(
                    0.7 * a + 0.3 * b
                    for a, b in zip(
                        self.pending,
                        cand
                    )
                )

            else:

                self.hits = 1
                self.pending = cand

        else:

            self.hits = 1
            self.pending = cand

        # 连续五次检测成功，才确认圆盘
        if self.hits >= DISK_CONFIRM_SCANS:

            self.disk = self.pending
            self.locked = True
            self.misses = 0

            print(
                'DISK LOCKED',
                self.disk
            )

            return self.disk

        return None


# ==================== 顶部圆面识别 ====================

def material_candidates(img, gray, disk):

    cx0, cy0, disk_r = disk

    out = []

    # 只在已确认圆盘后使用颜色候选
    blobs = color_blobs(img)

    for label, cx, cy, x, y, w, h, pixels in blobs:

        d = math.sqrt(
            (cx - cx0) ** 2
            + (cy - cy0) ** 2
        )

        # 物料中心应位于转盘内部规定的分布区域
        if not (
            0.39 * disk_r
            <= d
            <= 0.96 * disk_r
        ):
            continue

        # 仅对候选物料的小区域使用霍夫检测
        pad = 4

        x0 = max(0, x - pad)
        y0 = max(0, y - pad)

        x1 = min(
            img.width(),
            x + w + pad
        )

        y1 = min(
            img.height(),
            y + h + pad
        )

        if x1 - x0 < 18 or y1 - y0 < 18:
            continue

        circles = gray.find_circles(
            roi=(
                x0,
                y0,
                x1 - x0,
                y1 - y0
            ),
            threshold=MATERIAL_HOUGH_THRESHOLD,
            x_stride=2,
            y_stride=2,
            x_margin=5,
            y_margin=5,
            r_margin=3,
            r_min=CAP_R_MIN,
            r_max=min(
                CAP_R_MAX,
                (min(w, h) + 6) // 2
            ),
            r_step=2
        )

        best = None

        for c in circles:

            xc = a(c, 'x')
            yc = a(c, 'y')
            rr = a(c, 'r')

            if not (
                x - 2 <= xc <= x + w + 2
                and y - 2 <= yc <= y + h + 2
            ):
                continue

            diameter_ratio = (
                2.0 * rr / float(min(w, h))
            )

            if not (
                0.20 <= diameter_ratio <= 1.18
            ):
                continue

            radial_d = math.sqrt(
                (xc - cx0) ** 2
                + (yc - cy0) ** 2
            )

            if not (
                0.35 * disk_r
                <= radial_d
                <= 1.00 * disk_r
            ):
                continue

            score = a(
                c, 'magnitude', 0
            )

            if (
                best is None
                or score > best[0]
            ):
                best = (
                    score,
                    xc,
                    yc,
                    rr
                )

        if best is None:
            continue

        # quality仅为检测强度参考值，不是概率
        quality = min(
            99,
            int(best[0] / 40.0)
        )

        out.append((
            label,
            best[1],
            best[2],
            best[3],
            quality
        ))

    # 去除同一物料的重复检测
    result = []

    for v in out:

        separate = all(
            (v[1] - q[1]) ** 2
            + (v[2] - q[2]) ** 2
            > 13 ** 2
            for q in result
        )

        if separate:
            result.append(v)

    return result


# ==================== 卡尔曼滤波 ====================

class KF:

    # 状态向量：[位置, 速度]

    def __init__(self, pos):

        self.x = float(pos)
        self.v = 0.0

        self.p00 = 30.0
        self.p01 = 0.0
        self.p11 = 500.0

    def predict(self, dt):

        dt = clamp(
            dt, 0.001, 0.3
        )

        p00 = self.p00
        p01 = self.p01
        p11 = self.p11

        self.x += self.v * dt

        self.p00 = (
            p00
            + 2 * dt * p01
            + dt * dt * p11
            + 225 * dt ** 4
        )

        self.p01 = (
            p01
            + dt * p11
            + 450 * dt ** 3
        )

        self.p11 = (
            p11
            + 900 * dt * dt
        )

    def correct(self, pos):

        e = pos - self.x
        var = self.p00 + 9.0

        k0 = self.p00 / var
        k1 = self.p01 / var

        old01 = self.p01

        self.x += k0 * e
        self.v += k1 * e

        self.p00 = max(
            0.01,
            self.p00 * (1 - k0)
        )

        self.p01 = (
            self.p01 * (1 - k0)
        )

        self.p11 = max(
            0.01,
            self.p11 - k1 * old01
        )


# ==================== 单个物料跟踪 ====================

class Track:

    def __init__(self, det, tid, now):

        self.label = det[0]
        self.id = tid

        self.kx = KF(det[1])
        self.ky = KF(det[2])

        self.r = float(det[3])
        self.quality = det[4]

        self.last = now
        self.last_sent = now

        self.hits = 1
        self.misses = 0

        self.seen = True

        # U 未确认
        # M 运动
        # S 静止
        self.state = 'U'

        self.pending = None
        self.pending_at = now

    def predict(self, now):

        dt = time.ticks_diff(
            now,
            self.last
        ) / 1000.0

        self.kx.predict(dt)
        self.ky.predict(dt)

        self.last = now
        self.seen = False

    def update(self, det, now):

        self.kx.correct(det[1])
        self.ky.correct(det[2])

        self.r = (
            0.7 * self.r
            + 0.3 * det[3]
        )

        self.quality = det[4]

        self.hits += 1
        self.misses = 0
        self.seen = True

        speed = math.sqrt(
            self.kx.v ** 2
            + self.ky.v ** 2
        )

        if speed >= MOVE_PX_S:
            proposed = 'M'

        elif speed <= STILL_PX_S:
            proposed = 'S'

        else:
            proposed = None

        if proposed is None:

            self.pending = None

        elif self.pending != proposed:

            self.pending = proposed
            self.pending_at = now

        else:

            duration = (
                MOVE_MS
                if proposed == 'M'
                else STILL_MS
            )

            if time.ticks_diff(
                now,
                self.pending_at
            ) >= duration:

                self.state = proposed


# ==================== 多物料跟踪 ====================

class MaterialTracker:

    def __init__(self):

        self.items = []
        self.next_id = 1

    def clear(self):
        self.items = []

    def update(self, detections, now):

        for t in self.items:
            t.predict(now)

        pairs = []

        for i, t in enumerate(self.items):

            for j, d in enumerate(detections):

                if t.label != d[0]:
                    continue

                dist2 = (
                    (t.kx.x - d[1]) ** 2
                    + (t.ky.x - d[2]) ** 2
                )

                gate = max(
                    22,
                    2.0 * t.r
                )

                if dist2 < gate ** 2:
                    pairs.append(
                        (dist2, i, j)
                    )

        pairs.sort()

        used_i = set()
        used_j = set()

        for _, i, j in pairs:

            if (
                i not in used_i
                and j not in used_j
            ):

                self.items[i].update(
                    detections[j],
                    now
                )

                used_i.add(i)
                used_j.add(j)

        for i, t in enumerate(self.items):

            if i not in used_i:

                t.misses += 1
                t.state = 'U'

        lost = [
            (t.label, t.id)
            for t in self.items
            if t.misses > TRACK_MISSES
        ]

        self.items = [
            t for t in self.items
            if t.misses <= TRACK_MISSES
        ]

        for j, d in enumerate(detections):

            if (
                j not in used_j
                and len(self.items) < 3
            ):

                self.items.append(
                    Track(
                        d,
                        self.next_id,
                        now
                    )
                )

                self.next_id += 1

        return lost


# ==================== UART通信 ====================

class Link:

    def __init__(self):

        self.uart = None

        if SEND_UART:

            self.uart = UART(
                UART_ID,
                UART_BAUD,
                timeout=0
            )

    def send(self, msg):

        if self.uart is not None:

            self.uart.write(
                msg.encode('ascii')
            )

    def track(self, t, now):

        # 只有真实检测并跟踪确认的目标才能发送
        if not t.seen or t.hits < 3:
            return

        if time.ticks_diff(
            now,
            t.last_sent
        ) < SEND_MS:
            return

        speed = math.sqrt(
            t.kx.v ** 2
            + t.ky.v ** 2
        )

        msg = (
            '#TRK,%d,%d,%d,%d,%d,%d,%d,%d,%s,%d\n'
            % (
                t.label,
                t.id,
                int(round(t.kx.x)),
                int(round(t.ky.x)),
                int(round(t.r)),
                int(round(t.kx.v)),
                int(round(t.ky.v)),
                int(round(speed)),
                t.state,
                t.quality
            )
        )

        self.send(msg)
        t.last_sent = now


# ==================== 主程序 ====================

def main():

    cam = csi.CSI()

    cam.reset()
    cam.pixformat(csi.RGB565)
    cam.framesize(csi.QVGA)

    cam.snapshot(time=1400)

    cam.auto_gain(False)
    cam.auto_whitebal(False)

    gate = DiskGate()
    tracker = MaterialTracker()
    link = Link()

    frame = 0
    last_debug = time.ticks_ms()

    was_locked = False

    print('STRICT DISK GATE STARTED')
    print('White rim + 120-degree layout')
    print('Material detection disabled until disk lock')

    while True:

        img = cam.snapshot()

        frame += 1
        now = time.ticks_ms()

        # 第一阶段：仅提取场景验证特征
        blobs = color_blobs(img)

        gray = img.to_grayscale(
            copy=True
        )

        disk = gate.update(
            gray,
            blobs,
            frame
        )

        # 圆盘锁定状态变化
        if gate.locked != was_locked:

            link.send(
                '#DISK,%d\n'
                % (1 if gate.locked else 0)
            )

            if not gate.locked:

                tracker.clear()

                print(
                    'DISK LOST - MATERIAL DISABLED'
                )

            was_locked = gate.locked

        # ================================
        # 未确认圆盘：禁止物料输出
        # ================================

        if disk is None:

            img.draw_string(
                (3, 3),
                'SEARCH DISK - NO MATERIAL TX',
                color=(255, 255, 0),
                scale=1
            )

            # 黄色圆圈仅表示未确认的圆盘候选
            if gate.pending is not None:

                x, y, r = gate.pending

                img.draw_circle(
                    (
                        int(x),
                        int(y),
                        int(r)
                    ),
                    color=(255, 255, 0),
                    thickness=1
                )

        # ================================
        # 已确认圆盘：允许物料识别
        # ================================

        else:

            px, py, pr = disk

            # 绿色圆圈标记已确认的白色转盘
            img.draw_circle(
                (
                    int(px),
                    int(py),
                    int(pr)
                ),
                color=(0, 255, 0),
                thickness=2
            )

            # 只有现在才运行顶部圆面检测
            dets = material_candidates(
                img,
                gray,
                disk
            )

            # 卡尔曼跟踪
            lost = tracker.update(
                dets,
                now
            )

            for label, tid in lost:

                link.send(
                    '#LOST,%d,%d\n'
                    % (label, tid)
                )

            for t in tracker.items:

                if not t.seen:
                    continue

                x = int(round(t.kx.x))
                y = int(round(t.ky.x))
                r = int(round(t.r))

                color = COLORS[
                    t.label - 1
                ]

                # 顶部圆面圆圈
                img.draw_circle(
                    (x, y, r),
                    color=color,
                    thickness=2
                )

                # 物料圆心
                img.draw_cross(
                    (x, y),
                    color=(255, 0, 0),
                    size=8
                )

                img.draw_string(
                    (
                        max(0, x - 15),
                        max(0, y - r - 14)
                    ),
                    '%d %s' % (
                        t.label,
                        t.state
                    ),
                    color=color
                )

                # 运动和静止都实时发送
                link.track(
                    t,
                    now
                )

        del gray

        # ================================
        # 调试信息
        # ================================

        if time.ticks_diff(
            now,
            last_debug
        ) > DEBUG_MS:

            print(
                'GATE=%s confirm=%d rim=%s tracks=%d'
                % (
                    'LOCKED'
                    if gate.locked
                    else 'SEARCH',

                    gate.hits,

                    str(
                        tuple(
                            int(v)
                            for v in gate.disk
                        )
                    )
                    if gate.disk
                    else 'NONE',

                    len(tracker.items)
                )
            )

            last_debug = now

        if frame % 25 == 0:
            gc.collect()


if __name__ == '__main__':
    main()

