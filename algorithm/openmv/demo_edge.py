
# demo_edge.py
# OpenMV Cam H7 Plus / Firmware v5.0.0
# 斜向下摄像头 + 前进运动 + 细黑线检测
# 单文件独立测试，不发送串口运动指令

import csi
import time


# ========== 1. 摄像头及检测参数 ==========

FRAME_SIZE = csi.QVGA

# 只检测图像下方约 67% 的区域
ROI_TOP_RATIO = 0.33

# 霍夫直线检测
HOUGH_THRESHOLD = 600
MAX_ANGLE_SLOPE = 0.45
MIN_LINE_SPAN_RATIO = 0.55

# 灰度亮暗验证
DARK_MAX = 150
MIN_CONTRAST = 18
MAX_STRIPE_THICKNESS = 9
SIDE_OFFSET = 7
HOUGH_MIN_SAMPLES = 6

# 霍夫漏检时的多列扫描
FALLBACK_COL_STEP = 12
FALLBACK_HALF_WIDTH = 60
FALLBACK_ROW_TOL = 6
FALLBACK_MIN_COLUMNS = 8
FALLBACK_MIN_SPAN = 85

# 停车测试阈值：像素，不是毫米
STOP_ROW_PX = 190

# 连续帧判断
CONFIRM_FRAMES = 3
RELEASE_FRAMES = 3
MAX_ROW_JUMP = 28

PRINT_INTERVAL_MS = 400


# ========== 2. 初始化摄像头 ==========

def init_camera():
    """初始化 H7 Plus 灰度摄像头"""
    cam = csi.CSI()
    cam.reset()

    cam.pixformat(csi.GRAYSCALE)
    cam.framesize(FRAME_SIZE)

    cam.snapshot(time=1500)

    return cam


def value(obj, attr):
    """兼容 OpenMV 对象字段访问"""
    v = getattr(obj, attr)

    if callable(v):
        return v()

    return v


def gray(img, x, y):
    """读取一个灰度像素"""
    return img.get_pixel(
        (int(x), int(y))
    )


# ========== 3. 细黑线亮暗验证 ==========

def is_dark_stripe(img, x, y):
    """
    验证当前位置是不是细黑线。

    要求：
      线上较暗
      上方较亮
      下方较亮
      黑色条带不能过厚
    """
    x = int(x)
    y = int(y)

    if x < 0 or x >= img.width():
        return False

    if y < SIDE_OFFSET + 3:
        return False

    if y >= img.height() - SIDE_OFFSET - 3:
        return False

    dark = min(
        gray(img, x, y + d)
        for d in (-2, -1, 0, 1, 2)
    )

    top = gray(img, x, y - SIDE_OFFSET)
    bottom = gray(img, x, y + SIDE_OFFSET)

    if not (
        dark <= DARK_MAX
        and top - dark >= MIN_CONTRAST
        and bottom - dark >= MIN_CONTRAST
    ):
        return False

    dark_limit = (
        min(top, bottom)
        - MIN_CONTRAST // 2
    )

    thickness = 0

    for d in range(
        -SIDE_OFFSET + 1,
        SIDE_OFFSET
    ):
        if gray(img, x, y + d) <= dark_limit:
            thickness += 1

    return thickness <= MAX_STRIPE_THICKNESS


# ========== 4. 统一检测结果格式 ==========

def make_result(
    x1, y1, x2, y2,
    support, source, width
):
    """生成可供以后总工程调用的结构化结果"""
    if x2 == x1:
        return None

    row = (
        y1 + (width // 2 - x1)
        * (y2 - y1) / float(x2 - x1)
    )

    return {
        "x1": int(x1),
        "y1": int(y1),
        "x2": int(x2),
        "y2": int(y2),
        "row": int(round(row)),
        "support": int(support),
        "source": source
    }


# ========== 5. 霍夫直线检测 ==========

def find_line_hough(img, top):
    """
    第一种检测方式：
    使用 OpenMV 内置霍夫变换寻找直线。
    再判断直线附近是否存在细黑条带。
    """
    w = img.width()
    h = img.height()

    lines = img.find_lines(
        roi=(0, top, w, h - top),
        threshold=HOUGH_THRESHOLD,
        x_stride=1,
        y_stride=1,
        theta_margin=10,
        rho_margin=8
    )

    best = None
    best_score = -1

    for line in lines:
        x1 = value(line, "x1")
        y1 = value(line, "y1")
        x2 = value(line, "x2")
        y2 = value(line, "y2")

        dx = x2 - x1
        dy = y2 - y1

        # 排除横向跨度不足的线
        if abs(dx) < w * MIN_LINE_SPAN_RATIO:
            continue

        # 排除过于倾斜的线
        if abs(dy) > MAX_ANGLE_SLOPE * abs(dx):
            continue

        hits = 0

        # 沿候选线选取9个横向位置验证
        for i in range(9):
            x = w * (i + 1) // 10

            if x < min(x1, x2):
                continue

            if x > max(x1, x2):
                continue

            y = int(round(
                y1 + (x - x1)
                * dy / float(dx)
            ))

            if y < top + SIDE_OFFSET:
                continue

            if y >= h - SIDE_OFFSET:
                continue

            if is_dark_stripe(img, x, y):
                hits += 1

        if hits < HOUGH_MIN_SAMPLES:
            continue

        score = (
            hits * 1000
            + int(value(line, "magnitude"))
        )

        if score > best_score:
            best_score = score

            best = make_result(
                x1, y1, x2, y2,
                hits, "HOUGH", w
            )

    return best


# ========== 6. 多列灰度扫描补检 ==========

def fallback_thin_stripe(img, top):
    """
    第二种检测方式：
    霍夫检测没有结果时，使用多列灰度扫描。

    这种方法对接近水平、
    只有几个像素厚的黑线比较有用。
    """
    w = img.width()
    h = img.height()

    y_start = max(
        top + SIDE_OFFSET + 2,
        SIDE_OFFSET + 3
    )

    y_end = h - SIDE_OFFSET - 3

    if y_end - y_start < 10:
        return None

    xs = range(
        w // 2 - FALLBACK_HALF_WIDTH,
        w // 2 + FALLBACK_HALF_WIDTH + 1,
        FALLBACK_COL_STEP
    )

    columns = []

    for x in xs:
        if x < 0 or x >= w:
            continue

        base_y = y_start - SIDE_OFFSET - 2

        pixels = [
            gray(img, x, y)
            for y in range(
                base_y,
                y_end + SIDE_OFFSET + 3
            )
        ]

        positions = []

        for y in range(y_start, y_end):
            i = y - base_y
            v = pixels[i]

            # 寻找局部较暗的像素
            if v > DARK_MAX:
                continue

            if v > pixels[i - 1]:
                continue

            if v > pixels[i + 1]:
                continue

            upper = pixels[i - SIDE_OFFSET]
            lower = pixels[i + SIDE_OFFSET]

            if not (
                upper - v >= MIN_CONTRAST
                and lower - v >= MIN_CONTRAST
            ):
                continue

            dark_limit = (
                min(upper, lower)
                - MIN_CONTRAST // 2
            )

            thickness = 0

            for offset in range(
                -SIDE_OFFSET + 1,
                SIDE_OFFSET
            ):
                if pixels[i + offset] <= dark_limit:
                    thickness += 1

            if thickness <= MAX_STRIPE_THICKNESS:
                positions.append(y)

        columns.append((x, positions))

    best = None
    best_score = -1

    # 将不同采样列的暗线候选进行匹配
    for seed_y in range(y_start, y_end, 2):
        points = []

        for x, positions in columns:
            near = None
            delta = FALLBACK_ROW_TOL + 1

            for yy in positions:
                d = abs(yy - seed_y)

                if d < delta:
                    delta = d
                    near = yy

            if near is not None:
                points.append((x, near))

        if len(points) < FALLBACK_MIN_COLUMNS:
            continue

        if (
            points[-1][0] - points[0][0]
            < FALLBACK_MIN_SPAN
        ):
            continue

        # 最小二乘直线拟合
        n = len(points)

        mean_x = (
            sum(p[0] for p in points)
            / float(n)
        )

        mean_y = (
            sum(p[1] for p in points)
            / float(n)
        )

        denom = sum(
            (p[0] - mean_x) ** 2
            for p in points
        )

        if denom == 0:
            continue

        slope = sum(
            (x - mean_x) * (y - mean_y)
            for x, y in points
        ) / denom

        if abs(slope) > 0.22:
            continue

        max_error = max(
            abs(
                y - (
                    mean_y
                    + slope * (x - mean_x)
                )
            )
            for x, y in points
        )

        if max_error > 3:
            continue

        score = n * 100 + int(mean_y)

        if score > best_score:
            x1 = points[0][0]
            x2 = points[-1][0]

            y1 = int(round(
                mean_y + slope * (x1 - mean_x)
            ))

            y2 = int(round(
                mean_y + slope * (x2 - mean_x)
            ))

            best = make_result(
                x1, y1, x2, y2,
                n, "PROFILE", w
            )

            best_score = score

    return best


# ========== 7. 对外统一检测函数 ==========

def detect_black_line(img):
    """
    以后可直接移入 utils.py 的核心函数。

    输入：
        img - OpenMV灰度图像

    返回：
        None：没有找到有效黑线

        字典：
            x1, y1, x2, y2
            row
            support
            source

    不负责：
        摄像头初始化
        串口发包
        电机控制
    """
    top = int(
        img.height() * ROI_TOP_RATIO
    )

    result = find_line_hough(img, top)

    if result is None:
        result = fallback_thin_stripe(
            img, top
        )

    return result


# ========== 8. 连续帧停车测试 ==========

def update_stop_state(result, state):
    """
    对像素停车条件进行多帧确认。

    注意：
    仅返回测试触发标志，
    不会控制任何电机。
    """
    if result is None:
        row = None
    else:
        row = result["row"]

    coherent = (
        row is not None
        and (
            state["last_row"] is None
            or abs(
                row - state["last_row"]
            ) <= MAX_ROW_JUMP
        )
    )

    near = (
        coherent
        and row >= STOP_ROW_PX
    )

    if coherent:
        state["last_row"] = row
    else:
        state["last_row"] = None

    if near:
        state["near_count"] += 1
        state["clear_count"] = 0
    else:
        state["near_count"] = 0
        state["clear_count"] += 1

    triggered = (
        state["near_count"] >= CONFIRM_FRAMES
        and not state["latched"]
    )

    if triggered:
        state["latched"] = True

    if state["clear_count"] >= RELEASE_FRAMES:
        state["latched"] = False

    return triggered


# ========== 9. 图像显示 ==========

def display_result(img, result):
    """绘制检测线和参考停止线"""
    w = img.width()

    # 灰色水平线为测试停车阈值
    img.draw_line(
        (0, STOP_ROW_PX,
         w - 1, STOP_ROW_PX),
        color=125,
        thickness=1
    )

    if result is None:
        return

    # 白线为实际检测到的黑线位置
    img.draw_line(
        (
            result["x1"],
            result["y1"],
            result["x2"],
            result["y2"]
        ),
        color=255,
        thickness=3
    )


# ========== 10. 独立测试主循环 ==========

def main():
    cam = init_camera()

    state = {
        "near_count": 0,
        "clear_count": 0,
        "last_row": None,
        "latched": False
    }

    last_print = time.ticks_ms()

    print("Thin black-line demo started")
    print("Grayscale + Hough + Profile")
    print("NO UART / NO MOTOR COMMANDS")

    while True:
        img = cam.snapshot()

        result = detect_black_line(img)

        triggered = update_stop_state(
            result, state
        )

        display_result(img, result)

        if triggered:
            print(
                "STOP TEST TRIGGERED at row:",
                result["row"]
            )

        now = time.ticks_ms()

        if time.ticks_diff(
            now, last_print
        ) >= PRINT_INTERVAL_MS:

            last_print = now

            if result is None:
                print("EDGE valid=0")
            else:
                print(
                    "EDGE valid=1 "
                    "row=%d support=%d "
                    "source=%s near=%d" %
                    (
                        result["row"],
                        result["support"],
                        result["source"],
                        int(
                            result["row"]
                            >= STOP_ROW_PX
                        )
                    )
                )


if __name__ == "__main__":
    main()

