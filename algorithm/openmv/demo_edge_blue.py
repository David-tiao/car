
# demo_edge_blue.py
# OpenMV H7 Plus / Firmware v5.0.0
# 蓝色启停区域检测

import time
import csi

BLUE_THRESHOLD = (
    10, 100, 0, 80, -128, -30
)

MIN_PIXELS = 200
MIN_DENSITY = 0.10
MIN_SIDE_PX = 3
MIN_LONG_PX = 20

PRINT_INTERVAL_MS = 400


def init_camera():
    cam = csi.CSI()
    cam.reset()
    cam.pixformat(csi.RGB565)
    cam.framesize(csi.QVGA)

    cam.snapshot(time=1500)
    cam.auto_gain(False)
    cam.auto_whitebal(False)
    cam.snapshot(time=300)

    return cam.snapshot


def field(obj, name, default=0):
    value = getattr(obj, name, None)

    if value is None:
        return default

    return value() if callable(value) else value


def get_blue_region(img):
    """
    检测最大蓝色区域

    返回：
        x, y, w, h
        cx, cy
        pixels
        cropped
        mode

    找不到返回 None
    """
    blobs = img.find_blobs(
        [BLUE_THRESHOLD],
        pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_PIXELS,
        merge=True
    )

    best = None

    for blob in blobs:
        w = field(blob, "w")
        h = field(blob, "h")
        pixels = field(blob, "pixels")

        if w <= 0 or h <= 0:
            continue

        if w < MIN_SIDE_PX:
            continue

        if h < MIN_SIDE_PX:
            continue

        if max(w, h) < MIN_LONG_PX:
            continue

        if pixels / float(w * h) < MIN_DENSITY:
            continue

        if (
            best is None
            or pixels > field(best, "pixels")
        ):
            best = blob

    if best is None:
        return None

    x, y, w, h = field(
        best, "rect", (0, 0, 0, 0)
    )

    cropped = (
        x <= 1
        or y <= 1
        or x + w >= img.width() - 1
        or y + h >= img.height() - 1
    )

    if w >= 3 * h:
        mode = "HORIZONTAL_EDGE_CANDIDATE"

    elif h >= 3 * w:
        mode = "VERTICAL_EDGE_CANDIDATE"

    else:
        mode = "AREA_OR_CORNER_CANDIDATE"

    return {
        "x": x,
        "y": y,
        "w": w,
        "h": h,
        "cx": field(best, "cx"),
        "cy": field(best, "cy"),
        "pixels": field(best, "pixels"),
        "cropped": cropped,
        "mode": mode
    }


def display_blue_region(img, result):
    """显示蓝色区域检测框"""
    if result is None:
        return

    rect = (
        result["x"],
        result["y"],
        result["w"],
        result["h"]
    )

    img.draw_rectangle(
        rect,
        color=(255, 255, 0),
        thickness=2
    )

    img.draw_cross(
        (result["cx"], result["cy"]),
        color=(255, 0, 0),
        size=9
    )


def main():
    snapshot = init_camera()

    last_print_ms = time.ticks_ms()

    print("Blue region demo started")

    while True:
        img = snapshot()

        result = get_blue_region(img)

        display_blue_region(img, result)

        now = time.ticks_ms()

        if time.ticks_diff(
                now, last_print_ms
                ) >= PRINT_INTERVAL_MS:

            last_print_ms = now

            if result is None:
                print("BLUE valid=0")

            else:
                print(
                    "BLUE valid=1 "
                    "x=%d y=%d w=%d h=%d "
                    "cx=%d cy=%d "
                    "visible=%s "
                    "cropped=%d pixels=%d" %
                    (
                        result["x"],
                        result["y"],
                        result["w"],
                        result["h"],
                        result["cx"],
                        result["cy"],
                        result["mode"],
                        int(result["cropped"]),
                        result["pixels"]
                    )
                )


if __name__ == "__main__":
    main()

