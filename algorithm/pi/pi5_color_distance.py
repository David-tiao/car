#!/usr/bin/env python3
"""Pi5 CSI single-camera RGB material demo with per-color ruler calibration.

Bench-only, not real 3-D depth measurement; cannot safely guide robot motion.
Only the largest red/green/blue colored candidate is shown. Distance is
estimated from bounding-box width, so material size, pose, exposure, and
view direction must be consistent with the calibration sample.

Typical Pi desktop usage:
    python3 pi5_color_distance.py --distance-cm 30
Place a reference object at that measured distance and press C to calibrate.
Press Q to stop. The json file is intentionally local and should not contain
any credentials or device tokens.
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
from picamera2 import Picamera2

FRAME_SIZE = (640, 480)
CALIBRATION_FILE = Path(__file__).with_name("material_calibration.json")
COLOR_RANGES = {
    "RED": [((0, 90, 65), (10, 255, 255)),
            ((170, 90, 65), (179, 255, 255))],
    "GREEN": [((35, 65, 55), (85, 255, 255))],
    "BLUE": [((95, 70, 55), (135, 255, 255))],
}


def find_largest_blob(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    hits = []
    for color, intervals in COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low, high in intervals:
            mask |= cv2.inRange(hsv, np.array(low, dtype=np.uint8),
                                np.array(high, dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 600:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w >= 12 and h >= 12:
                hits.append((area, color, x, y, w, h))
    return max(hits, default=None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--distance-cm", type=float, default=30.0,
                        help="physical calibration distance measured by a ruler")
    args = parser.parse_args()
    if not 1 <= args.distance_cm <= 1000:
        parser.error("--distance-cm must be between 1 and 1000")
    try:
        references = json.loads(CALIBRATION_FILE.read_text(encoding="utf-8"))
        if not isinstance(references, dict):
            references = {}
    except (OSError, ValueError):
        references = {}

    camera = Picamera2()
    camera.configure(camera.create_video_configuration(
        main={"size": FRAME_SIZE, "format": "RGB888"}))
    camera.start()
    time.sleep(1)
    try:
        while True:
            frame = camera.capture_array("main")
            candidate = find_largest_blob(frame)
            if candidate is None:
                label = "No RGB material candidate"
            else:
                _, color, x, y, w, h = candidate
                reference = references.get(color)
                if (isinstance(reference, dict)
                        and isinstance(reference.get("width_px"), (int, float))
                        and isinstance(reference.get("distance_cm"), (int, float))
                        and reference["width_px"] > 0
                        and reference["distance_cm"] > 0):
                    distance_cm = reference["distance_cm"] * reference["width_px"] / w
                    label = f"{color} ~ {distance_cm:.1f} cm (estimate only)"
                else:
                    label = f"{color}: press C at {args.distance_cm:g} cm"
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(frame, (x+w//2, y+h//2),
                           4, (255, 255, 255), -1)
            cv2.putText(frame, label, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 255), 2)
            cv2.putText(frame, "C: calibrate current color / Q: quit",
                        (10, 465), cv2.FONT_HERSHEY_SIMPLEX,
                        .52, (255, 255, 255), 1)
            cv2.imshow("Pi5 HSV & calibrated width demo", frame)
            key = cv2.waitKey(1) & 0xff
            if key == ord("q"):
                break
            if key == ord("c") and candidate is not None:
                _, color, _, _, width, _ = candidate
                references[color] = {
                    "width_px": width,
                    "distance_cm": args.distance_cm,
                }
                CALIBRATION_FILE.write_text(
                    json.dumps(references, indent=2), encoding="utf-8")
                print(f"Calibrated {color}: {width} px at "
                      f"{args.distance_cm:g} cm", flush=True)
    finally:
        camera.stop()
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
