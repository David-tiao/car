#!/usr/bin/env python3
"""Pi5 CSI camera GUI experiment: QR task text + RGB color blobs.

Bench-only tool. NOT a replacement for production OpenMV/STM32 V4 protocol.
Does not publish ROS2 messages or send any motor/serial commands.

Required on supported Raspberry Pi OS: python3-opencv, python3-numpy,
python3-picamera2, python3-pil.imagetk and tkinter (package names can vary).
Run from a physical Pi desktop with a compatible connected camera.
"""
import re
import tkinter as tk

import cv2
import numpy as np
from PIL import Image, ImageTk
from picamera2 import Picamera2

FRAME_SIZE = (640, 480)
COLOR_RANGES = {
    "RED": [((0, 100, 70), (10, 255, 255)),
            ((170, 100, 70), (179, 255, 255))],
    "GREEN": [((35, 70, 60), (85, 255, 255))],
    "BLUE": [((95, 85, 60), (135, 255, 255))],
}
DRAW_BGR = {"RED": (0, 0, 255), "GREEN": (0, 255, 0),
            "BLUE": (255, 0, 0)}


def valid_competition_code(text):
    """Offline-only validation of 2027 preliminary task code syntax."""
    if not re.fullmatch(r"[1-6]{3}\+[1-3]{3}\+[1-6]{3}\+[1-3]{3}", text):
        return False
    first_colors, first_rings, second_colors, second_rings = text.split("+")
    return (len(set(first_colors)) == 3
            and set(first_colors) == set(second_colors)
            and set(first_rings) == set("123")
            and set(second_rings) == set("123"))


def find_color_regions(bgr):
    """3-color HSV blob demo; not a 6-color/shape material classifier."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), dtype=np.uint8)
    detections = []
    for name, intervals in COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in intervals:
            mask |= cv2.inRange(hsv, np.array(lower, dtype=np.uint8),
                                np.array(upper, dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < 1800:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            detections.append((name, x, y, w, h))
    return detections


class Viewer:
    def __init__(self, root):
        self.root = root
        self.active = True
        self.camera = None
        self.last_code = None
        root.title("Pi5 QR / RGB demo (NOT car control)")
        root.geometry("740x740")
        tk.Label(root, text="Pi5 CSI QR / Material HSV - bench demo",
                 font=("Sans", 14, "bold")).pack(pady=8)
        self.preview = tk.Label(root)
        self.preview.pack()
        self.result = tk.StringVar(value="Waiting for QR...")
        tk.Label(root, textvariable=self.result, wraplength=700,
                 font=("Sans", 12)).pack(pady=8)
        self.show_color = tk.BooleanVar(value=False)
        tk.Checkbutton(root, text="Show red/green/blue HSV candidates",
                       variable=self.show_color).pack(pady=4)
        self.status = tk.StringVar(value="No camera session")
        tk.Label(root, textvariable=self.status).pack(pady=4)
        tk.Button(root, text="Close", command=self.close).pack()
        self.detector = cv2.QRCodeDetector()
        self.camera = Picamera2()
        # Picamera2 RGB888 capture_array is BGR ordered for OpenCV on Pi.
        self.camera.configure(self.camera.create_video_configuration(
            main={"size": FRAME_SIZE, "format": "RGB888"}))
        self.camera.start()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(100, self.refresh)

    def refresh(self):
        if not self.active:
            return
        try:
            frame = self.camera.capture_array("main")
            content, corners, _ = self.detector.detectAndDecode(frame)
            if content:
                valid = valid_competition_code(content)
                self.result.set(
                    ("Valid preliminary code: " if valid
                     else "QR decoded, not a validated race task code: ")
                    + content)
                if content != self.last_code:
                    print("QR:", repr(content), "valid=", valid, flush=True)
                    self.last_code = content
                if corners is not None:
                    contour = np.int32(np.round(corners)).reshape((-1, 1, 2))
                    cv2.polylines(frame, [contour], True, (0, 255, 0), 2)
            if self.show_color.get():
                hits = find_color_regions(frame)
                for name, x, y, w, h in hits:
                    cv2.rectangle(frame, (x, y), (x+w, y+h),
                                  DRAW_BGR[name], 2)
                    cv2.putText(frame, name, (x, max(16, y-5)),
                                cv2.FONT_HERSHEY_SIMPLEX, .55,
                                DRAW_BGR[name], 2)
                self.status.set("HSV candidate regions: " + str(len(hits)))
            else:
                self.status.set("Color demo disabled")
            image = ImageTk.PhotoImage(Image.fromarray(
                cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
            self.preview.configure(image=image)
            self.preview.image = image
        except Exception as exc:
            self.status.set("Camera / detection error: " + str(exc))
        finally:
            if self.active:
                self.root.after(70, self.refresh)

    def close(self):
        if not self.active:
            return
        self.active = False
        try:
            if self.camera is not None:
                self.camera.stop()
                self.camera.close()
        finally:
            self.root.destroy()


def main():
    root = tk.Tk()
    Viewer(root)
    root.mainloop()


if __name__ == "__main__":
    main()
