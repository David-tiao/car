# OpenMV H7 Plus / firmware 5.x. Standalone calibration capture helper.
# Run it temporarily from OpenMV IDE; DO NOT replace the existing project main.py.
# Images must match the 320x240, full-frame, uncorrected capture configuration.
import csi
import os
import time

SAVE_DIR = '/sdcard/calib'  # Some older firmwares mount SD at '/'; see README.
NUM_IMAGES = 35
INTERVAL_MS = 2500

# Refuse to write the camera internal flash by accident.
try:
    os.listdir('/sdcard')
except OSError:
    raise RuntimeError("SD card not mounted at /sdcard; check mount path in IDE before continuing")

try:
    os.mkdir(SAVE_DIR)
except OSError:
    # Existing directory is OK.
    os.listdir(SAVE_DIR)

cam = csi.CSI()
cam.reset()
cam.pixformat(csi.RGB565)
cam.framesize(csi.QVGA)
cam.snapshot(time=2000)

print('Capturing chessboard photos: 320x240 RGB565, 9x6 inner corners')
print('Every 2.5 seconds, move/tilt the chessboard to a DIFFERENT pose')
print('Keep the entire chessboard visible and in focus')

for i in range(NUM_IMAGES):
    time.sleep_ms(INTERVAL_MS)
    img = cam.snapshot()
    filename = '%s/cal_%03d.jpg' % (SAVE_DIR, i + 1)
    img.save(filename, quality=90)
    print('saved', filename)

print('Done. Copy the photos to demo/calibration/images on the PC.')
