# OpenMV H7 Plus MicroPython example. Save as main.py on OpenMV ONLY if you do
# not need the existing vision / STM32 communication script. Integrate the
# request handler into the existing main loop when using QR / color detection.
# It answers USB VCP 'snap' commands with a little-endian 32-bit JPEG length
# followed by the JPEG bytes. See docs.openmv.io pyb.USB_VCP.
import sensor
import struct
from pyb import USB_VCP

usb = USB_VCP()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 160x120: stable low-bandwidth preview
sensor.skip_frames(time=1500)

while True:
    command = usb.recv(4, timeout=100)
    if command == b"snap":
        # Do not use compress_for_ide(): this must be a standard JPEG for browsers.
        frame = sensor.snapshot().compress(quality=55)
        usb.send(struct.pack("<I", frame.size()))
        usb.send(frame)
