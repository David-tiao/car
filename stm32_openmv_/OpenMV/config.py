
# config.py
# OpenMV H7 Plus / Firmware 5.x

UART_ID = 3
UART_BAUD = 115200

# 已验证成功的 LCD 设置
LCD_WIDTH = 128
LCD_HEIGHT = 160
LCD_BACKLIGHT = 100

QR_CONFIRM_FRAMES = 3
QR_RECT_HOLD_MS = 600

# 六种比赛物料颜色，LAB 阈值
# 物料几何及圆盘参数在 utils.py 中
MATERIAL_THRESHOLDS = {
    1: (15, 100, 25, 100, 10, 90),
    2: (55, 100, -40, 25, 35, 110),
    3: (10, 100, 0, 80, -128, -30),
    4: (20, 100, -90, -20, 10, 70),
    5: (0, 25, -20, 20, -20, 20),
    6: (50, 100, -30, 10, -80, -20),
}

# Fixed claw-side station: must be calibrated on the real vehicle.
# The image-side point showing the direction from disk centre towards the claw.
PICK_DIRECTION_X = 160
PICK_DIRECTION_Y = 350
# Desired image location of the fixed station. Align pixels with STM32 config.
PICK_TARGET_X = 160
PICK_TARGET_Y = 190
PICK_SLOT_TOL_PX = 12
PICK_SLOT_CONFIRM_MS = 150
PICK_SLOT_SEND_MS = 220

# STM32 confirms pickup has begun; only then may the material be occluded.
OBJ_TAKE_TIMEOUT_MS = 2400
ACK_TIMEOUT_MS = 1500
MAX_RESENDS = 3
STATUS_PERIOD_MS = 2000
