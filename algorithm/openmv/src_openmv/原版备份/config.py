# config.py - OpenMV H7 Plus, firmware 5.x
# Restore the confirmed QR/LCD settings. Other thresholds from existing demos.

UART_ID = 3
UART_BAUD = 115200

LCD_WIDTH = 128
LCD_HEIGHT = 160
LCD_BACKLIGHT = 100

QR_CONFIRM_FRAMES = 3
QR_RECT_HOLD_MS = 600

# 1=red 2=yellow 3=blue 4=green 5=black 6=lightblue
MATERIAL_THRESHOLDS = {
    1: (15, 100, 25, 100, 10, 90),
    2: (55, 100, -40, 25, 35, 110),
    3: (10, 100, 0, 80, -128, -30),
    4: (20, 100, -90, -20, 10, 70),
    5: (0, 25, -20, 20, -20, 20),
    6: (50, 100, -30, 10, -80, -20),
}
MATERIAL_MIN_PIXELS = 200
MATERIAL_MIN_DENSITY = 0.35
MATERIAL_STILL_FRAMES = 8
MATERIAL_POS_TOL_PX = 15
MATERIAL_SIZE_TOL_PCT = 8
MATERIAL_STABLE_FRAMES = 3
MATERIAL_DIAMETER_MM = {1: 40.0, 2: 40.0, 3: 40.0,
                        4: 40.0, 5: 40.0, 6: 40.0}

# Distance is NOT emitted as nonzero until calibration is explicitly enabled.
DISTANCE_CALIBRATED = False
FOCAL_PX = 0.0
RING_DIAMETER_MM = 0.0
PRINCIPAL_X_PX = 160.0
PRINCIPAL_Y_PX = 120.0

# Black printed concentric rings, NOT colored rings.
RING_DETECT_INTERVAL = 2
RING_MAX_RADIUS = 65
RING_MIN_RADIUS = 9
RING_BLACK_THRESHOLD = -1   # -1 = automatic
RING_RADIAL_MIN_FRACTION = 0.41
RING_MIN_HITS = 3
RING_OCR_WINDOW = 5
RING_OCR_VOTES = 3
RING_OCR_MIN_SCORE = 0.74
RING_OCR_MIN_GAP = 0.05
RING_TEXT_ROTATION = None   # None: auto 0/90/180/270; or set 90 etc.

ACK_TIMEOUT_MS = 1500
MAX_RESENDS = 3
STATUS_PERIOD_MS = 2000

# Edge demo thresholds: near = lower image row, NOT physical millimeters.
EDGE_STOP_ROW = 190
EDGE_CONFIRM = 3
EDGE_RELEASE = 3
EDGE_DARK_MAX = 150
EDGE_CONTRAST = 18

# Blue area, same LAB as demo_edge_blue.
BLUE_THRESHOLD = (10, 100, 0, 80, -128, -30)
BLUE_MIN_PIXELS = 200
BLUE_MIN_DENSITY = 0.10

# Only remove a display group on STM32's #GROUP_DONE,n message.
# K/B remain SINGLE ITEM acknowledgements, never group completion.

# #MODE,EDGE / #MODE,BLUE are auxiliary modes. They do not drive motors.
