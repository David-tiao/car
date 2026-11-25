"""OpenMV H7 Plus 初赛配置。在实际安装姿态和比赛光照下标定像素参数。"""
# UART3：P4 TX 接 STM32 RX，P5 RX 接 STM32 TX，两板共地。
UART_ID = 3
UART_BAUD = 115200
UART_BITS = 8
UART_PARITY = None
UART_STOP_BITS = 1
UART_READ_TIMEOUT_MS = 0
UART_WRITE_TIMEOUT_MS = 30
UART_READ_CHUNK = 192         # 每次主循环最多读取的字节数
UART_MAX_LINE = 64           # 接收帧的 ASCII 字节上限，不含换行
SEQ_REPEAT_MS = 1000         # 等待 ARM 时重复上报已确认的任务码
REPORT_MS = 220              # 工位及观测上报间隔
OBJ_TAKE_TIMEOUT_MS = 2400
OBJ_RETRY_MS = 700

# 相机固定 RGB565/QVGA（320 × 240）。本程序与 V5 的像素坐标均基于此分辨率。
CAMERA_SETTLE_MS = 1500
GC_EVERY_FRAMES = 30
# 任务屏。无屏台架可置 False，#STATUS 可查步骤及确认计数。
LCD_ENABLED = True
LCD_WIDTH = 128
LCD_HEIGHT = 160
LCD_BACKLIGHT = 100
QR_CONFIRM_FRAMES = 3

# 1 红 2 黄 3 蓝 4 绿 5 黑 6 浅蓝。用实物和比赛补光在 IDE 中重测。
MATERIAL_THRESHOLDS = {
    1: (15, 100, 25, 100, 10, 90),
    2: (55, 100, -40, 25, 35, 110),
    3: (10, 100, 0, 80, -128, -30),
    4: (20, 100, -90, -20, 10, 70),
    5: (0, 25, -20, 20, -20, 20),
    6: (50, 100, -30, 10, -80, -20),
}

# 图2 两圆直径比。下列亮度、半径、相机朝向及时间均为工程初值。
ORBIT_RATIO = 200.0 / 300.0
DISK_R_MIN = 95
DISK_R_MAX = 165   # 原 V5 #SLOT 接口允许的半径上限
RIM_IN_BRIGHT_MIN = 150
RIM_CONTRAST_MIN = 42
CAP_R_MIN = 7
CAP_R_MAX = 31
MATERIAL_HOUGH_THRESHOLD = 1200
PICK_DIRECTION_X, PICK_DIRECTION_Y = 160, 350
PICK_TARGET_X, PICK_TARGET_Y = 160, 190
PICK_SLOT_TOL_PX = 12
STILL_MS = 550
SLOT_HOLD_MS = 150
MAX_FRAME_GAP_MS = 450

# 盘沿、色块及轨迹的确认参数。速度单位为像素/秒，半径和边长单位为像素。
DISK_SCAN_EVERY = 4
DISK_CONFIRM_SCANS = 5
DISK_LOST_FRAMES = 3
RIM_MIN_HITS = 8
RIM_MIN_SECTORS = 2
RIM_WHITE_FRACTION = 0.72
BLOB_PIXELS_MIN = 65
BLOB_SIDE_MIN = 13
BLOB_SIDE_MAX = 105
MOVE_PX_S = 8.0
STILL_PX_S = 3.0
MOVE_MS = 150
TRACK_MISSES = 4

# 遮号工位。按整车约定的可重复停车姿态、相机俯角和焦距实测填写。
# 填写 (中心u, 中心v, 半窗宽)，窗口应完整包含指定料顶并排除相邻底座。
# 三个数字同时被挡时需要实测窗口；None 仅使用当前帧中能定位的环位。
COVERED_WINDOWS = {
    'ROUGH': {1: None, 2: None, 3: None},
    'STORE': {1: None, 2: None, 3: None},
}
RING_MAX_RADIUS = 65
RING_MIN_RADIUS = 9
RING_BLACK_THRESHOLD = -1
RING_RADIAL_MIN_FRACTION = 0.41
RING_OCR_MIN_SCORE = 0.74
RING_OCR_MIN_GAP = 0.05
RING_TEXT_ROTATION = None  # 或 0/90/180/270（相机中数字的方向）
STATION_CONFIRM_FRAMES = 3
STATION_JITTER_PX = 5
# 蓝启停区的 LAB 范围、最小面积在返程视角下调；由底盘区分两个启停区。
HOME_BLUE = (10, 100, 0, 80, -128, -30)
HOME_MIN_PIXELS = 1200

# 电控 architecture-v2 使用 V4；原 V5 主控可显式选择 V5。
UART_PROTOCOL = 'V4'
RING_TARGET_X, RING_TARGET_Y = 160, 120

# In the fixed camera view, teach (u, v, half_width) for each car pocket
# (slot 1..3). The tray angle can differ before and after a movement.
# Each empty pocket needs a visible patch of the taught LAB background.
# Use a marker on the vehicle only. None means "no physical evidence".
TRAY_WINDOWS = {
    ('PICK', 'B'): (None, None, None), ('PICK', 'A'): (None, None, None),
    ('PUT', 'B'): (None, None, None), ('PUT', 'A'): (None, None, None),
}
# Override a view when the tray reaches a different angle on a particular
# transfer. Keys are (mission step 1..24, 'B'/'A'), values use the same format.
TRAY_STEP_WINDOWS = {}
TRAY_EMPTY_LAB = None
TRAY_EMPTY_FRACTION = 0.55
# With an oblique camera, teach disjoint top-face windows at one and two
# levels per storage ring. If the levels look identical, use another view.
STACK_WINDOWS = {1: (None, None), 2: (None, None), 3: (None, None)}

# At the taught home pose, choose a ground-only (u, v, half_width) patch for
# each start zone. Calibrate the blue coverage threshold on the real floor.
HOME_WINDOWS = {1: None, 2: None}
HOME_BLUE_FRACTION = 0.75
