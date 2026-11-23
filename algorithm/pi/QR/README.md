# Raspberry Pi 5 · C++ 二维码识别与车载屏显示

> 2026-11-23 新建，属于 **Pi5 视觉任务第 1 阶段**。C2 连接由项目负责人确认已完成；本目录只开发 QR 模块，不重复做 C2。**源码及离线单测 ≠ Pi5 摄像头 / 实体显示屏已实测。**

## 目标与边界

从树莓派 5 的 **CSI 排线摄像头** 获取连续画面，使用 **C++17 + OpenCV** 识别二维码，按比赛任务码格式校验并 **连续 3 帧确认**，最终把二维码中的有效任务码大字号显示在接到 Pi 的 HDMI / DSI 桌面显示器上。

例子：`156+123+516+231`

屏幕为防止小尺寸屏文字太小，采用两行显示，第一行 `156+123+`，第二行 `516+231`，拼接后仍是完整原始任务码；识别状态有 `CONFIRMED / LIVE`、`LAST CONFIRMED / NOT LIVE`、`INVALID`、`MULTIPLE QR`。

**设备边界**：本程序只采集、解码、显示与可选保存 UTF-8 纯文本；**不会向 STM32、OpenMV、ROS2、GPIO、网络或电机发送任何指令**。比赛是否将 Pi 摄像头设为唯一 QR 来源，还需与原有 OpenMV H7 Plus 任务码权威链协商，防止两个 QR 来源产生冲突。未来接入正式比赛 FSM 必须按统一协议设计。

## 目录

```text
algorithm/pi/QR/
├── CMakeLists.txt
├── README.md
├── include/pi_qr/task_code.hpp       # 比赛码格式及连续确认状态机
├── src/task_code.cpp                 # 与 OpenMV parse_task 对齐
├── src/main.cpp                      # Pi CSI / USB / image → OpenCV → 屏幕
└── tests/test_task_code.cpp          # 纯 C++ 单元测试，无硬件依赖
```

## 1. 环境准备（Raspberry Pi OS / ARM64）

先确认 **实体 CSI 相机和桌面显示器** 在树莓派 5 上可用：

```bash
rpicam-hello --list-cameras
rpicam-hello --timeout 3000
```

如果没有 CSI 相机，可用 `--source usb --device /dev/video0` 接 USB/V4L2 摄像头；也可以在没有设备的 Ubuntu PC 上测试 PNG/JPEG 图片。

安装编译依赖（需连接软件仓库；先检查是否已有这些包）：

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev rpicam-apps
```

当前摄像头路径使用 **rpicam-vid --codec yuv420** 通过一个只读管道输出原始帧。该用法参照 [Raspberry Pi 官方 Camera Software](https://www.raspberrypi.com/documentation/accessories/camera-software.html) 的 YUV420 示例。仅将 YUV 数据交给 OpenCV 解码和检测，不走 Python/Picamera2，不依赖独立 Python 进程。

> 使用厂商定制镜像、旧 Pi OS 或 Ubuntu ARM64 时，`rpicam-vid` 的名字/支持的参数和 CSI 驱动可能不同；运行前先查看 `rpicam-vid --help` 和 `rpicam-hello --list-cameras`。没有真实摄像头不可以声称抓帧通过。

## 2. 构建与 C++ 测试

在 `car` 仓库根目录执行：

```bash
cmake -S algorithm/pi/QR -B build/pi_qr -DCMAKE_BUILD_TYPE=Release
cmake --build build/pi_qr -j2
ctest --test-dir build/pi_qr --output-on-failure
```

单测检查格式、两批同色集合、环号、重复数字、非法字符、丢帧/多二维码拒绝、3 帧稳定确认及清空操作。它不等于相机和屏幕真机验收。

## 3. 实体树莓派：识别并显示

```bash
# 默认 CSI 摄像头 / 640x480 / 15fps / 连续3帧 / 全屏
./build/pi_qr/pi_qr --source pi

# 例如 800×480 屏幕；实体有效显示高度经尺子测得 85 mm
./build/pi_qr/pi_qr --source pi \
  --screen-width 800 --screen-height 480 --screen-height-mm 85

# 额外把新确认的任务码写到本机纯文本文件（仅供以后软件联调）
./build/pi_qr/pi_qr --source pi --output /tmp/pi_qr_verified.txt

# USB 摄像头（如果 Pi5 CSI 不可用）
./build/pi_qr/pi_qr --source usb --device /dev/video0 --window
```

`--screen-height-mm` 根据显示屏的**实测物理显示高度**估算文字毫米高度；如果码文由于窗口/屏幕尺寸不足而不得不缩小，画面会警告 `TEXT UNDER 12mm`。若显示屏驱动改变缩放/分辨率，必须用实物尺子再次确认。**缺少此参数不代表已经满足现场字高要求**。

按 Q / Esc 退出，按 R 清除上一次已确认任务码；不运行二次 QR 或其他自动动作。若暂时看不到 GUI，可用 `--headless` 在 SSH 终端验证采集和输出（**不满足车载屏显示验收**）。需要运行图形桌面的 `DISPLAY` / Wayland / Xwayland/OpenCV HighGUI 支持。

## 4. 不接硬件的图片测试

二维码测试图请先用受信任的工具生成，内容例如：

```text
156+123+516+231
```

若电脑装有 `qrencode`，可直接生成：

```bash
qrencode -o /tmp/pi_qr_valid.png -s 8 '156+123+516+231'
./build/pi_qr/pi_qr --image /tmp/pi_qr_valid.png --headless
```

预期输出包含：

```text
QR_CONFIRMED 156+123+516+231
```

图片测试为**同一张静态图重复输入状态机**，不能证明真实 3 个不同时间帧的稳定识别。故对照片/视频/真人工况还要实测：摆放角度、大小、距离、光照、反光、模糊、二维码随机位置、同时出现两个码、断流恢复。

## 5. 和原有比赛源码保持一致的格式

参考 [正式 OpenMV `tools.py::parse_task`](../../openmv/openmv_all/tools.py) 与 [比赛任务代码协议](../../../通信协议.md)：

- **四组**，每组严格 3 个 ASCII 数字，以三个 `+` 分隔。
- 组 1 / 3：材料色号 `1..6`、组内不得重复，**两组的三个色号集合相同**（顺序可不同）。
- 组 2 / 4：环号 `1..3`，每组都是 1、2、3 的排列。
- 允许 QR 文本首尾 ASCII 空格、回车或换行（与 OpenMV `strip()` 一致），**不允许任何正文命令或其它字符**。
- 同帧多个二维码、无有效码、不符合格式的码会打断待确认计数，不会生成 `QR_CONFIRMED`。
- 屏幕会保留**上次已确认**任务码，但失去当前连续检测后明确显示 `NOT LIVE`，不会把旧码伪装成新抓到的码。

第二批在暂存区按第一批**同色**对应底座码垛，不能将第四组三位数当作暂存区码垛新底座位置。这一规则与原 OpenMV `make_plan()` 一致。

## 6. 实机验收清单

- [ ] 树莓派 5 实机能列出 CSI 摄像头，编译成功，稳定运行 ≥10 分钟且退出后摄像头资源释放。
- [ ] 连续多帧正常任务码能被确认，非法码、变码、重复色号/环号、多码同时出现均拒绝。
- [ ] 在比赛预期二维码板距离和角度下重复识别，记录成功率、误判率、首次稳定识别耗时、帧率与光照。
- [ ] 显示器实际能显示任务码，物理字高 ≥12mm、无遮挡、在场地光照下可读；显示内容与原始二维码一致。
- [ ] 扫码途中掉帧、遮挡或摄像头断线不得生成假确认；上一次结果与当前画面状态标识分开。
- [ ] 与 OpenMV/STM32 明确二维码**唯一权威来源**、启动触发、确认机制及后续比赛显示内容接口；本 QR Demo 不直接发布驱动指令。
- [ ] 显示屏及摄像头接线、电源、固定角度、机身总体尺寸符合比赛限制。

**当前状态**：已建立 C++ 模块与单元测试；尚未在实体 Pi5 摄像头和显示屏完成验收。
