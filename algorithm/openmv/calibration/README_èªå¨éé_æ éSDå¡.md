# OpenMV H7 Plus 自动采集相机标定照片（无需 SD 卡）

## 放在哪里

将本压缩包中的 `demo/calibration/` 目录合并到**你现有的** `demo/calibration/`，不要重命名你的 `demo`，不覆盖 `demo/src_openmv/main.py`、`config.py`、`utils.py`。

## 工作原理

电脑 Python 程序通过 OpenMV 官方 USB 主机接口临时执行摄像头采图代码（RAM 中运行，不写相机 `main.py`），从 USB 获取 320×240 图像。OpenCV 在电脑上检测 9×6 **内角点**，预览中自动标记，检测通过才保存。

- `demo/calibration/images/cal_001.png`：**无任何标注**的原始照片，供标定程序使用。
- `demo/calibration/marked/cal_001.png`：绘制了角点的检查照片，**不要**用于相机标定。
- 自动过滤暗画面、角点不全、棋盘格过小/边缘超界、过于模糊，以及太相近的重复视角。
- 默认保存 30 张，完成后自动退出；按 `Q` / `Esc` 或 `Ctrl+C` 可提前结束。

## 运行条件

电脑 Python 需要 **3.12 或更新**。`openmv` 官方主机包要求这个版本；OpenMV 板上不用安装任何包。安装：

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install openmv opencv-python numpy
```

若没有 `python3.12`，推荐使用 `uv`：

```bash
# 仅在 uv 未安装时，先按 uv 官方网站的安装说明安装 uv
uv run --python 3.12 01_auto_capture_pc.py --port /dev/ttyACM0 --count 30
```

脚本自带 uv 依赖声明，uv 会按需安装 Python 3.12 与所需软件包；不用改变系统 Python。Ubuntu 默认 `python3` 可能低于 3.12，检查 `python3 --version`。不要为了安装 Python 删除原有 ROS/系统 Python。

## 启动步骤

1. 用 USB 数据线连接 OpenMV H7 Plus，**完全断开 OpenMV IDE 与相机的连接**（建议直接关掉 IDE）。同一个 `/dev/ttyACM0` 不能同时被两端软件占用。
2. 终端输入 `ls /dev/ttyACM*` 看设备名称。Ubuntu 通常为 `/dev/ttyACM0`；Windows 通常为 `COM3` 等。
3. 进入 `demo/calibration` 文件夹并运行：

```bash
python 01_auto_capture_pc.py --port /dev/ttyACM0 --count 30
```

如需要换到其他路径保存：

```bash
python 01_auto_capture_pc.py --port /dev/ttyACM0 --images /home/你的用户名/demo/calibration/images --marked /home/你的用户名/demo/calibration/marked
```

运行后新弹出的 `OpenMV calibration` 窗口会实时显示画面，并在识别成功时描绘棋盘格角点。每移动或倾斜棋盘格并稍停片刻，自动保存一张。请从不同角度、远近、左右、四角及中间区域各采一些，采集 20–35 张。

采集时请用之前给你的 **10×7 方格、9×6 内角点、每格 20 mm** 的标定板，打印 100% 原尺寸。尽量保证完整可见、光照均匀、无反光、清晰对焦。

## 采集完成后计算内参

```bash
python 02_calibrate_pc.py --images images --square-mm 20
```

获得 `camera_intrinsics.json` 后再回填你原有 `demo/src_openmv/config.py` 的 `FX_PX` 等参数；不要直接修改相机上的主项目程序。

## 常见问题

- **一直显示 TOO DARK / LOW CONTRAST**：你上次截图的 OpenMV Frame Buffer 几乎全黑。首先把相机朝向亮白纸，在电脑预览里确认有正常画面，检查镜头遮挡、现场照明、自动曝光等问题。单靠保存程序无法修复全黑图像。
- **NO COMPLETE 9x6 BOARD**：必须看到完整棋盘格；检查是不是实际 10 列×7 行方格，背景不能占掉太多画面。
- **MOVE / TILT BOARD**：是去重机制，倾斜/旋转或改变距离和位置再保存；不要停在同一姿态反复取样。
- **Device busy / Permission denied**：关闭 IDE；Linux 检查 `/dev/ttyACM0` 权限，可给当前账号加入 `dialout`（重新登录生效）。
- **找不到 `python3.12`**：先装 Python 3.12+。不建议改变 Ubuntu 的系统 Python 版本。
- **没有 GUI / 远程 SSH**：添加 `--no-preview`，只能在终端看状态；图像仍自动保存在文件夹里。
- **关闭后仍有图**：程序只在电脑硬盘上存照片，已保存的照片不会被删；OpenMV 内部 `main.py` 没有被修改。

## 软件验证方法（非设备实测）

可使用现有 320×240 的棋盘格照片离线验证自动识别和保存：

```bash
python 01_auto_capture_pc.py --test-images /path/to/example_images --no-preview --count 5
```

此模式不连接 OpenMV，不代表完成 USB/相机实机验收。
