# CSI 摄像头独立测试（Pi5）

OV5647 和 IMX219 是两种传感器，实际型号与编号以枚举结果为准。仅测试 CSI 采集与本地预览，不接入 QR、Material、ROS 或 STM32，不改系统配置，不安装任何包。Pi5 使用正确的 22-pin 转接排线，关机断电后插拔。停止其他使用摄像头的程序，再运行本测试。

依赖：Bash、GNU timeout、awk、已有 rpicam-hello/rpicam-still（旧系统 libcamera-hello/libcamera-still）；拍照校验使用 Python3 标准库。无需 OpenCV、Picamera2 或网络。

从仓库根目录运行：

```bash
bash algorithm/pi/tests/csi_camera_demo.sh list
bash algorithm/pi/tests/csi_camera_demo.sh capture 0
# 在树莓派实体屏幕终端执行，预览 30 秒：
bash algorithm/pi/tests/csi_camera_demo.sh preview 0 30
```

编号 0 仅为示例；双摄时必须按 list 结果明确选择编号，分别测试，不同时打开两路。只有一台时可省略编号。capture 保存到新建临时目录，输出路径并检查 JPEG 起止标记，不代表完整图像解码或画质合格。preview 默认 10 秒，最长 300 秒；Ctrl+C 可中断，外部 timeout 限制异常执行时间。预览命令返回 0 不能替代肉眼确认画面。

如果枚举失败，先检查排线方向、接头和型号，核对系统 camera_auto_detect / dtoverlay 设置；本脚本不自动修改启动配置。参考 https://www.raspberrypi.com/documentation/accessories/camera-software.html 。

软件回归：
```bash
bash -n algorithm/pi/tests/csi_camera_demo.sh
python3 algorithm/pi/tests/test_csi_camera_demo.py -v
```
模拟驱动测试覆盖单路/无设备/双路选择/JPEG 边界/参数/驱动失败。实际 OV5647 与 IMX219 双路、实体屏幕及异常中断验收待目标机执行；不要将模拟测试当实机成功。
