# CSI 桶形畸变校正实验

仅测试目录，不修改 QR、Material、ROS 或 STM32。Brown–Conrady 五系数模型，使用 OpenCV initUndistortRectifyMap + remap；参数变化才重建映射。默认 k1=-0.25、k2=0、fx=fy=图像宽度，均为手动试调初值，不是 OV5647 标定结果。

依赖：已有 Python3、cv2、numpy（需 HighGUI）、rpicam-hello/rpicam-vid 或旧版 libcamera 工具；先检查，不自动安装。实体屏幕终端运行，先退出占用该相机的程序：

```bash
python3 -c 'import cv2,numpy; print(cv2.__version__,numpy.__version__)'
python3 algorithm/pi/tests/undistort_demo.py --self-test
python3 algorithm/pi/tests/undistort_demo.py --sensor ov5647 --device-path /base/axi/pcie@120000/rp1/i2c@88000/ov5647@36
```

型号唯一时可省略 --device-path；默认 640x480 30fps 为请求值，不是实测。原图/校正图并排，调 K1 (滑条值减60再除100)、K2 (减20再除100)、Zoom (除100，最小1)，右图显示实际系数和 remap 耗时。负 k1 用于初步校正桶形；过大可能导致非单调映射/边缘变形，须肉眼检查，不能用于测距。Q/Esc 退出，Ctrl+C/SIGTERM 回收所属进程组，帧等待超时10秒。静态测试可在有 Python OpenCV GUI 的电脑运行 --image /path/frame.jpg。

可选 --calibration 接收 JSON 对象，size 为 [width,height]、device_path 为枚举绝对身份、K 为 3x3 内参矩阵、D 为 [k1,k2,p1,p2,k3]。真实标定板图像和参数尚未采集；本实验不自动标定、不伪造参数。导入文件必须来自相同镜头焦距、分辨率及传感器裁剪模式。广角严重时需比较普通模型与鱼眼模型、独立验证边缘误差。

本地 Python 语法、四项合成映射检查及模拟驱动选路/错误路径/JPEG/退出清理检查通过；Pi5 Python 绑定、GUI、真实画面及实际帧率待验收。不得将手动校正视为度量标定。
