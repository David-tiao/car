# Sim2Real — Smart Transport Robot Digital Twin (V13)

**启动：** `car` · **工作目录：** `~/Desktop/car/sim2real` · **访问：** `http://127.0.0.1:8080/`

## 设备状态

目前实体小车：STM32F4 + OpenMV H7 Plus。理想结构：STM32F4 + OpenMV + 树莓派 5（ROS2）+ 77° 摄像头 + 未来接入的激光雷达。**雷达和真实 `/odom` 通信尚需硬件联调**。

## 主要模式

- **模拟运行：** 5×5 蛇形编号、2 个障碍、A* / Dijkstra / D* Lite、二维码、转盘、抓取、伸缩机构、作业阶段动画。
- **实车实时运行：** ROS2 `/odom` 速度及轨迹、RViz 风格二维雷达地图、TF、Nav2 路径、OpenMV 和树莓派摄像头。没有反馈时显示离线。
- **人工训练（仅模拟）：** 开启后点击安全目标点行驶；W/A/S/D 平移、Q/E 转向、G 夹爪、R 伸缩、空格停车。不会将控制指令发给真正的车。

## 底盘调试与误差

在「实时轨迹误差 · 麦克纳姆完整调试台」中展开两大面板：

1. **实时偏差分析：** 利用已配准的 `/odom` 位置与规划折线，计算横向偏差 RMS、P95、最大值、航向偏差 RMS，并绘制曲线。参考速度差值不是闭环跟踪误差，也不能当作真实定位误差。
2. **完整 ULog43 调试台：** 原始 `kart_log_view_mecanum_v4(1).html` 的六个页面和工具均保留，支持历史 CSV 导入、回放、四轮、PID、波形、数据质量、通道标定、轨迹配准、停车分析；也可导入来自网页的具名列 `/odom` 记录。当前录制中没有的四轮或 PID 数据显示「未记录」，不补零伪造。

完整调试网页：`http://127.0.0.1:8080/tools/mecanum_debugger.html`。

## 理想数据流

```text
STM32F4 编码器/状态 → 树莓派 5 串口驱动（待对接）
  → ROS2 /odom + TF → 局域网 DDS → Ubuntu ROS2 桥接
  → /api/telemetry → 浏览器实车轨迹 / 误差分析 / CSV
OpenMV / Pi5 JPEG → HTTP 视频接口 → 浏览器实时双画面
雷达（后续） → /scan → ROS2 可视化桥接 → 网页激光图层
```

## Ubuntu 使用

```bash
# 终端 1：网页
car

# 终端 2：已接通 ROS2 后
cd ~/Desktop/car/sim2real
source /opt/ros/humble/setup.bash
bash start_ros_visualization.sh

# 状态检查
curl -s http://127.0.0.1:8080/api/build
curl -s http://127.0.0.1:8080/api/telemetry
```

**安全：** 实车模式为监控模式。网页人工训练不会向 STM32 或 `/cmd_vel` 发送控制；实体控制功能需后续审查硬件协议、紧急停止、通信失联自动停车、限速与赛事禁止遥控等要求。
