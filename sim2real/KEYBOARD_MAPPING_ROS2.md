# 映射集：电脑 ROS 2 → 树莓派 5 ROS 2 → STM32F4（安全开发基线）

## 当前完成程度

`http://127.0.0.1:8090/keyboard` 现在是可交互的**键盘映射与跟踪误差实验室**，不是空白占位页面。
浏览器与电脑 ROS 2 节点运行在**同一台 Ubuntu 电脑**，树莓派 5 运行 ROS 2 Humble：

```text
[电脑浏览器 /keyboard，按住 Shift + WASDQE]
                 |  localhost HTTP /api/keyboard/intent
                 v
[电脑 ROS2 keyboard_ros2_pc.py]
                 |  /car/teleop/desired_twist  geometry_msgs/TwistStamped
                 |  ROS_DOMAIN_ID 相同、双机 DDS 发现
                 v
[树莓派5 ROS2 keyboard_ros2_pi.py]
                 |  时间戳检查 / base_link / 速度限制 / 300ms 看门狗
                 v
[树莓派 ROS2 /car/teleop/stm32_candidate]
                 |  尚未实现、未授权 CAR-LINK 实体控制 TX
                 X
[STM32F4 真实运动接口：需要电控队友确定并实现]

[STM32 实际编码器/IMU → 串口上报 → 树莓派 ROS2 /odom]
                 |  ROS2 DDS 数据返回电脑（若网络/域已打通）
                 v
[电脑 ros2_visualization_bridge.py --web-channel real]
                 |  localhost HTTP /api/real/telemetry
                 v
[/keyboard 实测速度、残余停车速度与误差 RMS]
```

**本阶段不会驱动实体电机、不会发送实际 `/cmd_vel` 或 UART 帧。**
`/car/teleop/stm32_candidate` 只是一个未接物理设备的候选接口，不意味着
STM32 已接受指令。映射集主要用于先调试键位、电脑/树莓派 ROS2 网络、
消息延迟、里程计反馈与数据误差计算。其用途是赛前安全调试而非比赛时遥控。

## 键盘与车辆动作映射

使用 ROS REP-103 的 `base_link` 平面坐标：
- **W/S**：车体前进/后退（`linear.x` 正/负）。
- **A/D**：车体左移/右移（`linear.y` 正/负；麦轮可以横移）。
- **Q/E**：左转/右转（`angular.z` 正/负）。
- **Shift**：需要持续按住的使能键（deadman）。松开立即请求零速度。
- **空格 / Esc / 失焦 / 切换标签页**：停止并归零；服务端 300ms 无新指令也归零。
- **W+A** 等斜向组合会把平移速度归一化，避免超过滑块设定上限。
- **G 夹爪、R 伸缩**：暂只显示接口预留，**没有**真实机械动作请求，
  因为尚无对齐的 STM32 机械控制权、限位和完成反馈协议。

默认线速度 0.12 m/s，允许滑块范围 0.03–0.25 m/s；角速度默认
0.30 rad/s，允许滑块范围 0.05–0.50 rad/s。这只是**软件候选限幅**，
不能代替真实电控的电机输出限制、碰撞防护、现场人员安全和急停。

HTTP 请求强制 localhost + 同源；每个意图带单调递增 `client_seq`，
重置/松开后的迟到旧请求不能恢复运动。电脑端每 50ms 发布候选速度，
树莓派再进行独立限幅/ROS 时钟校验与 300ms 看门狗，超时归零。
真正连接 STM32 前须另行完成物理硬急停、独占控制权、上电禁止自动运动、
串口 ACK/执行反馈/控制超时及 UART TX 断电测试。

## 误差面板采用哪些数据？

- 期望速度 `(vx_d,vy_d,wz_d)`：根据键位和速度滑块生成的 **base_link**
  目标速度，**不是已下发 STM32 的实测目标**。
- 实际速度 `(vx_a,vy_a,wz_a)`：来自 `/api/real/telemetry` 中
  **REAL:ros2_odom** 且最新数据未超时的 ROS2 `nav_msgs/Odometry`。
- **严格核对**位姿 `header.frame_id=odom` 和速度 `child_frame_id=base_link`。
  若未连接、数据过时、处于 TEST/MOCK、坐标系不对或到来的数据比按键更旧，
  显示“不可比较”，不填造假的误差数字。

```text
Δvx = vx_d - vx_a
Δvy = vy_d - vy_a
Δω  = wz_d - wz_a
速度向量误差 e = sqrt(Δvx² + Δvy²)
RMS = sqrt(平均(e²))   // 只使用新增的有效 odom 样本，重复轮询不计数
```

当前网页以约 4Hz 拉取最新里程计进行比较，可能跳过 15Hz 或更快的
中间采样；该 RMS 是**网页采样 RMS**，不应冒充 STM32 控制周期的
高频跟踪误差。如需高频 PID/电机级测试，应在树莓派 ROS2 直接记录
每帧 `/odom`、`/car/chassis/wheels` 和底盘控制目标到 rosbag。

相对位置偏差是在同一次映射会话内，从首次有效里程计位置开始，
对**期望速度**进行积分，然后与随后里程计的 `x/y/yaw` 比较。
这是“指令积分参考 ↔ odom”的**相对轨迹偏差**，不是绝对定位准确度。
无真值定位时不可用其评价 SLAM / 地图坐标误差。
停车后会继续观察约 1.2 秒的**实际残余滑行**，便于测停车响应。
可通过“导出误差 CSV”保存每次有效样本。
误差曲线同一时间轴包含 `Δvx`、`Δvy`、`Δω`；
角速度单位不同，需要分别查看数值，不应将其直接当作距离误差。

## Ubuntu 电脑端运行

1. 先从最新开发分支同步代码并运行离线验收：

```bash
cd ~/Desktop/car
bash algorithm/car_ws/bash/test_keyboard_mapping.sh
```

2. 在电脑终端启动网站（**本机 localhost**）：

```bash
cd ~/Desktop/car
python3 sim2real/server.py --host 127.0.0.1 --port 8090
```

打开 `http://127.0.0.1:8090/keyboard`。

3. 电脑终端启动 ROS 2 命令候选发布器：

```bash
cd ~/Desktop/car
export ROS_DOMAIN_ID=42
bash algorithm/car_ws/bash/start_keyboard_pc_ros2.sh --server http://127.0.0.1:8090
```

4. 新开一个电脑终端检查发出的 ROS 2 话题：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic echo /car/teleop/desired_twist
```

先在网页选“本机启用键盘 ROS2 候选指令”，按住 Shift + W。
echo 中的 `linear.x` 应为 0.12，松 Shift 后变成 0；没有网页更新
超过 300ms 也应保持 0。

## 树莓派 5 端运行

树莓派需克隆同一版本的仓库、安装 ROS 2 Humble，并与电脑在同一
LAN，确认 ROS 2 DDS 可以发现彼此且 `ROS_DOMAIN_ID=42` 一致。
两台机器时钟要同步（chrony/NTP），否则 Pi 的时间戳检查将拒绝旧消息。
检查 Wi-Fi AP 是否开启客户端隔离、UDP Multicast 防火墙策略、
`ROS_LOCALHOST_ONLY` 不应为 1。

```bash
cd ~/Desktop/car
export ROS_DOMAIN_ID=42
bash algorithm/car_ws/bash/start_keyboard_pi_ros2.sh
```

在树莓派另一个终端：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic echo /car/teleop/stm32_candidate
ros2 topic echo /car/teleop/pi_guard_status
```

**成功标准：** 树莓派能看见候选消息，键位松开/电脑掉线后零速度，
`pi_guard_status` 明确输出 `uart_tx_enabled:false`。
不能将这些反馈解释为实体 STM32 正在运动。

## 实车里程计的只读展示

如果树莓派通过原有、**已确认匹配烧录固件**的 CAR-LINK 或 ULog43
读串口桥接发布了真实 `/odom`，且该话题经 ROS2 网络传到电脑，
在电脑额外终端启动：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
cd ~/Desktop/car
/usr/bin/python3 sim2real/ros2_visualization_bridge.py \
  --server http://127.0.0.1:8090 --web-channel real --odom /odom
```

该桥只读回传 `/odom` 到 REAL HTTP 通道，不会发速度控制。
**电脑到树莓派的话题流通并不保证反向 /odom 已存在。** 没有真实
STM32 里程计就显示 `NO_REAL_ODOM`，不得用网页模拟/TEST 值替代。

## 后续如何接上最后的 STM32F4？

需先与电控确认：
1. STM32F4 实际烧录固件支持哪一套串口命令（原版无树莓派写入口），
   CAR-LINK V1 只是树莓派侧的拟议协议，不能直接假定电控支持。
2. 物理 UART 口/电平、波特率、控制权归属、车体坐标映射/正负方向。
3. 速度命令及 STOP 帧编码、ACK、重发/去重/超时停车、急停硬件、
   上电默认锁定、主控失联以及相机/雷达仲裁。
4. 实物转轮空载小速度闭环，且有人员监护与急停；
   配合编码器、IMU 与 /odom 的速度和符号标定。

通过验收后再增加独立的受控 CAR-LINK TX 适配器。
**不要**把 `stm32_candidate` 直接 remap 到 `/cmd_vel` 或
未验证的串口发送函数。

## 本阶段软件验收

```bash
bash algorithm/car_ws/bash/test_keyboard_mapping.sh
```

运行纯 Python WASDQE/Shift/超时/旧命令拒绝、REAL 遥测隔离、
误差/停车滑行、坐标系校验与 HTTP 跨域测试，
并重跑四模式 HTTP 兼容测试。若安装 Node.js，检查键盘脚本语法。

此功能仍是**待 Ubuntu 验收的开发 PR**。即使测试通过，
也只能说明软件接口有效，不代表实体 STM32 控制闭环完成。
