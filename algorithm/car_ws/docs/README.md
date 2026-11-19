# 工创智能搬运小车 ROS2 总体架构、数据流与话题设计文档

**项目：** `David-tiao/car`  
**版本：** V1.0  
**日期：** 2026-11-18  
**ROS2 版本：** Humble  
**开发语言：** C++17 / rclcpp  
**主工作空间：** `/home/tao/Desktop/car/algorithm/car_ws`  
**GitHub：** https://github.com/David-tiao/car

> 当前处于 ROS2 离线开发和模拟数据联调阶段，尚未接入真实 STM32、OpenMV、树莓派及小车运动反馈。本文区分已实现、已编译、待验证和待开发功能。

---

## 一、项目总体目标

本项目采用 **OpenMV H7 Plus + STM32F4 + 树莓派5 + YDLIDAR X3 + Ubuntu** 的分层架构。

最终实现以下闭环：

1. OpenMV 识别二维码、颜色、物料、数字圆环及工位坐标。
2. STM32F4 根据视觉结果控制底盘、机械手和比赛任务。
3. 树莓派5通过独立通信链路获取 STM32 的实时状态。
4. ROS2 将底盘、视觉、机构、任务、IMU 和雷达数据统一发布为 Topic。
5. 上层任务节点根据反馈协调导航与比赛流程。
6. Ubuntu 通过 RViz2、Sim2Real 查看轨迹、轮速、任务状态和故障信息。
7. 后续在安全条件满足时，允许 ROS2 向 STM32 发出经过授权的运动请求。

**设计原则：STM32 负责底层实时控制，ROS2 负责数据管理和上层任务。**

---

## 二、各硬件模块职责

| 模块 | 主要职责 | 开发语言 |
|---|---|---|
| OpenMV H7 Plus | 二维码、颜色、物料、圆环和工位识别 | MicroPython |
| STM32F4 | 四轮电机、编码器、PID、IMU、机械控制、比赛状态机 | C / HAL |
| 树莓派5 | ROS2 通信、消息处理、诊断、任务协调、导航 | C++17 |
| YDLIDAR X3 | 二维激光扫描、障碍信息 | ROS2 驱动 |
| Ubuntu 电脑 | 软件开发、离线调试、RViz2、Sim2Real | ROS2 / C++ |
| Sim2Real | 轨迹、状态和仿真数据可视化 | 独立监控系统 |

说明：

- ROS2 执行节点统一采用 C++。
- OpenMV 独立运行，不加入 `car_ws` 编译。
- STM32 保留已有 PID 和底盘控制代码。
- ROS2 自定义消息使用 `.msg` 定义，由 ROSIDL 生成 C++ 接口。
- Launch 文件可以使用 XML，不要求用 C++ 编写启动描述。

---

## 三、最终系统数据流

```text
                 OpenMV H7 Plus
            二维码 / 物料 / 圆环识别
                        │
                 USART3 115200
                        │
                        ▼
                     STM32F4
       ┌────────────────┼────────────────┐
       │                │                │
    四轮电机PID      编码器/IMU       机械机构
       │                │                │
       └────────────────┼────────────────┘
                        │
                 比赛状态机
                        │
                独立通信接口
                   （待开发）
                        │
                        ▼
                 树莓派5 ROS2
                        │
                car_stm32_bridge
                        │
          ┌─────────────┼──────────────┐
          │             │              │
          ▼             ▼              ▼
        /odom      /car/chassis/*  /car/mission/*
          │             │              │
          └─────────────┼──────────────┘
                        │
                  car_debug
                 状态监控/诊断
                        │
                  ROS2 DDS
                        │
                        ▼
                 Ubuntu 电脑
                RViz2 / Sim2Real

  YDLIDAR X3 ──> /scan ──> SLAM / Nav2
```

**注意：** STM32 与树莓派之间的独立通信接口目前尚未实现。上图是最终目标架构。

### 3.1 当前已经实现的软件数据流

```text
模拟 ULog43 CSV
       │
       ▼
car_stm32_bridge
 C++ 解析、单位转换
       │
       ├── /odom
       ├── /joint_states
       ├── /car/chassis/telemetry
       ├── /car/chassis/wheels
       ├── /car/mission/state
       └── /car/debug/ulog43
                  │
                  ▼
              car_debug
                  │
                  ├── /car/debug/trajectory
                  └── /car/debug/diagnostics
                  │
                  ▼
               RViz2
```

目前四个自研 ROS2 包已经编译成功，桥接节点已能启动模拟 CSV 回放。

但 `/odom` 等话题是否完整接收到数据，仍需通过 ROS2 命令验证。

---

## 四、ROS2 工程目录

```text
/home/tao/Desktop/car/
│
├── STM32F4/
│   ├── App/
│   ├── Control/
│   ├── Hardware/
│   ├── Config/
│   ├── Core/
│   └── MDK-ARM/
│
├── algorithm/
│   │
│   ├── openmv/
│   │   ├── main.py
│   │   ├── tools.py
│   │   ├── serial.py
│   │   └── config.py
│   │
│   └── car_ws/
│       │
│       ├── src/
│       │   ├── car_interfaces/
│       │   ├── car_stm32_bridge/
│       │   ├── car_debug/
│       │   ├── car_bringup/
│       │   ├── ydlidar_ros2/
│       │   ├── ydlidar_ros2_driver/
│       │   ├── slam_gmapping/
│       │   └── openslam_gmapping/
│       │
│       ├── maps/
│       ├── config/
│       ├── behavior_trees/
│       ├── rviz/
│       ├── docs/
│       ├── build/
│       ├── install/
│       └── log/
│
└── sim2real/
```

### 目录管理原则

- `car_ws` 是唯一 ROS2 主工作空间。
- 所有新 ROS2 包放在 `car_ws/src/`。
- 使用 `colcon build` 统一编译。
- 旧 `ydlidar_ws` 已迁出项目，保存在本地备份目录。
- `build/`、`install/`、`log/` 不提交 GitHub。
- `maps/COLCON_IGNORE` 用于排除历史地图工程。
- `ydlidar_ros2_driver/COLCON_IGNORE` 暂时排除备用雷达驱动。
- OpenMV 和 STM32 固件独立管理，不参加 ROS2 编译。

---

## 五、ROS2 功能包总表

| 功能包 | 职责 | 当前状态 |
|---|---|---|
| `car_interfaces` | 统一自定义消息类型 | 已编译 |
| `car_stm32_bridge` | STM32 数据解析与 ROS2 发布 | 已编译、已启动 |
| `car_debug` | 轨迹、诊断、数据监控 | 已编译 |
| `car_bringup` | 统一启动及 RViz 配置 | 已编译 |
| `ydlidar` | YDLIDAR X3 驱动 | 已编译 |
| `openslam_gmapping` | GMapping 算法库 | 待验证 |
| `slam_gmapping` | GMapping 建图节点 | 待验证 |
| `car_vision` | 视觉数据处理 | 待开发 |
| `car_mechanism` | 机械机构状态处理 | 待开发 |
| `car_mission` | 上层比赛任务协调 | 待开发 |
| `car_navigation` | 地图、定位和导航 | 待开发 |
| `car_base` | 底盘接口与状态融合 | 待开发 |

注意：

`car_interfaces` 只定义消息，不发布 Topic。

`car_bringup` 负责统一启动，不直接控制底盘。

未来新增 `car_base` 时，必须避免和 `car_stm32_bridge` 重复发布同一个 `/odom` 或 TF。

---

## 六、ROS2 话题总表

### 6.1 已编写的话题

| Topic | 类型 | 发布者 | 功能 |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | `car_stm32_bridge` | 位置、速度、航向 |
| `/joint_states` | `sensor_msgs/JointState` | `car_stm32_bridge` | 四轮关节速度 |
| `/car/chassis/telemetry` | `ChassisTelemetry` | `car_stm32_bridge` | 完整底盘遥测 |
| `/car/chassis/wheels` | `WheelTelemetry` | `car_stm32_bridge` | 四轮目标与实际速度 |
| `/car/mission/state` | `MissionState` | `car_stm32_bridge` | STM32 原始比赛状态 |
| `/car/debug/ulog43` | `Ulog43` | `car_stm32_bridge` | 原始43通道数据 |
| `/car/debug/trajectory` | `nav_msgs/Path` | `car_debug` | 历史运动轨迹 |
| `/car/debug/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `car_debug` | 数据健康、误差及状态 |
| `/tf` | `tf2_msgs/TFMessage` | 可选 TF 广播器 | 坐标变换，当前默认关闭 |

上述话题有发布代码，不代表已经完成全部运行验收。

### 6.2 待开发或待接入话题

| Topic | 类型 | 用途 |
|---|---|---|
| `/car/vision/event` | `VisionEvent` | OpenMV 视觉识别结果 |
| `/car/mechanism/state` | `MechanismState` | 夹爪、升降、转盘状态 |
| `/imu/data` | `sensor_msgs/Imu` | IMU 姿态与惯性数据 |
| `/tf_static` | `tf2_msgs/TFMessage` | 传感器固定坐标变换 |
| `/scan` | `sensor_msgs/LaserScan` | 激光雷达扫描 |
| `/map` | `nav_msgs/OccupancyGrid` | 静态地图或 SLAM 地图 |
| `/cmd_vel` | `geometry_msgs/Twist` | 未来底盘速度请求 |
| `/car/control/state` | 待定 | 控制模式与控制权状态 |
| `/car/mission/summary` | 待定 | ROS2 上层任务汇总 |
| `/plan` | 由 Nav2 配置决定 | 全局规划路径 |
| 局部规划话题 | 由 Nav2 配置决定 | 局部轨迹及控制信息 |

**已定义消息不等于 Topic 已有实时发布者。**

---

## 七、`car_interfaces` 自定义消息

第一阶段定义六种消息：

```text
car_interfaces/msg/
├── ChassisTelemetry.msg
├── WheelTelemetry.msg
├── MissionState.msg
├── Ulog43.msg
├── VisionEvent.msg
└── MechanismState.msg
```

### 7.1 ChassisTelemetry

用于底盘位置、速度、目标值和误差。

| 字段 | 含义 |
|---|---|
| `header` | ROS2 时间及坐标系 |
| `source_time_ms` | STM32 开机运行毫秒 |
| `chassis_mode` | 速度模式或位置模式 |
| `target_reached` | 是否到达目标 |
| `x_m`, `y_m` | 里程计位置，米 |
| `yaw_rad` | 航向角，弧度 |
| `vx_mps`, `vy_mps` | 实际车体速度 |
| `wz_radps` | 实际角速度 |
| `wheel_estimated_wz_radps` | 轮速估计角速度 |
| `target_x_m`, `target_y_m` | 目标位置 |
| `target_yaw_rad` | 目标航向 |
| `target_wz_radps` | 目标角速度 |
| `target_speed_mps` | 目标平移速度模长 |
| `target_linear_components_valid` | 目标速度分量是否可靠 |
| `x_target_error_m` | X 位置误差 |
| `y_target_error_m` | Y 位置误差 |
| `yaw_target_error_rad` | 航向误差 |
| `lateral_target_error_m` | 车体系横向目标误差 |
| `driver_voltage_v` | 驱动侧电压 |
| `board_voltage_v` | 主板电压 |
| `travelled_distance_m` | 累计里程 |
| `imu_yaw_rad` | IMU 航向 |
| `imu_valid` | IMU 有效标志 |

当前 ULog43 中，`ch10–13` 的整数输出会损失低速精度，因此不能可靠恢复目标 X/Y 速度分量。

### 7.2 WheelTelemetry

用于四轮电机控制和反馈分析。

| 字段 | 含义 |
|---|---|
| `target_pulse_per_s[4]` | 四轮目标脉冲速度 |
| `actual_pulse_per_s[4]` | 四轮实测脉冲速度 |
| `target_mps[4]` | 四轮目标线速度 |
| `actual_mps[4]` | 四轮实际线速度 |
| `max_abs_pid_output` | 最大 PID 输出绝对值 |
| `per_wheel_pid_output_valid` | 是否有每轮独立 PID 输出 |
| `encoder_counts_valid` | 是否有累计编码器计数 |

轮序固定：

```text
0 = FL 左前
1 = FR 右前
2 = RL 左后
3 = RR 右后
```

### 7.3 MissionState

记录 STM32 原始比赛状态。

主要字段：

- `phase_code`：运行阶段。
- `status_code`：比赛状态码。
- `action_index`：当前动作序号。
- `error_code`：错误码。
- `plan_index`：任务计划索引。
- `plan_done`：任务是否完成。
- `race_state_semantics_known`：是否确认当前固件运行在比赛模式。

未来 `car_mission` 不应重复发布同名话题。

### 7.4 Ulog43

保存原始 STM32 调试日志。

主要字段：

- `header`
- `source_time_ms`
- `channels[43]`
- `valid_semantics[43]`
- `original_line`

作用是保留原始数据，便于和 STM32 固件、网页调试台及 CSV 文件对照。

### 7.5 VisionEvent

用于统一视觉识别结果。

主要字段：

- `event_type`：事件类型。
- `raw_line`：原始视觉串口消息。
- `task_code`：二维码任务码。
- `step`：任务步骤。
- `token`：观测令牌。
- `operation`：PICK、PLACE、STACK、HOME。
- `station`：RAW、ROUGH、STORE、HOME。
- `color_id`：颜色编号。
- `ring_id`：圆环编号。
- `pixel_u`、`pixel_v`：像素坐标。
- `radius_px`：目标半径。
- `pixel_position_valid`：像素信息是否有效。
- `metric_position_valid`：米制坐标是否经过标定。
- `state_name`：视觉任务状态。

**当前只有消息定义，没有真实视觉发布节点。**

### 7.6 MechanismState

用于机械机构遥测。

主要字段：

- `gripper_angle_deg`：夹爪角度。
- `turntable_angle_deg`：转盘角度。
- `lift_height_mm`：升降高度。
- `extension_mm`：伸缩距离。
- `operation_active`：动作执行状态。
- `fault_code`：机构错误码。
- 各机构对应的有效性标志。

当前 STM32 ULog43 不包含这些完整机构反馈，因此不能简单填零当作真实测量。

---

## 八、STM32 底盘数据来源

对应 GitHub：

```text
STM32F4/
├── Control/
│   ├── Motor.c
│   ├── Motor.h
│   ├── mecanum_chassis.c
│   ├── mecanum_chassis.h
│   ├── nav.c
│   └── nav.h
├── Hardware/
│   ├── bsp_ulog.c
│   ├── bsp_ulog.h
│   ├── vision.c
│   └── app_imu.c
└── App/
    ├── app_runtime.c
    ├── app_race.c
    └── race_fsm.c
```

### 8.1 底盘坐标约定

- X：车头前方为正。
- Y：小车左侧为正。
- Z 轴旋转：逆时针为正。
- 轮序：FL、FR、RL、RR。
- 位置单位：m。
- 线速度单位：m/s。
- 角速度单位：rad/s。
- 编码器原始速度单位：pulse/s。

### 8.2 STM32 当前参数

| 参数 | 当前值 |
|---|---|
| 轮半径 | 0.040 m |
| 半轴距 | 0.093 m |
| 半轮距 | 0.110 m |
| 转动系数 L | 0.203 m |
| 减速比配置 | 1.0 |
| 编码器每轮转脉冲 | 440 |
| ULog 脉冲换算常量 | 1750 pulse/m |
| ULog 周期 | 50 ms |

这些是仓库当前软件配置，后续仍需实车标定。

### 8.3 麦克纳姆逆运动学

已知底盘目标：

- `vx`：前后速度。
- `vy`：左右速度。
- `ω`：旋转角速度。

四轮线速度：

```text
v_FL = vx - vy - L*ω
v_FR = vx + vy + L*ω
v_RL = vx + vy - L*ω
v_RR = vx - vy + L*ω
```

其中：

```text
L = 半轴距 + 半轮距 = 0.203m
```

转换为编码器脉冲速度：

```text
wheel_pps = wheel_mps * pulses_per_meter
```

### 8.4 麦克纳姆正运动学

通过四轮速度计算底盘速度：

```text
vx = (v_FL + v_FR + v_RL + v_RR) / 4

vy = (-v_FL + v_FR + v_RL - v_RR) / 4

ω  = (-v_FL + v_FR - v_RL + v_RR) / (4L)
```

ROS2 桥接使用四轮实测脉冲速度恢复车体线速度。

### 8.5 PID 控制分工

STM32 负责：

1. 四轮电机速度 PID。
2. 位置 X PID。
3. 位置 Y PID。
4. 航向角 PID。
5. 编码器采集。
6. IMU 辅助位姿估计。
7. 实际 PWM 控制。

ROS2 当前不重复执行电机 PID。

后续需要分析的误差：

```text
速度误差 = 目标速度 - 实际速度

X位置误差 = 目标X - 当前X

Y位置误差 = 目标Y - 当前Y

航向误差 = 目标角度 - 当前角度
```

---

## 九、STM32 ULog43 全部通道

来源：`STM32F4/Hardware/bsp_ulog.c`

ULog43 每帧包含 43 个数值，编号为 `ch0–ch42`。

| 通道 | 含义 | 单位或说明 |
|---|---|---|
| ch0 | 底盘控制模式 | 0 速度 / 1 位置 |
| ch1 | 运动阶段 | 空闲、移动、转向、停稳、完成 |
| ch2 | 比赛/测试状态 | 依运行模式解释 |
| ch3 | 动作索引 | 整数 |
| ch4 | 到达目标标志 | 0/1 |
| ch5 | STM32 运行时间 | ms |
| ch6 | 错误码 | 整数 |
| ch7 | 任务计划索引 | 整数 |
| ch8 | 阶段副本 | 同 ch1 |
| ch9 | 任务完成标志 | 0/1 |
| ch10 | 实际 vx | m/s，输出精度低 |
| ch11 | 实际 vy | m/s，输出精度低 |
| ch12 | 目标 vx | m/s，输出精度低 |
| ch13 | 目标 vy | m/s，输出精度低 |
| ch14 | 目标角速度 | rad/s |
| ch15 | 实际角速度 | rad/s |
| ch16 | 底盘航向 | deg |
| ch17 | IMU 航向 | deg |
| ch18 | 目标角速度 | deg/s |
| ch19 | 目标速度缩放值 | 需乘 ULOG_PV |
| ch20 | 实际速度缩放值 | 需乘 ULOG_PV |
| ch21 | 目标 X | m |
| ch22 | 目标 Y | m |
| ch23 | 目标航向 | deg |
| ch24 | X 位置误差 | m |
| ch25 | Y 位置误差 | m |
| ch26 | 航向误差 | deg |
| ch27 | 左前轮目标速度 | pulse/s |
| ch28 | 左前轮实际速度 | pulse/s |
| ch29 | 右前轮实际速度 | pulse/s |
| ch30 | 左后轮实际速度 | pulse/s |
| ch31 | 右后轮实际速度 | pulse/s |
| ch32 | 里程计 X | m |
| ch33 | 里程计 Y | m |
| ch34 | 累计行驶距离 | m |
| ch35 | 右前轮目标速度 | pulse/s |
| ch36 | 左后轮目标速度 | pulse/s |
| ch37 | 右后轮目标速度 | pulse/s |
| ch38 | 四轮最大 PID 输出绝对值 | 聚合值 |
| ch39 | 车体系横向目标误差 | m |
| ch40 | 驱动侧电压 | V |
| ch41 | 主板电压 | V |
| ch42 | 航向目标误差 | deg |

### 9.1 两个特别注意事项

**问题一：速度精度**

STM32 当前 `snprintf` 对 `ch10–13` 使用 `%.0f`。

因此低速数据可能被记录为零，不能直接用于精确速度监控。

当前 C++ 桥接使用：

```text
ch28、ch29、ch30、ch31
```

恢复四轮实际速度，再计算底盘线速度。

**问题二：目标速度**

`ch19` 是目标平移速度模长的缩放值，而不是某一个方向的速度。

```text
target_speed_mps = ch19 * 0.073632
```

不能单凭该数值恢复目标速度方向。

### 9.2 ULog43 没有提供的数据

- 四轮分别的完整 PID 输出。
- 四轮累计编码器计数。
- IMU 三轴加速度和完整角速度。
- 视觉二维码和原始像素观测。
- 机械手精确实际位置。
- 通信链路 CRC 和可靠 ACK。
- 完整相机标定与雷达数据。

上述信息需要未来新增 STM32 遥测协议才能获得。

---

## 十、OpenMV 视觉模块

OpenMV 位于：

```text
algorithm/openmv/
├── main.py
├── tools.py
├── serial.py
└── config.py
```

### 10.1 文件职责

| 文件 | 职责 |
|---|---|
| `main.py` | 主程序及视觉任务状态机 |
| `tools.py` | 图像处理、二维码、颜色、物料、圆环识别 |
| `serial.py` | 串口数据发送与命令解析 |
| `config.py` | 阈值、像素参数、UART 配置 |

### 10.2 视觉处理流程

```text
相机采集图像
      │
      ▼
图像预处理
      │
      ├── 二维码识别
      ├── 颜色识别
      ├── 物料识别
      ├── 数字圆环识别
      └── 工位位置识别
      │
      ▼
多帧确认 / 目标稳定性判断
      │
      ▼
串口发送视觉结果
      │
      ▼
STM32 比赛任务状态机
```

### 10.3 当前 OpenMV 串口参数

```text
UART3
115200 baud
8 data bits
No parity
1 stop bit
```

相机分辨率：

```text
320 × 240
RGB565
```

图像中的 `u/v` 表示像素坐标，不代表真实米制坐标。

### 10.4 OpenMV V5 风格消息

| 消息 | 作用 |
|---|---|
| `#SEQ` | 发送二维码任务码 |
| `#DISK` | 反馈转盘是否可见 |
| `#SLOT` | 转盘抓取位置 |
| `#OBJ` | 物料视觉信息 |
| `#ARM_OK` | 授权确认 |
| `#TAKE_OK` | 取料开始确认 |
| `#AT_OK` | 工位到达确认 |
| `#OBS` | 稳定视觉观测 |
| `#USE_OK` | 观测取用确认 |
| `#RESULT_OK` | 动作结果登记 |
| `#LOST` | 观测丢失 |
| `#STATE` | 任务状态 |
| `#ERR` | 错误信息 |

典型视觉数据：

```text
#OBS,22,17,STACK,STORE,5,2,158,122
```

含义：

- 当前步骤：22。
- 观测令牌：17。
- 动作：STACK。
- 工位：STORE。
- 颜色：5。
- 圆环：2。
- 像素坐标：158、122。

### 10.5 OpenMV 与 STM32 当前协议差异

目前仓库中存在两个不同的协议体系。

**OpenMV：**

```text
#SEQ,...
#OBS,...
#USE,...
#RESULT,...
```

**STM32 V4：**

```text
@sequence,length,body*XX
```

STM32 V4 协议包括序号、长度和 XOR 校验。

STM32 仅保留部分旧二维码兼容处理，不等于已经兼容全部 V5 视觉交互。

因此后续需要：

1. 确定唯一正式 OpenMV 固件版本。
2. 统一视觉串口帧格式。
3. 对齐消息和步骤编号。
4. 增加完整 ACK。
5. 增加超时和观测失效处理。
6. 将统一后的视觉状态转发给 ROS2。

视觉协议没有完成统一之前，不应直接视为实车视觉闭环已经跑通。

---

## 十一、STM32 比赛状态机

源码：

```text
STM32F4/App/race_fsm.h
STM32F4/App/race_task_plan.h
STM32F4/App/app_race.c
```

### 11.1 RaceState

| 数值 | 状态 |
|---|---|
| 0 | RACE_READY |
| 1 | RACE_RUNNING |
| 2 | RACE_DONE |
| 3 | RACE_FAULT |
| 4 | RACE_ABORTED |

该表仅在 STM32 实际运行 `APP_MODE_RACE` 时适用。

### 11.2 RaceError

| 数值 | 错误 |
|---|---|
| 0 | RACE_ERROR_NONE |
| 1 | RACE_ERROR_CONFIG |
| 2 | RACE_ERROR_MOVE_TIMEOUT |
| 3 | RACE_ERROR_SCAN_TIMEOUT |
| 4 | RACE_ERROR_IMU |
| 5 | RACE_ERROR_LINK |
| 6 | RACE_ERROR_LOOP_STALL |

### 11.3 比赛动作

底盘动作：

```text
MOVE
TURN
SETTLE
SCAN
```

任务动作：

```text
PICK
GO_ROUGH
PLACE_ROUGH
RETRIEVE_ROUGH
GO_STORAGE
PLACE_STORAGE
GO_MATERIAL
STACK_STORAGE
RETURN_START
```

未来的 `car_mission` 负责汇总和协调任务状态。

但当前 STM32 已有自己的比赛状态机，因此 ROS2 不应同时发出另一组冲突的底层动作。

---

## 十二、C++ STM32 桥接节点

功能包：

```text
car_stm32_bridge
```

当前可执行节点：

```text
car_ulog43_bridge
```

### 12.1 已实现功能

1. 读取 ULog43 CSV。
2. 检查每行是否具有 43 列。
3. 检查非法数值和时间戳。
4. 转换轮速与位置单位。
5. 重建底盘实际速度。
6. 生成 `/odom`。
7. 生成四轮遥测。
8. 发布 STM32 任务状态。
9. 发布原始43通道数据。
10. 支持离线回放及只读串口输入。

### 12.2 桥接参数

| 参数 | 默认值 | 用途 |
|---|---|---|
| `replay_file` | 空 | CSV 回放文件 |
| `port` | 空 | 只读串口 |
| `baud` | 115200 | 串口波特率 |
| `replay_interval_ms` | 50 | 回放间隔 |
| `pulses_per_meter` | 1750 | 脉冲转米 |
| `wheel_ppr` | 440 | 每轮转脉冲 |
| `wheelbase_half_m` | 0.093 | 半轴距 |
| `track_half_m` | 0.110 | 半轮距 |
| `publish_tf` | false | 是否广播 TF |
| `publish_joint_states` | true | 是否发布轮子关节速度 |

### 12.3 `/odom` 的数据来源

| 里程计字段 | STM32 数据 |
|---|---|
| `pose.position.x` | ch32 |
| `pose.position.y` | ch33 |
| `pose.orientation` | ch16 转弧度后计算 |
| `twist.linear.x` | 四轮实测速度重建 |
| `twist.linear.y` | 四轮实测速度重建 |
| `twist.angular.z` | ch15 |

坐标约定：

```text
header.frame_id = odom
child_frame_id  = base_link
```

当前协方差未进行实车标定，不应当作精确定位数据。

### 12.4 `/joint_states`

发布四轮角速度。

关节命名：

```text
wheel_fl_joint
wheel_fr_joint
wheel_rl_joint
wheel_rr_joint
```

由于当前 CSV 不包含累计编码器计数，所以只提供可靠范围内的速度信息，不提供真实累计轮角度。

---

## 十三、C++ 调试与诊断节点

功能包：

```text
car_debug
```

节点：

```text
car_data_monitor
```

### 13.1 输入话题

```text
/odom
/car/chassis/telemetry
/car/chassis/wheels
/car/mission/state
/car/debug/ulog43
```

### 13.2 输出话题

```text
/car/debug/trajectory
/car/debug/diagnostics
```

### 13.3 诊断内容

- 数据是否持续到达。
- 各话题接收帧数。
- 上次接收时间。
- 数据是否超时。
- 四轮目标速度与实际速度差值。
- 底盘当前运动状态。
- 比赛任务状态。
- 错误码。
- 电压。
- 历史轨迹点数。

默认参数：

| 参数 | 默认值 |
|---|---|
| `stale_after_ms` | 1000 |
| `max_path_points` | 1000 |
| `path_publish_stride` | 4 |
| `wheel_error_warn_pps` | 350 |

注意：离线 CSV 播放结束后，诊断出现 `STALE` 是正常现象，不代表真实 STM32 故障。

---

## 十四、ROS2 统一启动

功能包：

```text
car_bringup
```

启动文件：

```text
offline_debug.launch.xml
```

负责启动：

```text
car_ulog43_bridge
car_data_monitor
RViz2（可选）
```

### 启动流程

```text
car_bringup
     │
     ├── car_stm32_bridge
     │       │
     │       └── 发布底盘、轮速、任务状态
     │
     ├── car_debug
     │       │
     │       └── 发布轨迹与诊断
     │
     └── RViz2
             │
             └── 可视化
```

当前不会启动电机控制、机械机构控制或真实激光雷达。

---

## 十五、ROS2 TF 坐标系规划

后续推荐坐标树：

```text
map
 └── odom
      └── base_link
           ├── laser_frame
           ├── camera_frame
           ├── wheel_fl_link
           ├── wheel_fr_link
           ├── wheel_rl_link
           └── wheel_rr_link
```

各坐标系作用：

| 坐标系 | 功能 |
|---|---|
| `map` | 地图坐标 |
| `odom` | 连续里程计坐标 |
| `base_link` | 机器人底盘坐标 |
| `laser_frame` | 雷达坐标 |
| `camera_frame` | 相机坐标 |
| `wheel_*_link` | 轮子位置 |

### TF 设计原则

- `map → odom`：由未来定位或 SLAM 系统管理。
- `odom → base_link`：由唯一里程计权威节点发布。
- `base_link → laser_frame`：由雷达实际安装尺寸决定。
- `base_link → camera_frame`：由相机实际安装位置决定。
- 固定几何参数后续写入 URDF。
- 同一段 TF 禁止多个节点同时发布。

目前完整 TF 尚未实现。

---

## 十六、YDLIDAR 与导航

### 16.1 YDLIDAR

当前使用的包：

```text
car_ws/src/ydlidar_ros2
```

实际 ROS2 包名：

```text
ydlidar
```

输出话题：

```text
/scan
```

类型：

```text
sensor_msgs/msg/LaserScan
```

当前驱动已编译成功，但尚未验证真实雷达数据。

### 16.2 地图

地图位于：

```text
car_ws/maps/
```

未来通过地图服务器发布：

```text
/map
```

类型：

```text
nav_msgs/msg/OccupancyGrid
```

需要统一：

- 地图尺寸。
- 地图分辨率。
- 地图原点。
- 禁行区域。
- 雷达外参。
- 机器人底盘尺寸。
- TF 坐标关系。

### 16.3 Nav2

未来主要使用：

```text
/odom
/tf
/tf_static
/scan
/map
/cmd_vel
```

Nav2 需要：

1. 机器人当前位姿。
2. 全局地图。
3. 障碍物信息。
4. 规划器。
5. 控制器。
6. 可执行的底盘控制接口。

当前 STM32 尚未提供经过验证的 ROS2 速度控制接口，因此不能直接将 `/cmd_vel` 接入真实底盘。

---

## 十七、STM32 与树莓派通信规划

### 17.1 当前串口占用

| STM32 接口 | 用途 | 状态 |
|---|---|---|
| USART3 | OpenMV 视觉通信 | 已由固件使用 |
| USART2 | ULog43 → microSD 记录器 | 已由固件使用 |
| 树莓派专用接口 | ROS2 实时通信 | 待设计 |

USART3 对应：

```text
PD8 = TX
PD9 = RX
115200 8N1
```

USART2 对应：

```text
PD5 = TX
PD6 = RX
115200 8N1
```

不应未经验证直接让树莓派占用或并联上述通信接口。

### 17.2 未来串口协议

建议通信帧包含：

```text
帧头
协议版本
消息类型
数据长度
序列号
STM32 时间戳
数据内容
CRC 校验
```

建议数据类型：

```text
CHASSIS_STATE
WHEEL_STATE
IMU_STATE
VISION_EVENT
MECHANISM_STATE
MISSION_STATE
HEARTBEAT
ERROR
```

后续可扩展：

```text
CMD_VEL
CMD_STOP
CMD_ENABLE
CMD_MISSION
CMD_ACK
```

但控制消息必须在完成控制权仲裁和安全测试后启用。

### 17.3 安全机制

- 默认只读。
- 运动控制权唯一。
- 通信超时自动停止。
- 限制最大速度和角速度。
- 检查消息序号和 CRC。
- 禁止过期视觉观测驱动运动。
- 底盘本地保留急停。
- 机械机构需要执行结果确认。
- STM32 不能依赖 Ubuntu 在线才能安全停车。

---

## 十八、当前 Ubuntu 离线运行命令

### 18.1 加载环境

```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/car/algorithm/car_ws
source install/setup.bash
```

### 18.2 查看功能包

```bash
colcon list
```

### 18.3 编译自研 C++ 包

```bash
colcon build --packages-up-to car_bringup
```

### 18.4 检查自定义消息

```bash
ros2 interface show car_interfaces/msg/ChassisTelemetry
ros2 interface show car_interfaces/msg/WheelTelemetry
ros2 interface show car_interfaces/msg/VisionEvent
ros2 interface show car_interfaces/msg/MechanismState
```

### 18.5 启动模拟数据

终端一：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/car/algorithm/car_ws/install/setup.bash

ros2 launch car_bringup offline_debug.launch.xml \
  replay_interval_ms:=200
```

终端二：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/car/algorithm/car_ws/install/setup.bash

ros2 node list
ros2 topic list -t

ros2 topic echo /odom --once
ros2 topic echo /car/chassis/wheels --once
ros2 topic echo /car/mission/state --once
ros2 topic echo /car/debug/trajectory --once
ros2 topic echo /car/debug/diagnostics --once
```

注意：模拟数据有限，回放结束后需要重新启动，才能检查持续发布的话题。

### 18.6 单独启动桥接程序

```bash
ros2 run car_stm32_bridge car_ulog43_bridge \
  --ros-args \
  -p replay_file:=$HOME/Desktop/car/algorithm/car_ws/src/car_bringup/examples/sample_ulog43_synthetic.csv \
  -p replay_interval_ms:=200
```

### 18.7 启动 RViz2

```bash
ros2 launch car_bringup offline_debug.launch.xml \
  with_rviz:=true \
  replay_interval_ms:=200
```

当前离线 RViz 的 Fixed Frame 通常使用：

```text
odom
```

若机器人模型因 TF 不完整而无法显示，需要先检查 TF，而不是认为 CSV 解析一定失败。

---

## 十九、当前工程实际进度

### 已经完成

- [x] 建立统一 `car_ws` 工作空间。
- [x] 迁移 YDLIDAR 源码及配置。
- [x] 备份旧 `ydlidar_ws`。
- [x] 排除地图历史工程重复包。
- [x] 排除备用雷达驱动。
- [x] 编译 `ydlidar`。
- [x] 创建并编译 `car_interfaces`。
- [x] 创建并编译 `car_stm32_bridge`。
- [x] 创建并编译 `car_debug`。
- [x] 创建并编译 `car_bringup`。
- [x] 完成 C++ 模拟 CSV 解析代码。
- [x] 完成底盘、轮速、任务状态发布代码。
- [x] 完成轨迹及诊断发布代码。
- [x] 成功启动独立 CSV 回放桥接节点。

### 等待离线验证

- [ ] `/odom` 实际收到数据。
- [ ] 四轮速度话题正常发布。
- [ ] 比赛状态话题正常发布。
- [ ] 轨迹话题正常发布。
- [ ] 诊断话题正常发布。
- [ ] RViz2 显示轨迹。
- [ ] 自动离线测试全部通过。

之前 `smoke_test.sh` 曾报告 `/odom` 未在 10 秒内发布。随后单独启动桥接节点成功，但尚未确认完整端到端测试通过。

### 等待开发

- [ ] `car_vision` C++ 功能包。
- [ ] `car_mechanism` C++ 功能包。
- [ ] `car_mission` C++ 任务协调节点。
- [ ] `car_navigation`。
- [ ] 标准 IMU 话题。
- [ ] 完整 TF。
- [ ] URDF 机器人模型。
- [ ] 静态地图服务器。
- [ ] 激光雷达实测。
- [ ] STM32 与树莓派独立通信协议。
- [ ] OpenMV 和 STM32 视觉协议统一。
- [ ] 实车通信测试。
- [ ] 实车运动控制与安全测试。

---

## 二十、推荐下一步开发顺序

### 第一阶段：离线数据链验证

目标：

```text
CSV → C++ Bridge → ROS2 Topic → Debug → RViz2
```

重点：

- `/odom`
- `/car/chassis/wheels`
- `/car/debug/trajectory`
- `/car/debug/diagnostics`

### 第二阶段：视觉与机械模拟

新增：

```text
car_vision
car_mechanism
```

模拟发布：

```text
/car/vision/event
/car/mechanism/state
```

目的是不接真实小车也能验证识别结果和机构状态的数据流。

### 第三阶段：比赛任务管理

新增：

```text
car_mission
```

接收：

```text
/car/vision/event
/car/mechanism/state
/car/mission/state
/odom
```

输出上层任务进度和异常信息。

此阶段只做任务监控和软件协调，不抢占 STM32 比赛控制权。

### 第四阶段：导航系统

新增：

```text
car_navigation
```

整合：

```text
/map
/odom
/tf
/scan
```

逐步验证地图、定位、路径规划和避障。

### 第五阶段：STM32 实车通信

设计 STM32 到树莓派的专用链路。

首先实现只读遥测：

```text
STM32 → Pi → ROS2
```

确认可靠以后，再评估：

```text
ROS2 → STM32
```

并加入控制权仲裁、超时停车、急停和硬件安全保护。

### 第六阶段：完整实车联调

测试：

1. OpenMV 视觉识别。
2. STM32 任务接收。
3. 底盘运动。
4. 四轮速度反馈。
5. IMU 与里程计。
6. 机械机构动作及反馈。
7. 树莓派 ROS2 数据接收。
8. Ubuntu RViz2 显示。
9. Sim2Real 实时监控。
10. 安全停车及异常恢复。

---

## 二十一、核心设计原则

1. **只维护一个 ROS2 主工作空间：`car_ws`。**
2. **每项功能拆成独立功能包，统一编译。**
3. **ROS2 执行节点统一使用 C++17。**
4. **OpenMV 独立运行 MicroPython。**
5. **STM32 继续负责底层电机 PID 和实时运动控制。**
6. **优先使用标准 ROS2 消息，自定义消息只承载项目特有数据。**
7. **自定义消息保留原始数值、单位、时间戳和有效性信息。**
8. **同一状态话题应只有一个权威发布者。**
9. **没有标定的像素位置不能当作米制位移。**
10. **没有实际反馈的数据不能伪装成真实测量。**
11. **仿真通过不等于实车验收通过。**
12. **实车接入前先验证通信安全与停车保护。**

---

## 二十二、最终总结

目前项目已完成：

**ROS2 C++ 工程框架、自定义消息、STM32 ULog43 数据解析、底盘运动反馈消息、四轮速度消息、任务状态消息、轨迹诊断和统一启动工程。**

当前正在完成：

**模拟 CSV 到 ROS2 话题，再到调试和 RViz2 的完整软件数据链验证。**

未来需要完成：

**视觉事件、机械机构状态、IMU、TF、雷达、地图、导航，以及 STM32 与树莓派的实际通信。**

最终目标：

```text
视觉识别
    ↓
STM32 比赛决策
    ↓
底盘与机械机构执行
    ↓
STM32 实时反馈
    ↓
树莓派 ROS2
    ↓
任务管理 / 导航 / 诊断
    ↓
Ubuntu RViz2 / Sim2Real
```

**最终原则：底层实时控制归 STM32，上层数据与任务协调归 ROS2，OpenMV 独立视觉识别，Ubuntu 负责开发和监控。**
