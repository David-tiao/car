# 工创智能搬运小车——ROS 2 话题与上下位机数据流交接文档

**系统环境：** Ubuntu 22.04 + ROS 2 Humble + YDLIDAR X3 + Nav2 + STM32F4

**当前阶段：** 第 6.5 阶段（实时障碍物检测、道路封闭与 Nav2 路径规划）

**文档用途：** 供 ROS 2 上位机、STM32F4 下位机以及后续视觉模块开发人员对接使用。

> **重要说明：**
>
> - `[已实现]`：当前工程已有代码。
> - `[部分验证]`：已有功能代码，但仍需检查实机运行结果。
> - `[待实现]`：后续 STM32F4 和 Nav2 运动控制阶段需要开发。
> - `[历史/可选]`：旧第五、六阶段或特定启动方式使用的接口。
>
> 规划路径、方向箭头不等于底盘运动指令。目前还没有形成真实小车的运动闭环。

---

## 一、系统整体架构

目前系统主要分为五个模块：

1. **感知模块**：X3 雷达读取障碍物距离。
2. **地图模块**：建立田字形赛场地图，维护黄色禁行区及蓝色启停区。
3. **规划模块**：判断道路是否被障碍物阻断，调用 Nav2 规划路径。
4. **底盘控制模块（待实现）**：将 Nav2 运动指令发送给 STM32F4。
5. **运动反馈模块（待实现）**：接收 STM32F4 编码器数据，计算真实里程计。

系统最终数据链路：

```text
X3 雷达
    |
    v
/scan
    |
    v
障碍物检测与道路封闭
    |
    v
/competition/planning_map
    |
    v
Nav2 Planner Server
    |
    v
/competition/nav2_path
    |
    v
Nav2 Controller（待实现）
    |
    v
/cmd_vel（待实现）
    |
    v
ROS 2 串口通信节点（待实现）
    |
    v
STM32F4
    |
    v
电机运动
    |
    v
编码器反馈
    |
    v
STM32F4 串口上传
    |
    v
ROS 2 里程计节点（待实现）
    |
    +----> /odom
    |
    +----> odom → base_link 动态 TF
    |
    v
机器人定位与 Nav2 运动反馈
```

---

## 二、当前已有 ROS 2 话题列表

### 2.1 地图与雷达

| 话题 | 消息类型 | 主要数据 | 作用 | 状态 |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | 角度、测距数组、雷达坐标系、时间戳 | X3 实时扫描障碍物 | 已实现 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 地图原点、分辨率、宽高、占据数组 | 比赛场地物理底图 | 已实现 |
| `/keepout_filter_mask` | `nav_msgs/msg/OccupancyGrid` | 禁行区域占据数组 | 黄色禁行规则 | 已实现 |
| `/competition/planning_map` | `nav_msgs/msg/OccupancyGrid` | 整合禁行区和封闭路段后的栅格地图 | 供 Nav2 规划使用 | 已实现 |

#### `/scan` 的主要数据结构

```text
header.stamp
header.frame_id = laser_frame

angle_min
angle_max
angle_increment

range_min
range_max

ranges[]
intensities[]
```

其中：

- `ranges[]`：各扫描方向上的测距值，单位 m。
- `angle_min`：扫描起始角度，单位 rad。
- `angle_increment`：相邻测距点之间的角度增量，单位 rad。
- `header.stamp`：测量时间。
- `header.frame_id`：雷达所在坐标系。

**数据流：**

```text
X3 雷达
    |
    v
ydlidar_ros2_driver
    |
    v
/scan
    |
    v
TF 坐标转换
    |
    v
地图坐标下的障碍物位置
    |
    v
道路封闭判定
```

#### `/map` 和 `/competition/planning_map` 的区别

`/map` 是基础物理地图，反映赛场固定结构。

`/competition/planning_map` 是 Nav2 的规划地图，会把以下区域视为不可通行：

- 比赛场地外部。
- 黄色禁行区域。
- 已知固定障碍区域。
- 雷达判断被阻断的道路。

**规划地图不应直接覆盖 RViz 中的彩色田字形底图。**

---

### 2.2 道路拓扑规划

| 话题 | 消息类型 | 主要数据 | 作用 | 状态 |
|---|---|---|---|---|
| `/competition/road_graph` | `visualization_msgs/msg/MarkerArray` | 节点坐标、道路状态 | 显示红绿路段、道路节点 | 已实现 |
| `/competition/road_path` | `nav_msgs/msg/Path` | 拓扑候选路线的节点位姿 | Dijkstra 道路级路线 | 已实现 |
| `/competition/road_status` | `std_msgs/msg/String` | JSON 状态信息 | 反馈哪些道路封闭、是否可通行 | 已实现 |

#### 当前田字形路网

原始路网共有：

- 9 个道路节点。
- 12 条道路。
- 另增加启停区 1 的起点 `S1`，通过 `S1–C3` 连接到路网。

```text
             原料区方向

A3 -------- B3 -------- C3 ---- S1(启停区1)
 |           |           |
 |           |           |
A2 -------- B2 -------- C2
 |           |           |
 |           |           |
A1 -------- B1 -------- C1

                           启停区2
```

当前规则编号与测试坐标：

| 区域 | 位置 | 中心坐标 |
|---|---|---|
| 启停区 1 | 右上角 | `(2.25, 2.25)` m |
| 启停区 2 | 右下角 | `(2.25, 0.15)` m |

默认测试起点为 `S1`，默认演示目标为 `B3`。

`B3` 仅为联调目标，不代表比赛正式任务的第一个目标点。

#### 道路状态数据

`/competition/road_status` 使用 `std_msgs/String`，其 `data` 字段存放 JSON。

主要字段包括：

```json
{
  "ready": true,
  "start": "S1",
  "goal": "B3",
  "blocked": ["B1-C1"],
  "route": ["S1", "C3", "B3"],
  "status": "NAV2_OK"
}
```

以上仅为**数据格式示例**，不是实机当前的规划结果。

字段含义：

| 字段 | 作用 |
|---|---|
| `ready` | 数据是否准备完成 |
| `start` | 当前起始节点 |
| `goal` | 当前目标节点 |
| `blocked` | 已封闭的道路列表 |
| `route` | Dijkstra 候选节点序列 |
| `status` | 当前规划状态 |

注意：在第 6.5 阶段，`road_status` 还用于兼容性发布 Nav2 状态数据，内容不一定只包含原始第六阶段字段。

---

### 2.3 Nav2 真正的路径规划

| 接口 | 消息类型 | 主要数据 | 作用 | 状态 |
|---|---|---|---|---|
| `/competition/nav2_path` | `nav_msgs/msg/Path` | 规划得到的连续位姿点 | Nav2 实际路径 | 已实现 |
| `/competition/nav2_plan_status` | `std_msgs/msg/String` | JSON 状态、地图版本、请求次数 | Nav2 规划状态监控 | 已实现 |
| `/competition/nav2_direction_arrows` | `visualization_msgs/msg/MarkerArray` | 路径上的箭头位置及方向 | RViz 方向箭头 | 部分验证 |
| `/competition_nav2/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | 规划起点、终点、规划结果 | 调用 Nav2 Planner Server | 已实现 |

**`ComputePathToPose` 是 ROS 2 Action，不是普通 Topic。**

当前规划流程：

```text
X3 /scan
    |
    v
识别被障碍物封闭的路段
    |
    v
更新 /competition/planning_map
    |
    v
Nav2 专用 Global Costmap
    |
    v
ComputePathToPose Action
    |
    v
Nav2 Planner Server
    |
    v
/competition/nav2_path
    |
    +------> RViz 洋红色路径
    |
    +------> 方向箭头节点
                  |
                  v
        /competition/nav2_direction_arrows
                  |
                  v
          RViz 橙色方向箭头
```

#### `/competition/nav2_path` 数据结构

```text
nav_msgs/Path
    |
    +-- header
    |
    +-- poses[]
          |
          +-- header
          |
          +-- pose.position.x
          +-- pose.position.y
          +-- pose.position.z
          |
          +-- pose.orientation.x
          +-- pose.orientation.y
          +-- pose.orientation.z
          +-- pose.orientation.w
```

每个 `pose` 表示路径上的一个位置及姿态。

**注意：路径点不是电机速度。** 后续还需要 Nav2 Controller 将路径变成 `/cmd_vel`。

#### `/competition/nav2_plan_status` 主要字段

| 字段 | 作用 |
|---|---|
| `status` | 当前规划状态 |
| `nav2_path_ready` | 是否有有效 Nav2 路径 |
| `blocked` | 已封闭路段 |
| `route` | 道路级候选路线 |
| `map_version` | 规划地图版本号 |
| `nav2_requests` | Nav2 规划请求次数 |
| `replans` | 成功规划次数 |
| `scans` | 处理的扫描帧数 |
| `tf_errors` | 坐标转换失败计数 |
| `reason` | 当前状态原因 |

主要状态：

- `NAV2_OK`：Nav2 返回有效路径。
- `WAIT_FOR_NAV2`：等待规划器准备。
- `NAV2_PLANNING`：正在请求或计算路径。
- `NO_ROUTE`：道路图没有可行路线。
- `START_OR_GOAL_BLOCKED`：起点或终点不合法。
- `NAV2_FAILED`：Nav2 请求或结果校验失败。
- `SCAN_STALE`：扫描数据已过期。

只有 `NAV2_OK`，才说明当前规划请求得到有效路径。

---

### 2.4 RViz 可视化话题

| RViz 图层 | 话题 | 作用 |
|---|---|---|
| Physical Base Map | `/map` | 田字形赛场基础地图 |
| X3 Scan | `/scan` | 雷达扫描点 |
| Blue Start Zones | `/competition_start_zones` | 蓝色启停区 1、2 |
| Yellow Rule Keepout | `/competition/yellow_keepout_zones` | 黄色禁行区域 |
| Road Topology | `/competition/road_graph` | 道路节点、红绿路段 |
| Road Route | `/competition/road_path` | 道路级候选路线 |
| Real Nav2 Path | `/competition/nav2_path` | Nav2 真正规划的路径 |
| Nav2 Direction Arrows | `/competition/nav2_direction_arrows` | Nav2 路径方向箭头 |
| Stage65 Planning Map | `/competition/planning_map` | Nav2 实际使用的禁行栅格 |

其中：

**蓝色、黄色、红绿道路属于显示层；实际是否允许通行必须以规划地图及其约束为准。**

---

## 三、当前 TF 坐标关系

TF 用于描述机器人、雷达和地图之间的空间关系。

涉及话题：

| 话题 | 类型 | 作用 |
|---|---|---|
| `/tf` | `tf2_msgs/msg/TFMessage` | 动态坐标转换 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 静态坐标转换 |

目标 TF 结构：

```text
map
 |
 v
odom
 |
 v
base_link
 |
 v
laser_frame
```

含义：

| 坐标变换 | 数据来源 | 当前状态 |
|---|---|---|
| `map → odom` | 地图定位模块 | 当前为测试关系，真实定位待实现 |
| `odom → base_link` | 编码器里程计或融合里程计 | 当前使用固定测试 TF |
| `base_link → laser_frame` | 雷达实际安装位置 | 当前为测试参数，实车需标定 |

当前默认测试位姿：

```text
位置：
x = 2.25 m
y = 2.25 m

初始角度：
yaw = -135°
```

该姿态只用于静止规划测试。小车实际运动时必须替换成真实位姿。

---

## 四、后续 STM32F4 需要新增的话题

以下接口**尚未实现**，需要根据实际 STM32F4 工程和底盘结构确认。

### 4.1 上位机发送运动指令

| 话题 | 消息类型 | 数据 | 作用 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x`、`linear.y`、`angular.z` | Nav2 Controller 输出车体速度 |

主要数据：

```text
linear.x   前后线速度，单位 m/s
linear.y   横向线速度，单位 m/s
angular.z  旋转角速度，单位 rad/s
```

差速底盘通常使用 `linear.x` 和 `angular.z`。

麦克纳姆等全向底盘还可能使用 `linear.y`。

**需要先确定实际底盘类型，才能写 STM32F4 的速度分解公式。**

数据流：

```text
Nav2 Controller
    |
    v
/cmd_vel
    |
    v
ROS 2 底盘串口节点
    |
    v
串口速度控制数据帧
    |
    v
STM32F4
    |
    v
底盘运动学计算
    |
    v
电机速度闭环
    |
    v
电机驱动
```

### 4.2 STM32F4 编码器反馈

建议建立以下接口：

| 建议话题 | 消息类型 | 数据 | 作用 |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | 轮子角度、角速度 | 记录驱动轮运动状态 |
| `/competition/encoder_ticks` | 自定义消息，待定义 | 编码器原始累计计数、采样时间 | 保留原始编码器信息 |
| `/odom` | `nav_msgs/msg/Odometry` | 机器人位置、方向、速度 | 导航和定位反馈 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `odom → base_link` | 实时车体坐标变换 |

`/competition/encoder_ticks` 是**推荐设计**，不是当前已经存在的话题，也尚未定义对应的 ROS 自定义消息包。

建议 STM32 发送：

```text
编码器采样时间
编码器累计计数
电机转速（可选）
电机方向
数据帧序号
驱动器状态
```

例如四轮底盘，数据字段可以设计成：

```text
timestamp_ms

encoder_front_left
encoder_front_right
encoder_rear_left
encoder_rear_right

motor_status
sequence
```

以上是结构示例；实际顺序及类型应由底盘硬件确定。

### 4.3 `/odom` 里程计

消息类型：

```text
nav_msgs/msg/Odometry
```

主要数据：

```text
header.stamp
header.frame_id = odom

child_frame_id = base_link

pose.pose.position.x
pose.pose.position.y
pose.pose.orientation

twist.twist.linear.x
twist.twist.linear.y
twist.twist.angular.z

pose.covariance
twist.covariance
```

作用：

- 计算小车相对运动位姿。
- 反馈机器人线速度及角速度。
- 为 Nav2 提供移动状态。
- 配合定位模块完成真实导航。

编码器数据流：

```text
电机编码器
    |
    v
STM32F4 编码器计数
    |
    v
STM32F4 串口发送
    |
    v
ROS 2 串口接收节点
    |
    +------> /joint_states
    |
    +------> 原始编码器数据（可选）
    |
    v
底盘运动学与里程计计算
    |
    +------> /odom
    |
    +------> TF: odom → base_link
    |
    v
Nav2 / 定位系统
```

---

## 五、STM32F4 串口通信方案

推荐采用 **Ubuntu ROS 2 ↔ USB/UART ↔ STM32F4** 的双向通信结构。

雷达 X3 和 STM32F4 应使用各自独立的设备接口，避免串口冲突。

### 建议协议字段

| 字段 | 作用 |
|---|---|
| 帧头 | 标记数据帧开始 |
| 协议版本 | 兼容不同版本 |
| 命令类型 | 区分速度指令、编码器、状态 |
| 数据长度 | 确认载荷大小 |
| 序号 | 检查丢帧和乱序 |
| 时间戳 | 记录实际采样时间 |
| 数据载荷 | 运动指令或编码器数值 |
| CRC 校验 | 检查通信错误 |

建议将通信分为：

**上位机 → STM32F4**

- 速度控制指令。
- 停车和急停指令。
- 执行机构指令。
- 控制模式设置。

**STM32F4 → 上位机**

- 编码器累计值。
- 电机速度。
- 电机故障信息。
- 下位机运行状态。
- 执行机构反馈。

必须实现通信超时停车，防止上位机断开时小车继续运动。

此协议仅为设计建议，实际字节序、数据类型、帧头和 CRC 方案必须与 STM32 工程统一。

---

## 六、未来完整 Nav2 与 STM32F4 闭环

```text
                 [感知部分]

YDLIDAR X3
    |
    v
/scan
    |
    v
障碍物检测
    |
    v
道路封闭判断
    |
    v
/competition/planning_map


                 [路径规划部分]

Nav2 Planner Server
    |
    v
/competition/nav2_path
    |
    v
Nav2 Controller


                 [运动控制部分]

/cmd_vel
    |
    v
ROS 2 串口桥接节点
    |
    v
STM32F4
    |
    v
电机速度闭环
    |
    v
小车实际运动


                 [运动反馈部分]

电机编码器
    |
    v
STM32F4 计数和采样
    |
    v
UART 回传
    |
    v
ROS 2 串口桥接节点
    |
    v
里程计计算
    |
    +------> /odom
    |
    +------> TF: odom → base_link
    |
    v
Nav2 运动反馈


                 [地图定位部分]

/scan + /map + /odom
    |
    v
AMCL 或其他定位方案
    |
    v
TF: map → odom
    |
    v
更新真实机器人地图位姿
    |
    v
Nav2 持续控制与重规划
```

**这里需要特别说明：** 编码器本身主要提供相对运动信息，可能存在累计误差；地图定位还需要结合雷达或其他定位方法。

---

## 七、后续视觉模块数据流

视觉部分需要与 `vision.c`、`vision.h` 的实际代码对接。

建议总体结构：

```text
OpenMV H7 Plus / 摄像头
    |
    v
二维码、颜色或物料识别
    |
    v
视觉识别结果
    |
    v
STM32F4 或 Ubuntu 视觉桥接
    |
    v
ROS 2 视觉结果接口
    |
    v
任务决策模块
    |
    v
生成下一目标点
    |
    v
Nav2 Planner
```

建议预留：

| 建议话题 | 作用 | 当前状态 |
|---|---|---|
| `/competition/vision_result` | 视觉识别结果 | 待定义 |
| `/competition/task_goal` | 下一导航目标 | 待定义 |
| `/competition/base_status` | STM32F4 底盘状态 | 待定义 |
| `/competition/emergency_stop` | 急停请求 | 待定义 |

这些接口并非当前工程已发布的话题，具体消息类型仍需确定。

---

## 八、当前系统话题检查命令

### 查看全部话题及类型

```bash
source /opt/ros/humble/setup.bash
ros2 topic list -t
```

### 查看 Nav2 Action

```bash
ros2 action list -t
```

重点检查：

```text
/competition_nav2/compute_path_to_pose
```

### 查看雷达数据

```bash
ros2 topic info -v /scan
ros2 topic hz /scan
```

### 查看地图和规划结果

```bash
ros2 topic info /map
ros2 topic info /competition/planning_map
ros2 topic info /competition/nav2_path
```

### 查看规划状态

```bash
ros2 topic echo /competition/nav2_plan_status \
  --once --qos-durability transient_local
```

### 查看方向箭头

```bash
ros2 topic info -v /competition/nav2_direction_arrows
```

### 查看 TF

```bash
ros2 run tf2_ros tf2_echo map laser_frame
```

---

## 九、上下位机联调任务清单

### 已实现

- [x] YDLIDAR X3 发布 `/scan`。
- [x] 比赛基础地图 `/map`。
- [x] 黄色禁行区域。
- [x] 蓝色启停区域显示。
- [x] 田字形道路拓扑。
- [x] 雷达障碍物封路。
- [x] 持续扫描并触发重新规划。
- [x] Nav2 Planner 接口与规划路径发布代码。
- [x] Nav2 路径方向箭头生成代码。

### 还需进一步验收

- [ ] Nav2 返回 `NAV2_OK` 且路径点有效。
- [ ] RViz2 中真实路径方向箭头正常显示。
- [ ] 检查真实雷达安装位置与扫描投影精度。
- [ ] 检查障碍物出现、移走后的响应时间。

### 待实现

- [ ] STM32F4 串口双向通信。
- [ ] ROS 2 `/cmd_vel` 到 STM32F4 控制协议。
- [ ] 底盘电机速度闭环。
- [ ] STM32F4 编码器数据回传。
- [ ] ROS 2 `/joint_states` 或原始计数接口。
- [ ] 真实 `/odom`。
- [ ] 动态 `odom → base_link`。
- [ ] 地图定位 `map → odom`。
- [ ] Nav2 Controller 路径跟踪。
- [ ] 超时停车与急停保护。
- [ ] 视觉识别和任务决策对接。
- [ ] 真实比赛场地运动测试。

---

## 十、下一次联调需要准备的文件

STM32F4 侧：

- `main.c`
- UART/USART 初始化文件
- UART DMA/中断收发文件
- 电机控制源文件
- 编码器读取源文件
- PID 速度闭环相关代码
- 底盘运动学代码（若已有）
- 当前通信协议说明

视觉侧：

- `vision.c`
- `vision.h`
- 摄像头识别主程序
- 视觉串口协议
- 二维码和颜色识别结果定义

机械与硬件参数：

- 底盘类型
- 轮子数量
- 轮子半径
- 轮距、轴距
- 编码器分辨率
- 减速比
- 电机和编码器方向
- 串口号与波特率

---

## 十一、最终交接结论

当前工程已经建立：

**X3 雷达 → 障碍物检测 → 道路封闭 → Nav2 路径规划 → RViz 可视化**

后续需要建立：

**Nav2 Controller → `/cmd_vel` → STM32F4 → 电机 → 编码器 → `/odom` → TF → Nav2**

现阶段最大的接口缺口是**真实底盘运动反馈**。

后续建议先完成 STM32F4 编码器和 ROS 2 里程计，再接入 Nav2 Controller，让规划路径真正转换成小车运动。

同时需要保留当前正常运行的地图、实时雷达封路和第 6.5 阶段规划模块，避免上下位机联调时反复修改已经验证的感知与规划代码。
