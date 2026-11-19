# 第三阶段：ROS2 TEST 比赛任务回放与网页显示

## 目标

将阶段二的 CAR-LINK V1 **匿名 Linux PTY** 模拟任务闭环以低速分步的形式发布为独立 ROS 2 TEST 话题，并在现有 Sim2Real `/test` 页面展示任务进度。保持现有 `car_serial_link` 实车桥接**只读**。

此分支基于第二阶段分支 `feat/mission-carlink-pty-simulation-20261120`，因此**先合并 PR #2，再考虑本阶段 PR**。不需要修改 STM32F4。

## ROS2 TEST 话题

| Topic | 类型 | 内容 |
| --- | --- | --- |
| `/car/test/mission/state` | `std_msgs/String` JSON | 当前任务、步骤进度、模拟抓放统计、终止状态 |
| `/car/test/mission/feedback` | `std_msgs/String` JSON | TEST PTY 返回的 ACK、STARTED、COMPLETED、FAILED |
| `/car/test/mission/command_feedback` | `car_interfaces/CommandFeedback` | 类型化 B2 模拟回执；`original_command_type=0xE0`，并限定 TEST 命名空间 |
| `/car/test/mission/road_node_pose` | `geometry_msgs/PoseStamped` | **离散路网节点预览**，非真实定位 |
| `/car/test/mission/road_node_trace` | `nav_msgs/Path` | 仅显示已模拟完成的路网节点轨迹 |

**不发布** `/cmd_vel`、`/odom`、`/car/mission/state`、`/car/stm32/command_feedback`，防止将 TEST 数据伪装成真实控制或 STM32 源数据。

## Web TEST 数据隔离

新建 `sim2real/test_mission_store.py`，只提供：

- `GET /api/test/mission_replay`：只读获取最近 3 秒内的 MOCK 任务回放；
- `POST /api/test/mission_replay`：只接受 localhost、严格校验 `MOCK` / `SIM_ONLY` / `hardware_authorized=false` / `DISCRETE_ROAD_NODE` 的 JSON。

`sim2real/test_mission_relay.py` 只订阅 ROS 2 TEST 话题，将校验后的数据推给 localhost 网站。现有 `/api/test/inspection`、`/api/real/*`、`carx test` 和实车只读桥接保持原设计。新增网页 `test_mission_replay.js` 为独立监视组件，不复用已有虚拟 MCU 的**定时合成反馈**。

## 先运行无 ROS 离线验收

```bash
cd ~/Desktop/car
bash algorithm/car_ws/bash/test_mission_ros_phase3.sh
```

预期执行原有规划、任务和协议测试，加上新增 4 个模拟会话测试、3 个 MOCK 网页数据验证测试，合计 **37 项单元测试**。此外检查 Python 和 Bash 语法，并跑阶段二的独立 PTY 演示。

## Ubuntu ROS2+网页联调（两个终端）

阶段一：先构建已有 ROS2 包，以便原 TEST 页面正常工作：

```bash
cd ~/Desktop/car
bash algorithm/car_ws/bash/build.sh core
```

终端 A，启动**原有 TEST 模拟底盘**和网页：

```bash
cd ~/Desktop/car
bash algorithm/car_ws/bash/web.sh test
```

终端 B，启动**独立比赛任务 PTY 流程**：

```bash
cd ~/Desktop/car
bash algorithm/car_ws/bash/run_test_mission_ros.sh --start-zone upper_right --task-code '156+123+516+231' --block B3-C3
```

浏览器访问 `http://127.0.0.1:8080/test`。网页上会出现独立的 `TEST · 比赛任务状态机与虚拟串口回执` 面板。

终端 C，可以实时查看两个 TEST 话题：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=73
ros2 topic echo /car/test/mission/state std_msgs/msg/String
ros2 topic echo /car/test/mission/feedback std_msgs/msg/String
```

以上两个 `ros2 topic echo` 命令需要分别放在两个终端运行或逐个运行。启动网页并不需要实车或雷达。

故障注入：

```bash
bash algorithm/car_ws/bash/run_test_mission_ros.sh --fail-at 3
bash algorithm/car_ws/bash/run_test_mission_ros.sh --drop-ack-at 2
```

在模拟数据被拒绝或模拟 MCU 失败时，状态机会进入 `FAULT` 并停止推进后续步骤。不会尝试实际急停（因为无真实运动控制）。

## 测试范围和限制

1. 本阶段不生成真实底盘轨迹、速度命令或绝对目标位姿；`road_node_pose` 只是完成 `MOVE_NODE` 后**离散跳到目标节点**，不可作为导航精度证据。
2. 模拟 ACK 和 COMPLETED 确实通过 TEST PTY 字节帧传输，但仍来自软件模拟 MCU，**不代表 STM32 固件支持实际执行**。
3. 网页原来的 `test_console.js` 仍显示旧 MOCK MCU 定时合成反馈；新的任务面板独立显示**本次任务命令**，不能混为一谈。
4. 只应在 `ROS_DOMAIN_ID=73` TEST 环境运行。不能与实车控制权仲裁代码混用。
5. 由于本环境未安装仓库的 Ubuntu ROS 2 工作空间，真实 `rclpy` Topic 和网站集成需要在你们 Ubuntu 上执行上面两类验收，不能以源码存在代替运行验证。
