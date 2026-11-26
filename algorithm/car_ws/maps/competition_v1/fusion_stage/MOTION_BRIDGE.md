# ROS2 双向运动通信桥接 — 独立集成模块

基线：David-tiao/car，feat/lidar-region-scan，681bdf26a40b36d0249ed89c6e907de50ea0ee10。本模块独立运行，不接入现有一键启动，默认不打开串口。

## 模块与集成路径

位于 `algorithm/car_ws/maps/competition_v1/fusion_stage/`：

- `motion_bridge_core.py`：独立会话传输协调、逐段动作、许可与反馈时间约束、异常退出。
- `motion_serial.py`：Linux POSIX tty，115200 8N1，非阻塞，独占打开请求与 advisory flock，不自动重连。
- `motion_bridge_node.py`：ROS2 Humble Python 节点，单线程执行器、steady timer 10ms；默认 standby，不打开设备、不发送字节。参数 `enabled=false`、`operator_enabled=false`、`port=''`、`session=0` 均只读。未提供实机启用命令。启用参数本身不等于 MCU 本地物理授权。
- `test_motion_bridge.py`、`test_c_peer.py`：离线、Linux PTY 及真实 C 协议核心交叉测试。
- `MOTION_BRIDGE.md`：本文。

复用基线已有的 `motion_link.py`、`pi_session.py` 与 `STM32F4/App/motion_link.c/.h`，不覆盖这些文件。C 测试在仓库中读取原 `STM32F4/App`；独立审查包则可使用同目录副本。未修改 ULog43、STM32 固件、OpenMV、CMake/launch、原 fusion_core/preview 或一键启动。

## 离线验证

```bash
cd ~/workspace/05_project_github/car/algorithm/car_ws/maps/competition_v1/fusion_stage
python3 -m unittest -v test_motion_bridge test_c_peer
python3 -m py_compile motion_bridge_core.py motion_serial.py motion_bridge_node.py
```

需要 Linux、Python 3.10+、C 编译器 `cc`。新增 32 项测试通过；含 3 项 PTY 测试（字节分片/关闭、独占、实际 C core 端到端）。基线 65 项原有主机测试也重新执行通过，测试日志不随仓库提交，可按上述命令重新生成。C 端位姿、停车速度标志和本地 grant 都是合成值，未连接硬件。运行当前仓库的基线测试命令：`cd STM32F4/tests/pi_motion && python3 -m unittest -v`。

ROS2 环境中的默认只读 standby 启动（须在实际已安装 ROS2 Humble 的环境单独验收；当前容器无 rclpy）：

```bash
cd ~/workspace/05_project_github/car/algorithm/car_ws/maps/competition_v1/fusion_stage
source ~/workspace/00_system/workplace/scripts/activate.sh ros
python3 motion_bridge_node.py
```

另一终端查询 `/competition/motion/status`：预期 phase=STANDBY，pose_valid=false，navigation_authorized=false。尚未实际验证 ROS2 包安装/运行、QoS、启动退出以及 RViz。未来若需 ros2 run/launch，另行授权包安装与启动集成。本节点不自动启动旧雷达或控制器。

## ROS 接口

全部使用 reliable / volatile / depth=1。`goal`、`safety`、`feedback`、`status` 为 std_msgs/String JSON；`pose` 为 geometry_msgs/PoseStamped。仅 `odom` 坐标系；位置输入是整数 mm，角度为整数 mdeg（-180000、-90000、0、90000 四种目标）。不直接接收 map 位姿；外部必须先使用实测、已验证的 map->odom 变换，且转换后的正交动作符合 MCU 航向约定。

| Topic | 方向 | 说明 |
| --- | --- | --- |
| /competition/motion/goal | 订阅 | 单个正交目标，带 session/goal_id |
| /competition/motion/safety | 订阅 | 对特定目标的短时许可，带会话和递增序号 |
| /competition/motion/feedback | 发布 | 每次接受的新鲜 STATUS 位姿、状态、完成 ID |
| /competition/motion/status | 发布 | 约 10Hz 的状态/故障/有效性，包含反馈年龄 |
| /competition/motion/pose | 发布 | 新鲜反馈的 PoseStamped，odom，接收时间 |

**不发布 /odom 或 /tf**：M2 STATUS 不含速度、MCU 采样时戳或协方差，不能伪造完整 nav_msgs/Odometry。当前 ULog43 节点已有 /odom 发布，必须避免重复发布与争抢 USART2。PoseStamped 使用接收时间而非真实 MCU 采样时间。`pose_valid` 表示当前会话接受的反馈仍新鲜，不代表地图定位已经校准。旧 pose topic 的订阅者必须同时检查 status 的时效与有效性。

安全许可 schema `car_motion_safety_v1`：session、seq、stamp_ns（与 Pi5 wall clock 同一时间域的 Unix ns）、frame_id='odom'、allow、local_grant_confirmed、tf_verified、goal_id、kind、start、goal、center_bounds_mm=[xmin,ymin,xmax,ymax]、turn_clear。布尔值必须为 JSON boolean。启动前只接受 goal_id=0、kind=IDLE、start/goal=null 的新鲜许可；之后按目标 ID 发布新鲜许可。goal_id>0 时 start/goal 必须与 goal 消息完全相同；kind 必须为 TRANSLATE/TURN_90/MICRO_TRANSLATE。center_bounds 是外部安全监督器已经计算的保守**车体中心允许区域**，必须自行计入车辆足迹、定位误差、制动和障碍物；桥接仅检查动作端点及反馈是否位于其中，不具备真正动态碰撞检测能力。90°转向还必须获得 turn_clear=true。允许区必须覆盖目标和已下发分段终点。更新安全消息时不能在动作尚未完成前切换 goal_id。匹配会话的 allow=false 即使序号旧也触发故障。其他错误或陈旧许可不续期。安全许可收到时的年龄必须小于 250ms，剩余有效期按本地 monotonic 时钟计。建议监督器约 10Hz 发布，但必须以现场链路时延、停止距离和安全评估决定最终参数；未开发真实监督器。

目标 schema `car_motion_goal_v1`：session、goal_id（正整数递增，同一会话不重用）、stamp_ns、frame_id='odom'、kind、start=[x_mm,y_mm,yaw_mdeg]、goal=[...]。先发送对应新鲜许可，等待节点收到后再提交唯一目标。仅 IDLE 且反馈新鲜时接收。会话管理器检查与反馈的 8mm/3°容差，单轴、最大300mm分段，微移最大20mm。收到 DONE 后按反馈位置计算余量。目标已经处于容差内时可能无需运动；不代表物理毫米级精度。不得把旧版 SimMission 的模拟事件直接转成真实运动授权。

## 故障及限制

- 100ms 心跳，120ms 回复超时重试（最多两次），360ms 反馈新鲜度上限；外部许可有效期 250ms。参数为待实测工程值，不是保证的停止时延。
- 掉线、STOP、许可过期/撤销、无效目标、反馈越界、错误回执、部分写入等锁定本地故障、尝试一次 STOP 后关闭。`stop_written=true` 仅表示 OS 接受了整帧写入，**不是** MCU 收到或物理停车确认。USB/TTY 关闭可能丢失排队数据；断线时依赖 MCU 已实现的看门狗与实际急停。故障不自动恢复或重连；需要新的物理授权和独立会话。异常停机时 Python finally 不保证执行。
- Linux tty 锁无法阻止已经打开的非协作进程、特权进程或恶意发布者。需要独立设备所有权、停掉 ULog43、可信 DDS/ROS 网络与物理急停。CRC 和会话号不是鉴权机制。
- 目前正式 STM32 新模式仍固定 pi_local_enable()=0、无实际 LocalGrant 路径。因此实际 ARM 不会成功；不应为测试而绕开。OpenMV V4 在新模式下的实际任务接管也尚未开发。
- 无 ROS2 runtime、ARM/Keil 实编、物理串口、TF 标定、真实动态避障或整车测试。本次新增六个独立模块/测试/文档文件，不改已有启动入口或固件；无烧录、无硬件运动。
