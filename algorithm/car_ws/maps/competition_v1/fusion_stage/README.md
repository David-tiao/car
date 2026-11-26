# 雷达与 OpenMV 融合定位及自主接近 — 正交连续任务审查包（阶段 2）

基线：David-tiao/car `feat/lidar-region-scan`，`ea5a90ae9966068de4c8224faf368b05309c66d5`。本模块独立于现有启动入口；不修改现有工程和固件。

## 代码与边界

- `fusion_core.py`：读取既有 `car_startup_scene_v1`，验证双目标候选、锁路和障碍信息；对已开放路段细分，检查圆形保守车体包络与静态禁行区、场界和障碍圆，Dijkstra 寻找满足观察距离且视线无已知遮挡的观察位姿。目标朝向已改为仅允许四个正交航向（0/90/180/270°），利用麦轮横移完成最后对准；不再输出斜向停车姿态，也不是已测二维码板法向。圆形车体包络须包含相机/机械突出部与误差裕量；规划中 QR 板估计半径为 0.16m + 用户给定不确定度，圆盘同时保留名义和候选障碍。仅做道路与几何层的静态预览，不保证真实障碍中心、完整碰撞模型或可执行性。`project_pixel_to_plane` 接受去畸变后的像素、标定内参、map 下相机位姿及独立测得平面；`visual_step` 由像素雅可比矩阵逆解限幅小步，仅供仿真。
- `SimHandoff`：模拟导航停车后，连续新鲜且位置一致的视觉证据，二维码合法任务码/会话 ACK / Pitch 完成，或盘面确认及分步微调。每步停稳后才重新采样，超时、冲突、错误目标会拒绝或退出。合成事件不是 V4 实际报文，不会发给 STM32；微调步长不代表完成碰撞验证。当前没有真实运动、速度控制、视觉消息桥或动态定位输入。
- `preview.py`：纯 Python demo、保存快照规划、可选 ROS2 只读节点。真实预览从指定 checkout 动态读取 `road_topology.py` 的 NODES/EDGES 和 `zones.json`，不修改原文件；不得使用假尺寸/假坐标。订阅 `/competition/lidar/scene_status`，发布 `/competition/fusion_review/path` (nav_msgs/Path)、`/competition/fusion_review/markers` (MarkerArray)、`/competition/fusion_review/status` (String)。QoS reliable/transient local。超时、坏快照输出空路径和 DELETEALL。使用手动实测静止起点，不跟踪车辆运动；禁止行驶时运行本节点。原场景本身冻结目标快照并有时间限制，过期须重新开局扫描。无 `/cmd_vel`、Nav2 action、硬件或串口调用。
- `test_fusion.py`：28 个 unittest，使用与真实基线相同结构的合成图、合成尺寸/观测；非实际 ROS/实机测试。
- 演示 JSON 和测试日志可通过下列命令在本地生成，不随本模块提交。

## 无硬件运行（Python 3.10+）

```bash
cd ~/workspace/05_project_github/car/algorithm/car_ws/maps/competition_v1/fusion_stage
python3 -m unittest -v test_fusion
python3 preview.py mission-demo > demo_result_local.json
```

预期 28 tests OK；demo 两段均 `OBSERVATION_POSE_PREVIEW`，连续模拟最终 `READY_PICK`，所有硬件/导航/实际身份授权字段均 false。现在观察位姿只允许正交朝向；若找不到合法正交观察位，将返回 `NO_ORTHOGONAL_OBSERVATION_POSE`。示例路径长度可能随策略变化，不是实测值。

## 接入现场只读预览（需先实测并启动已有静止扫描）

```bash
cd ~/workspace/05_project_github/car/algorithm/car_ws/maps/competition_v1/fusion_stage
python3 preview.py ros --help
```

预期输出必填参数说明。真实调用需要显式填入 `--repo`（checkout 根目录）、`--start-pose X Y YAW_RAD`、`--envelope-m`、`--qr-range-m MIN MAX`、`--disk-range-m MIN MAX`、`--obstacle-uncertainty-m`、`--target-uncertainty-m`、`--target qr_board` 或 `raw_disk`。均需实测，不提供伪默认数值。开启 ROS 环境后单独运行，不会自行启动雷达或底盘。运行已有 RViz，Fixed Frame=map，添加 Path 与 MarkerArray，分别订阅上述预览话题，QoS reliable / transient local。原始二维码与盘面位置仍由既有 marker 提供。若状态 `WAIT_FRESH_SCENE`、`STALE_SCAN_EPOCH`、`REJECTED_SCENE` 或 `NO_SAFE_OBSERVATION_POSE`、`NO_ORTHOGONAL_OBSERVATION_POSE`，路径为空，不得解释为导航许可。

## 后续集成（需要逐项授权）

本模块包含 `algorithm/car_ws/maps/competition_v1/fusion_stage/` 内 `fusion_core.py`、`preview.py`、`test_fusion.py`、`README.md`，保留独立启动，不修改旧入口、STM32 或 OpenMV。后续才考虑把 preview 接入一键启动和 RViz；并另行授权 QR ACK / DONE / IDLE 在 STM32 侧互锁、OpenMV 提供 QR 中心/标记和盘面观测、真实视觉桥、时间同步、动态 TF、带完整足迹的停车及闭环控制。

## 验证情况

已执行 Python 语法检查、28 项 unittest、旧版两段合成 demo 和连续任务 mission-demo。ROS2 rclpy 在当前容器缺失，未运行 ROS2 节点或 RViz；未跑仓库全量 CI、Gazebo、实机或固件。此包不能驱动车辆，不能当作已经完成真实融合定位或自动驾驶。

## 阶段 2：连续任务与正交约束

本模块仅包含四个授权文件；原算法与旧版 `demo` 保留。`SimMission` 是连续任务仿真协调器，**不接收真实 V4 串口消息，不发送运动命令，不接入 ROS2 实车控制**。它复用 `Planner` 和 `SimHandoff`；每个阶段以独立合成反馈推进。

- `Planner`：拒绝斜向道路图；起点接入也须是经过检查的单轴或 L 型路径，局部接入上限为 0.35m；正交航向保持不变直到目标停车点，必要时生成原地 `TURN_90`（180° 拆为两次），不把路线方向当车头方向。圆形保守包络已覆盖各航向，转动不另外缩小包络；尚未实现矩形车体模型。起始实测航向与四个基准角之一误差最多 3°，超出则拒绝，允许的偏差不是自动校正。`path` 是几何路线，`actions` 是压缩后的预览动作列表，仍不是实际电机命令。
- `SimMission`：`NAV`（每段 ACTION_DONE 匹配当前编号/停车位姿）→ `QR_CONFIRM`（三帧）→ `WAIT_QR_ACK` → `WAIT_QR_DONE` → `WAIT_QR_IDLE`（RAW 视角且稳定）→ `NAV`（根据合成实测停车位姿重规划，递增会话）→ `RAW_CONFIRM` → `RAW_ALIGN` → `WAIT_STEP`（逐轴微移）→ `READY_PICK`（模拟视觉到位，不是夹爪动作）。旧会话、重放、过期事件不能推进；错误到位反馈、冲突、掉线、超时、姿态漂移、越界微移退出到 `ABORT`，清空待执行预览。`TICK` 必须由调用方定期提供，否则无事件时 Python 状态机不会自主触发超时。
- 微调：以旧雅可比反解作为建议，选择绝对值较大的一轴，限制单步不超过 20mm；锁定目标航向，将单轴车体位移变换到 map，检查静态包络及锁路，累计偏离观察位不超过 80mm（仿真假设）。每步必须有匹配编号且停稳的位姿反馈。此选择不保证强耦合雅可比下实际误差单调收敛；真实控制前应进行标定、限幅与收敛验证。
- 合成事件字段：`source=SIM_TEST_ONLY`、`session`、`seq`、`stamp`（同一仿真单调时间域的秒）、`type`，以及按事件需要的 `pose`、`stationary`、`action`、`step`、`view`、`pitch_stable` 等；见 `python3 preview.py mission-demo` 输出。同一会话 seq 递增，反馈时效 0.4s，单阶段 15s，全任务 120s（仿真值）。`QR_DONE` 与 `CAMERA_IDLE` 由仿真适配层构造；实际 V4 的 DONE,QR 与 IDLE 不携带会话号，未来真实桥接必须关联当前扫码事务并拒绝旧帧。当前并未修改 STM32 的 ACK 后任务启动行为。
- `preview.py ros` 仍是单目标静止预览，不运行连续仿真 FSM；Path 朝向使用固定航向和显式转向终点，非路线切线。RViz 展示未做实机 ROS2 测试。

离线示例（所有尺寸/坐标/时间均为合成值）：QR 观察位 (1.779545, 1.2, 0 rad)，路线 2.279545m；原料盘观察位 (1.2, 1.895455, pi/2 rad)，路线 1.275m。后者到位后只原地转 90° 一次；视觉微调单轴，最终 `READY_PICK`。不是比赛速度或现场距离承诺。较远 QR 观察位是保守圆包络和目标避障半径共同限制的结果，必须用实测尺寸、实际光轴及有效识别距离重新计算。

阶段 2 已知未完成：真实 V4 -> ROS2 观测桥、STM32 停车/等待 DONE/IDLE/真实控制回执、移动定位 TF、动态障碍与完整车辆碰撞/制动检查、相机/升降/Pitch/夹爪实测标定、实车验收。不能将 `sim_ready_pick` 当抓取授权。完整路线计划、状态机及数据回放只用于开发和审查。
