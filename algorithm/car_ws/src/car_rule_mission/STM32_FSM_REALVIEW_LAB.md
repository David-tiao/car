# 第五阶段 C：保留虚拟 STM32 端口，在「运行集」查看比赛状态机

## 核心设计决定

**现有模拟 STM32 端口没有被删除。** 原有代码保持：
- `algorithm/car_ws/src/car_rule_mission/mission_wire_sim.py` — CAR-LINK V1 二进制编解码、CRC16、`MockPeer`、去重与故障注入；
- `algorithm/car_ws/src/car_rule_mission/mission_test_session.py` — 匿名 PTY `pty.openpty()`、`TEST_HELLO` 握手、逐命令 ACK / COMPLETED、任务模拟状态机；
- `algorithm/car_ws/src/car_rule_mission/mission_ros_test.py` — 原来的 ROS 2 TEST 回放；
- `/api/test/mission_replay` 及 TEST 网页面板 — 原接口不变。

这次新增一个 **运行集可见、底层严格 MOCK 的隔离实验面板**：
`http://127.0.0.1:8090/real` 下的「运行集 · 模拟 STM32 PTY 状态机实验」。

为什么放在运行集仍然安全？新面板请求的固定是
`/api/test/stm32_fsm_lab`，返回 `source_type=MOCK`、`mode=SIM_ONLY`、
`hardware_authorized=false`、`model_kind=PYTHON_SIMEXECUTOR_NOT_COMPILED_STM32`。
绝不写入 `/api/real/*`、`/cmd_vel` 或实体 UART 设备，也不会把路网节点坐标标为真实 /odom。
原运行集实车监控仍独立展示真正收到的数据。

## 操作方式

运行环境：**Ubuntu / Linux**（使用匿名 PTY），Python 3。
不需要 ROS 2、OpenMV 实机、STM32 实物或外接串口。

```bash
cd ~/Desktop/car
python3 sim2real/server.py --host 127.0.0.1 --port 8090
```

浏览器访问 `http://127.0.0.1:8090/real`。
在新增的独立区域中填写任务码（默认 `156+123+516+231`）、起点和封锁道路；
点击「初始化虚拟 STM32」后可以「执行一步」或「运行至 DONE/FAULT」。

支持故障注入：`failed_at_3`（第三命令 FAILED）、
`missing_ack_at_2`（第二命令丢 ACK）。
出现 FAULT 后继续执行不会推进任务；必须重置/新建会话。
封锁导致 `NO_ROUTE` 时明确拒绝创建任务。更换二维码任务码后重新初始化。

**注意：** HTTP POST 仅允许服务器本机 loopback 且同源。
如果从其他电脑访问 Ubuntu 的网页，实验区仍是只读或提交被拒绝；
不要为了开启外网按钮而移除这项安全限制。

## 第二层：基于真实 STM32 C 任务表的安全反馈门控

新增：
- `STM32F4/App/race_task_gate.h/.c`：依赖原有 `RaceTaskPlan` 的**纯 C 无硬件状态机**。
- `STM32F4/tests/race/test_task_gate.c`：原生 GCC 执行，检查 34 个真实 C 任务动作的推进条件、六次 PICK、六次粗加工放置、六次取回、六次存放、三次 STACK 和返程，及不同批次的同色映射。

此门控只有 `READY → WAIT_ACK → WAIT_RESULT → READY/DONE`，错误时锁存 `FAULT`。
错误 cmd_id、提前 COMPLETED、重复 ACK、重复 COMPLETED 不得推进计数；
还检查 ACK/完成反馈超时、拒绝、执行失败和 CANCEL。
输出 `RaceTaskTestIntent.hardware_authorized` 固定为 0。
**它目前未挂载到 `AppRace_Poll()` 或任何真实电机执行链**，是下一阶段连接真实业务反馈的准备模块。

网页目前显示的是 **CAR-LINK 42 步 Python SimExecutor/PTY 模型**，
不是 MCU 已烧录的 C 固件。C 层有 **34 步** 任务计划，另行用 GCC 测试。
两者建模颗粒度不同；42 不应被误解为 STM32 C 固件恰好也有 42 步。

## 一键离线验收

```bash
cd ~/Desktop/car
bash algorithm/car_ws/bash/test_stm32_fsm_realview.sh
```

本脚本会：
1. 跑已有任务模拟、CAR-LINK 二进制 PTY、ROS TEST 会话等 Python 回归。
2. 真正创建匿名 PTY，验证完整 DONE、指令失败 FAULT、ACK 丢失 FAULT、
   封路/非法任务拒绝、关闭文件描述符和 REAL 通道隔离。
3. 使用 `gcc -std=c99 -Wall -Wextra -Werror` 编译并运行真正的
   `STM32F4/App/race_task_plan.c` + 新 `race_task_gate.c` 的单元测试，
   检查 34 个 C 任务动作的严格反馈门控。
4. 检查 Python 源码语法，如安装 Node.js 则检查前端脚本语法。

**验收状态：待在你的 Ubuntu 上实际运行，不能直接声明 PASS。**
在此之前此阶段保持 Draft PR，不直接合并。

## 尚未实现的真实比赛控制

- 这套 TEST/PTy 模拟 **没有** 真实里程计、雷达定位、实际停车点、机构传感器回执。
- STM32 C 新门控没有接入 `AppRace_Poll` 的底盘、转盘、夹爪驱动链。
- ROS 2 到真实 STM32 的 CAR-LINK 仍是只读，**不是驱动实车的授权接口**。
- OpenMV `GRIP_TARGET_CALIBRATED=False`、环位纠偏矩阵为零仍未校准。
- C `RaceTaskPlan` 的动作结构与 ROS 的道路级 Python 步骤需在未来建立正式映射；
  本次不伪造这种映射。

这样安排的意义：保留以往全部模拟能力，运行集可观察/实验状态机，
且真实监控与模拟始终物理分区并带有明显提示；为下一轮电控动作接口
对接提供一套可重复的离线验收基线。
