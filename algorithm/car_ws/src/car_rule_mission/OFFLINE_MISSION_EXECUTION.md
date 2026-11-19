# 2027 智能搬运：离线比赛执行状态机（第一阶段）

**位置：** `algorithm/car_ws/src/car_rule_mission/`  
**分支：** `feat/offline-mission-workflow-20261120`  
**安全边界：** **SIM_ONLY**。不接串口、不发 `/cmd_vel`、不产生真实 STM32 运动授权。请勿将这里的 `MOVE_NODE` 直接映射为实车电机指令。

## 已实现

1. `mission_execution.py`：严格校验四组任务码，复用 `rule_mission_core.build_mission()` 的 8 段 Dijkstra 路线，生成模拟任务步骤。
2. 使用第一批“颜色→暂存环号”映射第二批码垛，避免误把第四组粗加工环号当成暂存环号。
3. 模拟过程包含 **12 次抓取、9 次平面放置、3 次码垛、1 次扫码、1 次返程确认**，并且可以插入按路线节点行走的 `MOVE_NODE` 步骤。
4. `SimExecutor` 的每个 `cmd_id` 同一时刻仅允许一条在途命令，`ACK` 只确认接受，`COMPLETED` 才推进任务。拒绝、失败、超时、扫码内容不匹配会锁定 `FAULT`；丢弃旧命令号和重复回执。
5. `mission_sim_demo.py`：使用**既有**道路规划核心，给软件模拟执行器提供任务步骤，并自动给出合成 ACK/COMPLETED；支持注入执行失败，打印 JSON 统计。
6. `test_mission_execution.py`：纯 Python 单元测试；`bash/test_offline_mission.sh`：统一离线测试入口，包含原有 `test_rule_mission.py`。

## Ubuntu 验证步骤

在**仓库根目录**，进入本功能分支后运行：

```bash
cd ~/Desktop/car
git fetch origin
git switch feat/offline-mission-workflow-20261120
bash algorithm/car_ws/bash/test_offline_mission.sh
```

需要查看完整逐步执行记录：

```bash
cd ~/Desktop/car/algorithm/car_ws/src/car_rule_mission
python3 mission_sim_demo.py --start-zone upper_right --task-code '156+123+516+231' --block B3-C3
```

失败注入（应退出非零且摘要为 `FAULT`）：

```bash
python3 mission_sim_demo.py --fail-at 3
```

无可行路径（应输出 `NO_ROUTE`、退出码 3）：

```bash
python3 mission_sim_demo.py --block B3-C3 --block C2-C3
```

## 结果如何解读

- `DONE` 仅表示软件中**模拟**的所有事件依次完成，不代表成功夹到了实际物料。
- `pick_count=12`、`place_count=12`、`stack_count=3` 是执行器按成功模拟反馈统计，不是实际传感器量测。
- `MOVE_NODE` 使用道路拓扑节点，不是实车停车姿态，不包含位置标定或运动包络安全验收。
- `ACK` / `COMPLETED` 目前由演示器生成，不是 CAR-LINK UART 对端收到的真实回执。

## 后续开发边界与计划

1. 使用更细粒度的夹取/搬运安全条件（物料确实落到车上、机械状态有效、重复识别一致），不能仅凭模拟完成信号断言实物成功。
2. 构建与 `car_serial_link` 的**TEST 专用**传输适配器，通过独立虚拟串口接收任务意图并反馈；不修改真实串口只读桥接的安全边界。
3. 将任务状态汇总到 `car_mission` 的模拟话题，为网页显示和诊断提供逐步记录。
4. 实车到位前，既有 `rule_mission_preview.py` 继续只读，不提升它的 `commands_enabled` 权限。
5. 调试时明确区分真实比赛的雷达锁路数据与 `--demo` 中指定的人工障碍物。

## 本阶段验收记录

- 在隔离本地执行环境下，`test_mission_execution.py` 的 **7 个单元测试通过**。
- 本环境未安装运行本仓库 ROS 2 工作空间，因此**未进行** `colcon build`、现场 `/scan` 或 STM32 通信测试。
- 因当前执行环境无法直接克隆仓库，`bash/test_offline_mission.sh` 中依赖原 `rule_mission_core.py` 的完整集成测试需在你们 Ubuntu 工作空间运行。
