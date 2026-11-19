# 第四阶段：比赛全流程自动化软件验收（SIM_ONLY）

## 当前目的

在没有实车条件下，自动测试当前 **9 站、8 段 Dijkstra 道路规划 + 任务码编排 + CAR-LINK 虚拟 STM32**。验收对象是**软件逻辑与虚拟串口通信**，不是实车运动、真实视觉识别、定位精度或实际夹取成功率。

这套测试不会向 `/cmd_vel`、真实 `/dev/tty*` 或 STM32F4 发送命令。

## 命令

首先更新第四阶段开发分支：

```bash
cd ~/Desktop/car
git fetch origin
git switch --track origin/feat/competition-batch-acceptance-20261120
```

然后一键执行：

```bash
bash algorithm/car_ws/bash/test_competition_batch.sh
```

默认批量生成 100 个有固定随机种子的场景，并取最多 5 个**可达场景**走匿名 PTY 二进制模拟通信。全部场景仍会运行纯软件任务状态机。默认输出：

- `/tmp/car_phase4_acceptance/acceptance.json`：包含每个场景、任务码、道路封锁、任务统计、失败原因。
- `/tmp/car_phase4_acceptance/acceptance.md`：简明验收结论、可达/不可达场景统计和故障注入矩阵。

自定义规模和种子：

```bash
bash algorithm/car_ws/bash/test_competition_batch.sh \
  --cases 300 --seed 20271120 --wire-samples 10 \
  --output-dir /tmp/car_phase4_acceptance_300
```

**测试通过判据：**

1. 可达场景：8 段路线合法、不得经过封锁道路；24 次物料相关模拟动作（12 次抓取+12 次放置，其中 3 次码垛）和返程确认均完成。
2. 不可达场景：`NO_ROUTE`，没有虚构路径，也不生成虚构动作。
3. 二维码任务：两批颜色集合与落位对应关系正确；第二批码垛对应第一批暂存底环。
4. 通信故障矩阵：`FAILED`、丢失 ACK、CRC 错误、扫码内容错误应进入 `FAULT`；重复命令号不得造成额外动作。
5. 报告必须标注 `SIM_ONLY`、`hardware_authorized=false`，不能伪装成实机测试。

脚本若发现测试失败、计划错误或报告保存失败，使用非零退出码，便于未来接入 GitHub CI。

## 与已有模块关系

- 重用 `rule_mission_core.py` 的地图和 Dijkstra。
- 重用 `mission_execution.py` 的任务码解析与模拟状态机。
- 只在少量可达场景重用 `mission_wire_sim.py` 的 PTY 帧级闭环，以限制测试耗时。
- 不依赖 ROS 2，可在安装 Python 3 的 Ubuntu 上运行。运行此脚本时无需启动 TEST 网页。

**明确未完成：** 与真实 STM32F4 的控制权协商、正式 `SET_POSITION` 帧载荷对齐、雷达/视觉标定、闭环里程计控制、真实机构动作、实体障碍物安全验证。上述功能不应被此报告的 PASS 替代。

## 第四阶段人工验收流程

1. 运行一键脚本，确认单元测试结果和批量模拟报告均为 PASS。
2. 查看报告中 `PASS_SIM`、`PASS_NO_ROUTE`、`FAIL` 数值及故障矩阵。
3. 重复运行相同随机种子，场景配置与结论应一致。
4. 分别修改 `--seed` 和 `--cases`，扩大输入覆盖。
5. 保存成功报告供后续硬件联调对照。

## 当前状态

代码已经写入开发分支，尚需在真实 Ubuntu 工作空间运行本脚本，不能将源码审查等同于已通过完整软件验收。验收通过后才考虑合并本阶段 PR。
