# 第二阶段：CAR-LINK V1 虚拟串口测试闭环

**范围：仅 TEST 环境，不是实际运动控制软件。** 不修改 STM32 固件和原 `car_serial_link/src/serial_bridge_node.cpp` 的只读边界。

## 已实现的环节

1. 从 `rule_mission_core.build_mission()` 的原有 Dijkstra 比赛路线生成 `SimStep`，不自建另一套路径规划。
2. `mission_wire_sim.py` 使用 Python 标准库 `pty.openpty()` 创建匿名 Linux PTY。只读写本进程新建的文件描述符，不提供 `/dev/tty*` 可配置串口参数。
3. 发送 CAR-LINK V1 `A5 5A`、Version、Type、Flags、Seq、Length、Timestamp、Payload、CRC16-CCITT-FALSE 帧。验证长度、CRC、分包和噪声恢复。
4. 0xE1 `TEST_HELLO` 的 `CAR_LINK_MOCK_ONLY_V1` 标志必须先收到。0xE0 `TEST_STEP` 是**本地测试专用**扩展（正式 STM32 不应支持），承载任务步骤编号、阶段、目标路网节点、批次、颜色和环位。绝不发送正式运动控制消息 `0x80-0x87`。
5. 模拟 MCU 执行 0xE0 后，使用 B2 `0x90` ACK、`0x92` STARTED、`0x94` COMPLETED / `0x95` FAILED 返回，并以 0xE2 测试消息反馈二维码内容。
6. `SimExecutor` 只对正确 `cmd_id`、收到 ACK 后的 COMPLETED 推进下一步。扫码反馈不匹配、超时、CRC 错误或 FAILED 会锁定 FAULT；模拟 MCU 去重相同命令号。

## 在 Ubuntu 项目根目录验证

```bash
cd ~/Desktop/car
bash algorithm/car_ws/bash/test_mission_wire_sim.sh
```

以上包含 17 项无 ROS Python 单元测试，以及原有 Dijkstra 路线生成后的 PTY 端到端运行（该集成测试需在完整 Git 仓库中执行）。

单独演示：

```bash
cd ~/Desktop/car/algorithm/car_ws/src/car_rule_mission
python3 mission_wire_sim.py --start-zone upper_right --block B3-C3
python3 mission_wire_sim.py --fail-at 3              # 应输出 FAULT 和非零退出码
python3 mission_wire_sim.py --drop-ack-at 2          # 应输出 FAULT
python3 mission_wire_sim.py --corrupt-crc-at 2       # 应输出 FAULT
python3 mission_wire_sim.py --corrupt-qr             # 应输出 FAULT
python3 mission_wire_sim.py --duplicate-at 3        # 应输出 DONE 且执行次数不增长
```

## 不能从测试推导出的结论

- `MOVE_NODE` 是道路拓扑锚点，不是经过标定的实车停车位姿或下位机可直接执行的 `SET_POSITION` 参数。
- 当前没有实际 STM32 控制权仲裁、命令载荷对齐、物料检测、夹爪动作验证，也没有 Nav2 动态 TF 闭环。
- 0xE0/0xE1/0xE2 只是测试扩展；**不能上传到 STM32 固件作为正式协议**。
- 真实串口桥接仍为只读。本阶段软件回路验证只覆盖同一进程内的两个 PTY 端点，不等同于两个 ROS 2 节点与实体设备之间的验收。

## 后续工作

先在 Ubuntu 运行上述脚本并保存测试日志；之后在专用 ROS 2 TEST 域中将步骤和反馈映射为可视化状态，再单独实现与真实 STM32 固件协商的正式 B1 控制协议，经过控制权和运动边界验收后才可考虑实车发送。
