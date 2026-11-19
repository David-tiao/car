# STM32F407 ↔ Raspberry Pi 5 Serial 通信协议

**协议名称：** CAR-LINK V1.0
**适用项目：** 工创智能搬运小车
**通信方式：** UART4（建议 PC10 TX / PC11 RX，待硬件确认）
**串口参数：** 115200 / 8N1
**通信方向：** 双向
**协议状态：** 设计方案，尚未在 STM32 固件中实现

## 1. 协议分类

| 分类        |   | 功能                |   | 通信方向           |
| --------- | - | ----------------- | - | -------------- |
| A 类：物理量   |   | 位置、速度、编码器、IMU、电压等 |   | STM32 → Pi     |
| B1 类：控制指令 |   | 速度、位置、机械机构、任务控制   |   | Pi → STM32     |
| B2 类：执行反馈 |   | 指令接收、拒绝、开始、完成、失败  |   | STM32 → Pi     |
| B3 类：系统事件 |   | 心跳、比赛状态、视觉事件、故障   |   | 双向或 STM32 → Pi |

## 2. 串口数据帧格式

所有协议使用统一二进制帧：

| 字段        | 长度         | 说明                |
| --------- | ---------- | ----------------- |
| SOF       | 2 Byte     | 固定 `0xA5 0x5A`    |
| Version   | 1 Byte     | 协议版本 `0x01`       |
| Type      | 1 Byte     | 消息类型编号            |
| Flags     | 1 Byte     | 标志位，bit0 表示请求 ACK |
| Seq       | 2 Byte     | 帧序号               |
| Length    | 2 Byte     | Payload 长度        |
| Timestamp | 4 Byte     | 发送端运行时间，ms        |
| Payload   | 0～192 Byte | 实际数据              |
| CRC16     | 2 Byte     | CRC16-CCITT-FALSE |

约定：

- 所有多字节整数采用小端序。
- CRC16：多项式 `0x1021`，初值 `0xFFFF`，从 Version 校验至 Payload 末尾。
- 数据帧总长度 = `15 + Length` 字节。
- 接收方按 SOF、Length 和 CRC 拆包并验证。

## 3. A 类：物理量协议

| ID     | 消息名称               | 数据内容                     | 频率       |
| ------ | ------------------ | ------------------------ | -------- |
| `0x10` | CHASSIS            | X、Y、Yaw、vx、vy、wz         | 20 Hz    |
| `0x11` | WHEELS             | 四轮目标速度、实际速度、PID 输出       | 20 Hz    |
| `0x12` | IMU                | Roll、Pitch、Yaw、IMU 有效状态  | 20 Hz    |
| `0x13` | MECHANISM          | 夹爪、转盘、升降、伸缩位置及有效状态       | 10 Hz    |
| `0x14` | POWER              | 驱动电压、主板电压                | 2 Hz     |
| `0x15` | VISION_MEASUREMENT | 目标类型、像素坐标、观测有效性          | 按需       |
| `0x16` | CHASSIS_TARGET     | 目标 X、Y、Yaw、vx、vy、wz、控制模式 | 10～20 Hz |

### A 类数据单位

| 物理量            | 类型     | 单位           |
| -------------- | ------ | ------------ |
| X、Y、目标位置       | int32  | mm           |
| Yaw、Roll、Pitch | int32  | mrad         |
| vx、vy          | int32  | mm/s         |
| wz             | int32  | mrad/s       |
| 电机速度           | int32  | pulse/s      |
| PID 输出         | int16  | STM32 归一化输出值 |
| 机械角度           | int32  | mrad         |
| 升降、伸缩位置        | int32  | mm           |
| 电压             | uint16 | mV           |
| 像素坐标 u、v       | int16  | pixel        |
| 有效标志           | uint8  | 0 无效 / 1 有效  |

所有 A 类数据应包含采样时间信息。无法获得的实测值必须标记无效，不得用目标值冒充实际值。

## 4. B1 类：控制指令协议

通信方向：**Pi → STM32**

| ID     | 指令名称              | 参数                        | 功能          |
| ------ | ----------------- | ------------------------- | ----------- |
| `0x80` | REQUEST_CONTROL   | session_id                | 申请 ROS2 控制权 |
| `0x81` | RELEASE_CONTROL   | session_id                | 释放控制权       |
| `0x82` | SET_VELOCITY      | vx、vy、wz、session_id       | 设置底盘速度      |
| `0x83` | SET_POSITION      | X、Y、Yaw、cmd_id、session_id | 设置底盘目标位置    |
| `0x84` | STOP              | cmd_id、停止原因               | 请求安全停车      |
| `0x85` | START_MISSION     | cmd_id、任务编号               | 请求启动任务      |
| `0x86` | CANCEL_MISSION    | cmd_id、任务编号               | 请求取消任务      |
| `0x87` | MECHANISM_COMMAND | cmd_id、机构编号、动作、参数         | 请求机械机构动作    |

说明：

- `SET_VELOCITY` 采用 mm/s、mrad/s。
- `SET_POSITION` 采用 mm、mrad。
- STM32 必须校验参数范围与控制权。
- 当前比赛模式运行时不得被 ROS2 直接抢占控制权。
- 第一版只读联调阶段禁止启用运动指令。

## 5. B2 类：执行反馈协议

通信方向：**STM32 → Pi**

| ID     | 消息名称              | 参数            | 含义     |
| ------ | ----------------- | ------------- | ------ |
| `0x90` | COMMAND_ACK       | cmd_id、原指令、状态 | 指令已接受  |
| `0x91` | COMMAND_REJECTED  | cmd_id、错误码    | 指令被拒绝  |
| `0x92` | COMMAND_STARTED   | cmd_id        | 动作开始   |
| `0x93` | COMMAND_PROGRESS  | cmd_id、进度、阶段  | 动作执行中  |
| `0x94` | COMMAND_COMPLETED | cmd_id、结果     | 动作完成   |
| `0x95` | COMMAND_FAILED    | cmd_id、错误码    | 动作失败   |
| `0x96` | CONTROL_STATUS    | 控制权、模式、运行状态   | 控制状态反馈 |

指令执行流程：

```text
Pi → STM32：SET_POSITION
STM32 → Pi：COMMAND_ACK
STM32 → Pi：COMMAND_STARTED
STM32 → Pi：COMMAND_PROGRESS
STM32 → Pi：COMMAND_COMPLETED
```

**COMMAND_ACK 仅表示接收成功，不代表动作完成。**

每次离散动作使用 `cmd_id` 关联反馈；重复接收同一动作时不能重复执行。

## 6. B3 类：系统事件与链路管理

| ID     | 消息名称          | 通信方向       | 数据内容             |
| ------ | ------------- | ---------- | ---------------- |
| `0x01` | HELLO         | 双向         | 设备编号、协议版本、会话信息   |
| `0x02` | HEARTBEAT     | 双向         | 运行状态、在线心跳        |
| `0x03` | LINK_STATUS   | 双向         | 丢包数、CRC 错误数、连接状态 |
| `0x97` | MISSION_EVENT | STM32 → Pi | 比赛状态、动作编号、任务结果   |
| `0x98` | VISION_EVENT  | STM32 → Pi | 二维码、识别成功、目标丢失等事件 |
| `0x99` | FAULT_EVENT   | STM32 → Pi | 故障类型、错误码、故障来源    |

## 7. 通信安全与反馈规则

| 项目     | 规则                     |
| ------ | ---------------------- |
| 默认模式   | 只读监控                   |
| 心跳周期   | 100 ms                 |
| 速度命令周期 | 50 ms                  |
| 遥控速度失效 | 200 ms，安全停车            |
| 遥控链路超时 | 500 ms，停车并撤销遥控权        |
| 控制权    | STM32 比赛 / ROS2 遥控互斥   |
| 离散命令   | ACK + cmd_id 去重 + 最终结果 |
| 连续速度命令 | 新数据覆盖旧数据，不重发过期指令       |
| 错误数据帧  | 丢弃、不执行                 |
| 重启重连   | 重新握手并申请控制权             |
| 监控模式断线 | 仅报告失联，不干扰 STM32 自主比赛   |

## 8. ROS2 Topic 对应关系

| Serial 消息          | ROS2 Topic                            |
| ------------------ | ------------------------------------- |
| CHASSIS            | `/odom`、`/car/chassis/telemetry`      |
| WHEELS             | `/car/chassis/wheels`、`/joint_states` |
| IMU                | `/imu/data`                           |
| MECHANISM          | `/car/mechanism/state`                |
| POWER              | `/car/chassis/telemetry`              |
| VISION_MEASUREMENT | `/car/vision/event`                   |
| CHASSIS_TARGET     | `/car/chassis/telemetry`              |
| MISSION_EVENT      | `/car/mission/state`                  |
| VISION_EVENT       | `/car/vision/event`                   |
| FAULT_EVENT        | `/car/debug/diagnostics`              |
| LINK_STATUS        | `/car/stm32/link_status`              |
| SET_VELOCITY       | 未来由 `/cmd_vel` 经过控制权管理转换              |

## 9. 协议实现要求

STM32 端：

- 独立串口收发，不占用 OpenMV USART3。
- 中断或 DMA 接收，主循环解析。
- 从现有底盘、IMU、机构和比赛状态机获取数据。
- 不在 1 ms 电机控制中断中阻塞通信。
- 所有运动命令经过控制权和安全检查。

树莓派端：

- 使用 C++17 / ROS2 Humble。
- 实现统一的帧解析、CRC16 和消息分发。
- 发布标准 ROS2 Topic。
- 管理心跳、重连及命令反馈。
- 保留现有 ULog43 离线调试功能。

---

**协议设计总结：**

A 类负责 **物理量传输**，B 类负责 **控制、反馈与系统事件**。两类消息共用 CAR-LINK V1 二进制串口帧，但发送周期、确认机制和安全要求不同。

当前优先实现 **A 类全部可获取的物理量 + B3 链路管理 + B2 状态反馈**；主动底盘与机械机构控制等待实车安全验证后启用。
