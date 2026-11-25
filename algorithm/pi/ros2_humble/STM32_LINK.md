# ROS 2 Humble ↔ STM32F407：现有链路与未来双向通信

> 本文核对项目**现有源码**，不是通用教程照搬，更不是宣称电机能受 ROS 2 控制。当前首个安全目标：**STM32F407 USART2 ULog43 → Pi5 C++ 只读桥 → ROS 2 /odom**。

## 1. 三条不同的通信线路，不许互换

| 线路 | 实际对象/端口 | 协议 | 当前功能 | 禁忌 |
| --- | --- | --- | --- | --- |
| **ULog43（真实遥测）** | STM32F407 `USART2`，`PD5=TX`, `PD6=RX`；115200/8N1；经 3.3V TTL USB-UART 接树莓派 | **43 列数值 CSV**，无表头，CRLF，每约 50ms | STM32 → Pi **只读**，`car_stm32_bridge` 解析发布 ROS 2 | 不可往这个 CSV 日志串口输入 `/cmd_vel`；USART2 目前连 microSD UART 日志模块，若要并联监听必须先测电气负载 |
| **OpenMV 视觉 V4** | OpenMV H7 Plus UART3 ↔ STM32 `USART3` PD8/PD9；115200/8N1 | `@SEQ,LEN,BODY*XX\n`，XOR 校验 | STM32 ↔ OpenMV 图像任务/ACK/视觉点位 | 不可直接接入 Pi 端或把它当 ROS 2 ↔ STM32 电机通道 |
| **CAR-LINK V1（仿真/测试）** | Linux PTY / 仿真 MCU / 原型 `car_serial_link` | binary + CRC16 | 测试 `ACK/PROGRESS/COMPLETED`，不是实车协议 | 模拟反馈 ≠ 真实 STM32 固件已经实现此协议 |

源码定位：

- [STM32 ULog43 的硬件声明](../../../STM32F4/Hardware/bsp_ulog.h)：`PD5 TX`、`PD6 RX`，约 20Hz，`ULOG_PULSE_PER_METER=1750`。
- [C++ ROS 2 只读桥](../../car_ws/src/car_stm32_bridge/src/ulog43_bridge_node.cpp)：`open(..., O_RDONLY|O_NOCTTY|O_NONBLOCK)`，没有 `write()` 和 ROS 2 控制话题订阅。
- [CSV 解析器](../../car_ws/src/car_stm32_bridge/src/ulog43_parser.cpp)：恰好 43 列，浮点数、非空、MCU 毫秒时间校验；节点处理串口断联和时间倒退。
- [桥接配置](../../car_ws/src/car_stm32_bridge/config/bridge.yaml)：`baud=115200`，`publish_tf=false`，里程/轮速参数尚需实车复核。
- [启动文件](../../car_ws/src/car_stm32_bridge/launch/monitor.launch.xml)：`port` 参数进入只读桥。
- [OpenMV V4 串口协议](../../../STM32F4/docs/uart_protocol.md)：`USART3` 的 `@seq,len,body*XOR`，与 ULog43 **完全不同**。
- [全仓库接口合同](../../../docs/PROTOCOLS_AND_DATA_FLOW.md)：严格区分真实只读遥测、OpenMV V4 及 TEST CAR-LINK。

## 2. 真实的接线（第一阶段：只监听，绝不发送）

```text
STM32F407                                      Raspberry Pi 5
+---------------------------------+           +-----------------------+
| USART2 PD5 TX (3.3V TTL) -------+---------> | USB-to-TTL adapter RX |
|              GND ----------------+---------> | USB-to-TTL adapter GND|
|                                 |           |          USB plug      |
| USART2 PD6 RX (不连接！)         |           +-------------+---------+
+---------------------------------+                         |
                                                  Pi5 USB-A socket
                                                  /dev/ttyUSB0 (example)
```

- **接线前断电**、核对共地、引脚实际输出电平和开发板丝印；只用**3.3V TTL** 适配器，**不能**用 ±12V RS-232 端口。USB-UART 的 VCC/5V 引脚**不接** STM32 或 Pi GPIO。
- 最初只连 **TX→RX + GND** 两线，USB-UART 的 TX 悬空，避免对固件尚未支持的命令口写入数据。
- 原 USART2 已面向 microSD UART 记录模块。想同时保持存卡并给 Pi 监听时，先确认多路接收器共享一根 TX 不引入电平/负载/地环路问题；不满足时用隔离或改固件选择单一路径。
- `/dev/ttyUSB0` 不是固定设备名，插拔后可能变化；优先使用 `/dev/serial/by-id/...`。要明确识别 USB-UART 和雷达对应的**不同**串口。
- Pi5 上应用程序无需直接使用 GPIO UART，因此本方案不改 `config.txt`、不占用 OpenMV/STM32 USART3。

## 3. 已有 C++ 桥节点的工作原理

```text
STM32F407 USART2 TX
   -> ASCII "0,1,...,42\r\n"（实际 43 列数值）
   -> USB-to-TTL 3.3V RX -> Linux /dev/serial/by-id/...
   -> Docker 映射为 /dev/car_ulog (read-only)
   -> car_stm32_bridge::Ulog43Bridge [C++, O_RDONLY]
   -> parse_ulog43: 43 列、合法浮点数、源时间单调
   +-> /car/debug/ulog43          car_interfaces/msg/Ulog43
   +-> /car/chassis/telemetry     car_interfaces/msg/ChassisTelemetry
   +-> /car/chassis/wheels        car_interfaces/msg/WheelTelemetry
   +-> /car/mission/state         car_interfaces/msg/MissionState
   +-> /odom                      nav_msgs/msg/Odometry
   +-> /joint_states              sensor_msgs/msg/JointState（可配置）
```

实际字段下标（零起始）：`[5]` 为 MCU 运行毫秒、`[14]/[15]` 目标/实测角速度、`[16]/[17]` 航向角、`[28..31]` 四轮编码器脉冲每秒、`[32]/[33]/[34]` x/y/累计里程。相关数值的单位、轮序和正方向**以本车 STM32 固件实测为准**。

桥会发布一次数据后停止（如果源停发），但当前实现**没有永久在线的健康心跳/TTL 事件**；不能看到旧 `/odom` 就判断此刻 STM32 在线。实机联调需再增加独立 stale-data 检查和离线安全门。当前 `publish_tf=false`，不代表 `odom→base_link` TF 已就绪，须先验证 TF 所有权，避免重复广播。

## 4. 首次无硬件验证

在 Docker 容器中先编译，然后运行现有 `car_bringup` 的**合成 CSV 回放**：

```bash
bash algorithm/pi/ros2_humble/run.sh image
bash algorithm/pi/ros2_humble/run.sh build
bash algorithm/pi/ros2_humble/run.sh replay
```

第二终端：

```bash
bash algorithm/pi/ros2_humble/run.sh topics
bash algorithm/pi/ros2_humble/run.sh echo-odom
```

预期出现 `/odom`、`/car/debug/ulog43` 和 `/car/chassis/*`，但此数据是**SYNTHETIC 模拟数据**，无法证明串口硬件连通。

## 5. 首次真实只读串口联调

先关闭其它抢占同一个 USB-UART 的程序，然后：

```bash
bash algorithm/pi/ros2_humble/run.sh doctor
bash algorithm/pi/ros2_humble/run.sh monitor /dev/serial/by-id/YOUR-ADAPTER
```

第二终端：

```bash
bash algorithm/pi/ros2_humble/run.sh echo-odom
```

ROS 2 原始命令（在 Docker 的 ROS2/overlay 环境中；也可以 `run.sh shell`）：

```bash
ros2 node list
ros2 topic list -t
ros2 topic hz /odom
ros2 topic echo /car/chassis/telemetry --once
ros2 topic echo /odom --once
```

成功标准：**真实 STM32 连续发出有效 43 列帧，ROS2 频率稳定，静止状态数据与实测一致，拔线后新帧停止，重新插线可恢复；没有任何电机动作**。不要只看 ros2 node list 就判断串口已经成功。

## 6. 未来 Pi → STM32 真正控制：需要独立协议合同和固件

假定将来打通 Nav2/路径规划到 STM32 底盘，推荐：

```text
     Nav2 / Planner
          |
          v
 /cmd_vel (geometry_msgs/TwistStamped 或 Twist，需要统一)
          |
          v
  本地唯一的安全仲裁器
   - 实体急停 + enable 授权
   - 模式仲裁 / 离线停止
   - 速度/加速度/转向/夹爪限幅
   - TTL 和过期拒绝
          |
          v
 Pi -> STM32 全双工接口（新增/明确一个有 RX 的通信链路）
   - 帧头 magic/version
   - kind/seq/session/command_id
   - length + payload + CRC16
   - time validity / ACK / reject / completion / telemetry
          |
          v
 STM32 firmware
   - CRC, seq anti-replay, state/enable checks
   - true wheel speed PID/mecanum inverse kinematics
   - watchdog: command loss -> safe stop
   - E-stop independent of ROS2
   - report actual measured feedback & failure
```

这只是**建议设计，不是仓库已存在的实装接口**。可考虑另设 STM32 可用 UART（硬件引脚、DMA、中断、用途、电平和空闲资源须与电控负责人签字确认），或先新增经过固件验证的 USB CDC。**不能假设 Pi5 当前 USB-UART 的 RX-only 接线已经具备 Pi→STM32 下发能力，更不能向 USART3 的 OpenMV 视觉 UART 混发电机指令。**

建议协议中每条命令都携带：`cmd_id`、会话/阶段、物理单位和坐标系（车体 `base_link`）、单调递增序列、TTL 和安全模式；底盘固件独立执行速度上限、短时指令看门狗与急停，回复区分 **ACK（收到了）**、**COMPLETED（实际执行完成）**、**REJECTED**、**STOPPED**。收到一个 ROS2 消息 ≠ STM32 已执行；网络/串口失联时必须由 STM32 自己停机。

下一阶段用无电机/轮子离地的**独立可写协议台架**测试：分包粘包、CRC、重复命令、乱序、损坏帧、断线、超时、复位后序列同步、急停、掉电；通过后才考虑受限的微速度闭环。

## 7. 为何本 PR 不直接写 motor sender？

现在仓库 `car_serial_link` 的 CAR-LINK 运行在模拟/PTY 上，固件端还没有可靠实装的真实命令和执行回执。盲目写一个订阅 `/cmd_vel` 后往串口发字节的程序，**没有双方确定的协议就没有可信解析、停止或安全确认**。本 PR 所做的是先让已有的真实 C++ 遥测桥在 Pi5 上**有可复现的部署方式**，它正好可以作为后续双向方案的回读验证。

如要开发实车双向控制，请先提供/确认 STM32 上**预留的可写 UART/USB CDC 接口、引脚/波特率、真实固件版本和实物急停方式**，再在隔离分支开发有安全门的 C++ `ros2_control` 或独立串口网关。
