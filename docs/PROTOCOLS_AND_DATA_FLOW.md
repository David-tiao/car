# 通信协议、接口与数据流对齐说明

**状态基线**：2026-11-22，`main@1a7a696b`。此文描述当前仓库源码与已明确的协议边界。确切报文以 [STM32 USART3 协议](../STM32F4/docs/uart_protocol.md)、[OpenMV V4 联调](../algorithm/openmv/docs/README.md) 和 [根目录通信协议](../通信协议.md) 为参考，三者历史更新日期可能不同；有出入时**先对照真正运行的主机和相机源码，不任意混发协议**。

## 1. 按语义分类，而非按串口名混放

| 类别 | 典型字段 | 方向 | 单位与性质 |
| --- | --- | --- | --- |
| **物理量/观测** | `POS,*,dx,dy,cx,cy`、轮速、`/odom`、IMU、ULog43 | 传感器/STM32→应用 | 像素、米、rad、脉冲/s、ms；一定要有坐标系、时效和来源 |
| **任务/控制意图** | `JOB`、`ALIGN`、`STEP_DONE`、`CANCEL`、候选 `TwistStamped` | 主控→相机或 UI→PC | 意图/请求不是动作已执行，可能被安全门拒绝 |
| **执行反馈** | `ACK`、`DONE`、`ARRIVED`、`COMPLETED`、`PICK_OK` | 受控端→请求端 | 必须匹配任务 ID/session/step/cmd_id；ACK ≠ 完成 |
| **系统事件** | `READY,IDLE`、`ERR`、`LOST`、`TIMEOUT`、复位与离线 | 双向 | 生命周期/异常；必须能安全停止和恢复 |

真实遥测与模拟数据不是“都叫速度就可以混用”，还需 `source_type`、时间戳、消息序号和坐标框架等身份信息。

## 2. OpenMV H7 Plus ↔ STM32F407（V4）

- 物理链路：OpenMV UART3 P4(TX)→STM32 USART3 PD9(RX)，OpenMV P5(RX)←PD8(TX)，115200 bit/s、8N1、两板共地；真正接线/电平仍需实物检查。
- 当前四文件相机 `algorithm/openmv/openmv_all/config.py`：`UART_PROTOCOL='V4'`。可选旧 `V5` 需主控配套。请勿沿用旧版根 `通信协议.md` 中“algorithm/openmv 四文件版只使用 V5”的历史描述。
- 帧：`@SEQ,LEN,BODY*XX<LF>`。`SEQ` 为 0～65535 的无符号序号；`LEN` 为正文 ASCII 长度（包含正文逗号）；`XX` 为 `SEQ,LEN,BODY` 所有 ASCII 字节的逐字节 XOR 后两位大写十六进制；以单个 LF 结束。

真实测试示例（ASCII 最末添加换行字节）：

~~~text
@1,9,ALIGN,2,7*40
~~~

其中 `ALIGN,2,7` 长 9 字节；随意改校验字节应被接收器拒绝。接收器拒绝旧序号和重复序号；在特定 `READY,IDLE` 场景恢复会话基准。不能使用“收到一行文本”替代“收到完整且校验正确的帧”。

### 2.1 V4 常用正文与事务

| 发送方 | 正文 | 含义 |
| --- | --- | --- |
| 相机 | `READY,IDLE` | 就绪，主控可启动任务 |
| STM32 | `JOB,1` 或扫码会话扩展 `JOB,1,session` | QR 扫描；具体可选字段以当前相机接口为准 |
| 相机 | `ACK,JOB,1`、`QR,code,session`、`DONE,QR`、`IDLE` | 接受、二维码结果、完成、释放 |
| STM32 | `JOB,40` | 原料盘首次区域确认；相机默认首次锁盘依据三色几何，不是二色保证 |
| 相机 | `AREA_OK,cx,cy,r`；`IDLE` | 区域确认及释放，**可能分两次串口回调到达** |
| STM32 | `JOB,41` 或 `JOB,41,tol_x,tol_y` | 夹爪抓取工位对位；和真抓取成功不是同一回事 |
| 相机 | `POS,MATERIAL,0,dx,dy,cx,cy` 或 `ARRIVED,MATERIAL,0,dx,dy` | 位置/容差内到位 |
| STM32 | `ALIGN,target,session` | 开启目标 1/2/3 数字环一段对位会话 |
| 相机 | `ACK,ALIGN,target,session` | 接受环对位会话 |
| 相机 | `POS,RING,target,dx,dy,cx,cy,session,step` | 指定会话、指定步的新观测 |
| STM32 | `STEP_DONE,step,0/1` | 已停稳：0 原地再采，1 微移后再采 |
| STM32 | `ARRIVED,RING,target,dx,dy,session,count` | 连续稳定后结束 |
| STM32 | `STATUS/CANCEL/RESET` | 查询/取消/复位 |
| 相机 | `ERR/LOST/TIMEOUT` | 错误、目标丢失、任务超时 |

**注意历史差异**：旧文档曾把任意 `JOB,1,23` 写成“启动普通扫码”描述，但不同 V4 代码版本是否把第三字段当扫码 session 取决于实装版本；应做**双端同版本实测**。通用任务号列出 11～16、21～23、31、32、42，不代表当前四文件相机 V4 路径全部实现；当前 V4 联调文档写明未实现任务将返回 `ERR,UNSUPPORTED_JOB`，不能仅因为 STM32 解析器允许发送就算功能接通。

### 2.2 数字环单步闭环

~~~text
STM32             OpenMV
  |--- ALIGN,2,7 -->|
  |<-- ACK,ALIGN,2,7
  |<-- POS,RING,2,12,-5,172,115,7,0
  |  (通过 J 标定矩阵计算小步并停稳)
  |--- STEP_DONE,0,1 -->|
  |<-- ACK,STEP_DONE,0
  |<-- POS,RING,2,...,7,1
  |        ...重复直到连续稳定...
  |--- ARRIVED,RING,2,dx,dy,7,count -->|
  |<-- ACK,ARRIVED,7 + IDLE
~~~

`target`、`session`、`step` 都必须匹配。像素图原点在左上，`cx` 右增、`cy` 下增，`dx=cx−160`、`dy=cy−120`；该误差**不是**电机的毫米坐标。`app_ring_align.c` 负责数值反算、停稳、限幅与超时，`vision.c` 仅保存合法观测和会话状态。

### 2.3 V4 已知安全与联调陷阱

1. `AREA_OK` 与 `IDLE` 是两条帧。收到第一个就马上发 `JOB,41` 可能因相机仍 busy 被拒绝。以收到 `IDLE` 后再启动 41 为准，不靠随意 `sleep`。
2. `JOB,41/42` 依赖真实抓取目标/流程条件，不能把标定值不足或未完成动作授权当成已到位。
3. `PICK_READY`/`PICK_OK`/`PICK_FAIL` 等抓取回执在历史相机实现/固件版本中的支持程度不完全一致，需逐项核对双端，否则只能保证“视觉到位”，不能保证“实物夹取完成”。
4. UART 帧分片、恶意错误帧、XOR、跨 session 的旧观测、`STEP_DONE` 重放及 `IDLE` 释放都必须测，不能用一次理想串口消息代替。

## 3. OpenMV 旧 V5（不是 V4 报文的别名）

V5 旧路径（四文件相机显式设置 `UART_PROTOCOL='V5'` 才选用）使用换行分隔的 `#` 消息，例如 `#SEQ` 扫码，`#SLOT` 盘位，`#ARM` 授权，`#OBJ` 物料准备，`#TAKE`、`K` 完成握手。后续 4～25 步使用：

~~~text
#AT,step
#OBS,step,token,...
#USE,step,token
#RESULT,step,token,OK   (或 FAIL)
~~~

V5 的 token 与 V4 的 `session/step` 不能直接互换。正式主控保留部分旧 `R`/`#SEQ` 兼容，不代表拥有整个 V5 抓放协议的执行闭环。

## 4. STM32→树莓派 ULog43

主控 USART2 PD5 TX 输出 115200 8N1 的 **43 列 ASCII CSV** 遥测（无标题，CR LF 结束），`algorithm/car_ws/src/car_stm32_bridge` 只读解析器转换成 ROS2 数据。典型字段按零起始列号：`ch[5]` STM32 ms 时间，`ch[14/15]` 目标/实测角速度 rad/s，`ch[16/17]` 底盘/IMU 航向 °，`ch[28..31]` 四轮编码器脉冲/s，`ch[32/33/34]` x、y、累计路程 m。

桥接可能发布 `/odom`、`/car/chassis/telemetry`、`/car/chassis/wheels`、`/car/mission/state`、`/car/debug/ulog43`。这个**只读链**不实现从 ROS2 `/cmd_vel` 向 STM32 下发速度。格式错、缺列、NaN/Inf、时间倒退和断联须报错/离线，不能让网页继续假装有最新真值。

## 5. TEST CAR-LINK V1 / PTY

此链与 ULog43 不同：

~~~text
TEST Mock MCU
  → Linux PTY (/tmp/...，虚拟串口)
  → CAR-LINK V1（binary + CRC16）
  → car_serial_bridge
  → ROS2 /odom + /car/* telemetry
  → ros2_visualization_bridge / test_telemetry_bridge
  → /api/test/* → 浏览器 TEST
~~~

TEST 额外能模拟任务 ACK/PROGRESS/COMPLETED、CRC 错误、链接断线和 STM32 C FSM；其中完成反馈是**模拟器**产生，不能拿来证明真的插线、抓取或有双向实车控制。默认测试域与实车 ROS_DOMAIN_ID 分离（示例 `73`）。详见 [TEST 验收](../sim2real/TEST_SUITE.md)。

## 6. 键盘候选路径与网页接口

~~~text
WASDQE /key events (浏览器)
  → localhost GET/POST /api/keyboard/intent
  → keyboard_ros2_pc.py: TwistStamped
  → /car/teleop/local_test_twist
  → keyboard_ros2_web_feedback.py: 独立订阅
  → POST /api/keyboard/ros2_feedback
  → KeyboardChassisTwin(mode='direct')
  → /api/keyboard/dryrun → 紫色 MOCK 车
~~~

**这个链有真实的软件 ROS2 发布和订阅，但没有实际 STM32 电机/机械命令。** `hardware_authorized=false`、`stm32_tx_enabled=false` 等约束不可删除；`TTL_S=0.30` 秒过期停令。`/api/keyboard/command` 明确返回 403。Pi 候选 `/car/teleop/desired_twist` 和 `/car/teleop/stm32_candidate` 也不能被解释为 STM32 执行回执。

`keyboard_feedback_gateway.py` 另外维护只读 `/cmd_vel` 观察/反馈展示，用于诊断 ROS2 速度指令，与控制电机是两回事。同步失效/乱序时网页应区别“浏览器捕获键”“HTTP intent”“ROS2 echo”“MOCK 位姿”“REAL /odom”五个环节。

## 7. 真正合并前需要的“跨板接口合同”

至少用同一张表双方签字确认：

| 字段 | 必填问题 |
| --- | --- |
| 帧层 | UART 号、引脚、电平、baud、校验、终止符、最大长度、超时 |
| 身份 | source_type、任务号、session、step/cmd_id、时间戳、重试/重放策略 |
| 物理量 | 单位、坐标原点、正方向、frame_id、有效期、量程、无效值 |
| 控制 | 谁拥有控制权、谁能急停、什么时候允许动作、互斥和掉线策略 |
| 反馈 | ACK/执行完成/失败/停止/位置实测分别如何表示 |
| 版本 | `V4` / `V5` / `CAR-LINK V1` / `ULog43` 明确区分，不混封包 |
| 安全 | 故障默认停止、限位、无授权拒绝、未校准禁止自动对位、重复命令幂等 |

建议最先验证 **最小跨板闭环**：`READY`→`JOB,1`→QR→`ACK`→`IDLE`，以及 `ALIGN`/`STEP_DONE` 单步验证。每个阶段录十六进制字节流、解析后的字段和硬件实际状态。
