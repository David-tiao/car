# OpenMV H7 Plus → STM32F4 电控 V4 对齐（第五阶段，第一批）

## 上游基线（不可混用）

- 电控仓库：`Wangdachang888/GongChuangSai`
- 电控分支：`refactor/architecture-v2`
- 核对的电控 HEAD：`ffbbb393c9eb209914230ed2153ea014359c7b9d`
- 视觉正式三文件：`vision_openmv_stm32_v4_redownload/OpenMV/{main.py,config.py,utils.py}`
- 主控协议实现：`Hardware/vision.c`、`Hardware/vision.h`；主控 USART3
- 同步目标：`algorithm/openmv/openmv_v4/` 和 `STM32F4/vision_openmv_stm32_v4_redownload/OpenMV/`

本次采用电控仓库该提交的 **V4 相机三文件原样基线**。独立 V5 四文件
`algorithm/openmv/openmv_all/` **完全保留，未修改，不能与 V4 混烧**。

### 代码变化

1. 新增 `algorithm/openmv/openmv_v4/{main.py,config.py,utils.py}`，与上游电控分支上述三个文件逐字节相同。
2. 更新原本较旧的 `STM32F4/vision_openmv_stm32_v4_redownload/OpenMV/main.py` 和 `config.py`；其 `utils.py` 原本就与电控上游相同，保持不动。
3. 新增主机端 V4 契约测试，覆盖完整 ASCII 帧 XOR、LEN、SEQ 防重放、分包、二维码会话和常用 JOB 编号。

**本次不修改** `STM32F4/Hardware/vision.c`、`vision.h`、电机 PID、引脚、USART 资源和真实通信桥接。

## 关键差异：V4 不是 V5

| 条目 | V5（保留作为视觉实验版本） | 电控 V4 正式目标 |
| --- | --- | --- |
| 帧 | `#SEQ` / `#AT` / `#OBS` / `#USE` / `#RESULT` | `@SEQ,LEN,BODY*XX\n` |
| 帧校验 | 换行分隔，无 V4 XOR | BODY 长度 + ASCII XOR + 序号防重复 |
| 扫码 | `R` 和 `#SEQ` | `JOB,1,session` / `QR,code,session` / `ACK,QR,session` |
| 数字环 | 使用 V5 步骤消息 | `ALIGN,target,session` → `POS,RING,...,session,step` → `STEP_DONE` → `ARRIVED` |
| 电控角色 | 相机自己推进部分动作步骤 | STM32 决定任务开始和动作执行；OpenMV 只返回视觉观测 |

重要：上游 STM32 扫码当前还会先发 **单字节 R**，随后才发 V4 `JOB,1,session`。
V4 相机由 `JOB` 启动扫码，忽略旧式裸 `R`；不得因此去掉 STM32 对 V4 `JOB` 的发送。

## V4 帧和 QR 的精确约定

```text
@SEQ,LEN,BODY*XX\n
```

- `SEQ`：十进制 0～65535，双向独立计数；重复或旧序号应丢弃。
- `LEN`：BODY 的 ASCII 字节数，1～128。
- `XX`：对 `SEQ,LEN,BODY` 所有 ASCII 字节依次异或，输出两位十六进制。
- 例如 `@1,9,ALIGN,2,7*40\n` 是合法完整帧；修改最后校验值则被拒绝。
- QR 会话：`JOB,1,23` → `ACK,JOB,1` → `QR,156+123+516+231,23` → `ACK,QR,23` → `DONE,QR` → `IDLE`。
- 上游电控在 `qr_session != 0` 时**不接受无 session 的旧 QR 消息**。
- 扫码由 STM32 的扫描路段超时/取消控制；参考 V4 相机的 `JOB_QR_TIMEOUT_MS=None`。

全部数据只是 BODY，UART 上必须经过 V4 包装。

## 当前必须知道的硬件版本限制

**仓库内 `STM32F4/Hardware/vision.c` 的历史拷贝比上游目标分支旧**；
它尚未实现目标上游的完整 QR 会话处理。这里没有擅自把电控固件整体替换成
上游版本，因为对应 C 文件、头文件和应用调用链需要由电控负责人核对
并完成编译。要实现带会话号的真实扫码，**实际烧录到 STM32 的固件必须
与上游 `refactor/architecture-v2` 的 V4 接口一致**，不能只更新相机文件。

其次，参考电控 `Hardware/vision.c` 目前仍未完整解析 `PICK_READY`；
相机可在 `JOB,42` 成功后发送该信息，但自动抓取回执链的另一端尚未打通。
本次没有用假 `ACK` 或旧 `K` 伪装已完成抓取。

参考 `Config/race_config.h` 的圆环纠偏标定矩阵目前仍为 **0**；
在真实标定前圆环自动微移无法安全启用。
`config.py` 的 `GRIP_TARGET_CALIBRATED=False`、距离/焦距标定开关保持未授权。
因此目前只准进行无运动风险的 UART/扫码演示，不应据此触发抓手或底盘运动。

## Host 端检查（Ubuntu，无需 OpenMV 与 STM32）

```bash
cd ~/Desktop/car
bash algorithm/openmv/bash/test_v4_alignment.sh
```

该脚本用标准 Python `ast` 加载纯算法和 V4 串口解析类，给 UART 注入模拟缓冲。
**不**依赖 `csi`、`machine`、真实串口或 ROS 2。测试通过仅代表软件协议契约初步一致。
不等于 OpenMV Firmware 5.x 兼容性、Keil 编译或电控实机测试通过。

## 真正烧录前

1. 将 `algorithm/openmv/openmv_v4/` 目录下三个文件直接放在 OpenMV Flash 根目录；先备份旧 `main.py`，不要把 V5 `serial.py` 或 `tools.py` 混入。
2. OpenMV UART3 / P4 TX → STM32 USART3 RX（PD9），OpenMV P5 RX ← STM32 USART3 TX（PD8），**共地、115200 8N1、电平适配确认**。
3. 在机械输出禁用、电机断电情况下，看相机 `READY,IDLE` 是否正确入帧，再请求 `JOB,1,session`；确认合法扫码、会话号匹配、旧 QR 丢弃、断线后安全退出。
4. 待电控负责人确认真实固件 V4 会话支持后，再做 `JOB,40` 区域确认、`JOB,41` 夹爪像素标定及 `ALIGN` 数字环。未经现场测量保持底盘运动禁用。
5. 同步电控 `PICK_READY/PICK_OK/PICK_FAIL` 业务状态机后再做真实抓放闭环。

## 测试状态

- 上游文件内容：已从指定 GitHub 分支逐个读取和同步。
- Host 端测试：**NOT_RUN**（等 Ubuntu 输出）。
- OpenMV IDE/相机/串口/STM32 编译/实机：**NOT_RUN**。
- 上游的视觉图像算法为传统方法：LAB 分割、盘面几何/卡尔曼跟踪、
  近似同心黑环径向检测、字符模板及数字投票、二维码连续帧确认。
  本次未向它加入新目标检测模型，避免在通信对齐前变更算法行为。
