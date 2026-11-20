# 第五阶段 B：STM32F4 × OpenMV V4 扫码会话与 TEST 网页状态机实验

## 目标 / 边界

这次解决的是 **STM32 仓库历史版与已同步的 OpenMV V4 之间的 QR session / ACK 兼容问题**。
参考电控仓库：`Wangdachang888/GongChuangSai@refactor/architecture-v2`；
本次只引入会话相关的小范围修改，没有整体替换电控固件。

**状态严格区分：**

- **固件实际源码**：`STM32F4/Hardware/vision.c/.h` 与 `App/app_race.c` 接受 V4 QR session 并回发 ACK。
- **原生 C 测试**：Ubuntu GCC 编译实际 `Hardware/vision.c` 等 C 文件进行解析与状态回归。
- **TEST 网页实验**：`sim2real/test_v4_qr_lab.py` 是独立的纯 Python V4 协议契约模型，逐帧显示扫码和故障结果；**不是编译并运行 STM32 固件**。
- **现有网页 TEST 任务演示**：`/api/test/mission_replay` 继续是 CAR-LINK PTY 模拟任务，与 V4 QR 测试互相独立。
- **实车**：此次没有启用真实电机控制、USART 改线、树莓派命令发送、环位纠偏或夹爪动作。

## 具体修改

1. `vision.h` 引入 `qr_session` 和 V4 `Vision_StartQrJob()`、`RaceVision_BeginScanSession()`、`RaceVision_AckQr()`、`RaceVision_ScanFailed()`。
2. `vision.c` 对 OpenMV `QR,code,session` 核对当前 QR JOB、当前运行状态、会话号、任务码合法性。带会话的扫码拒绝无 session 的旧 `QR` 和旧 `#SEQ`。保留显式 session=0 的旧通讯兼容路径。
3. `app_race.c` 用比赛 `scan_session` 发起相机扫描；只有 `Race_SubmitQr()` 成功后回传 `ACK,QR,session`，失败则进入 `FAULT`。
4. 新增 C 主机回归：用真实 `Hardware/vision.c` 检验错帧、重复 SEQ、旧二维码会话、合法任务码、重启新会话、ACK。
5. `sim2real/web/test_v4_qr_lab.js`：网页 TEST-only 交互式二维码状态机，手动逐帧查看 V4 线路帧与状态；`/api/test/v4_qr_lab` 不接受任意命令，仅接受固定故障场景，拒绝远端发起和跨域调用，不触及 `/api/real`。

## 一键自动测试（Ubuntu，仅需 Python 3 + GCC）

```bash
cd ~/Desktop/car
bash algorithm/openmv/bash/test_v4_stm32_web_lab.sh
```

脚本依次执行：
- V4 OpenMV Python 协议契约测试；
- TEST 网页实验的 8 个故障/正常案例、HTTP 安全限制测试；
- 四组真实 STM32 C 源码的 GCC 主机测试：`test_race`、`test_uart`、`test_ring`、`test_app`。

不需 ROS2、STM32 开发板、OpenMV 或真实串口。若编译/断言失败，会立即以非零码退出，失败前不能将 PR 视为已验收。

## TEST 网页实验（无需 ROS 2）

```bash
cd ~/Desktop/car
python3 sim2real/server.py --host 127.0.0.1 --port 8080
```

浏览器访问 `http://127.0.0.1:8080/test`，下滑找到：

**TEST · OpenMV V4 ↔ STM32 二维码协议状态机**

选择 `正常扫码`、`重复帧`、`旧 session`、`坏 XOR`、`错误任务码`、`JOB ACK 丢失`、`二维码超时` 、`QR ACK 丢失` 或 `CANCEL`，点击「运行协议实验」，再点击「下一帧」查看每一帧的 V4 BODY、原始 ASCII 帧、状态和接收判定。

也可通过 shell 直接请求测试 API：

```bash
curl -s -H 'Content-Type: application/json' \
  -d '{"scenario":"success"}' \
  http://127.0.0.1:8080/api/test/v4_qr_lab
```

**预期状态：** success 与 duplicate_seq 最终 `DONE`，重复序号及丢失二维码 ACK 后重发均不能多接收二维码；旧 session、坏 XOR、错误任务码、无 ACK、无 QR 进入 `FAULT`；主动 CANCEL 是 `CANCELLED`。这里的超时属于**模拟故障注入**，不表示网页在真实时间等待八秒。

## 非本轮验收范围

- 真实 Keil 工程编译与 STM32F407 烧录。
- 真实 UART3 OpenMV 收发，OpenMV Firmware 5.x 兼容性和 LCD 显示。
- `RACE_PICK_DIRECT_KEY_TEST=1` 目前仍为夹取按键测试，**本轮未解除**。
- 原料区 JOB40 需要三色盘面布局，JOB41/42 的 `GRIP_TARGET_CALIBRATED=False` 保持；未完成机械标定。
- 圆环微调标定矩阵仍然全零，不应进入实车自动微移。
- ROS2 → STM32 正式 CAR-LINK 控制仍然只读，不代表这次已经完成真实移动控制。

通过该实验仅说明二维码会话的**双端软件契约已一致、主机测试未发现问题**；不意味着比赛完整任务状态机已经接上或已通过实机。
