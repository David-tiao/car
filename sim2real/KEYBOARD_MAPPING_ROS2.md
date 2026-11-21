# 映射集当前行为：键盘直通（2026-11-20）

## 车载机械结构 MOCK 操作台（2026-11-21，待浏览器验收）

**平滑动画与多模式统一（第二轮）**：以 `keyboard_mechanism_preview.js` 中的 `AXES` 为单一网页机构状态，`computeMechanismPose()` 计算统一姿态，`buildTopRig()` 构造一次规范化机械俯视轮廓。`/keyboard` 的详细俯视图、紫色 MOCK 小车、`/sim`、`/test`、`/real` 的小车轮廓都复用该生成器。大图侧视投影共用姿态值，不能用顶视平面旋转伪装 PB15 的真实翻转方向。TEST/REAL 的轮廓为**固定零位结构示意（UNMEASURED_SCHEMATIC_ONLY）**，不根据不存在的机构遥测猜测转盘角度、升降高度或夹爪开合。

**速度参数区分**：`firmwareRate` 仅记录固件名义速度（PC7 60°/s、PB15 127.764°/s、PB14 720°/s）；`previewRate` 与 `previewAccel` 仅决定浏览器动画。网页使用最大 1/120 秒的小步积分、根据制动距离减速、支持中途反向与 150ms 的长帧保护。升降网页演示速度不能代表未校准的步进电机实际 mm/s。转盘预设显示绝对角度 0/90/180/270，不把固定角度错误标记成实际轮次工位。无 ROS2/HTTP/UART 输出。


打开 `/keyboard` 可在现有 ROS2 速度回环和紫色 MOCK 小车旁看到仓库机械结构的俯视、侧视示意：左前方的升降导轨与旋转夹爪、车身侧部的三工位物料托盘、四个麦克纳姆轮。可调整以下纯浏览器演示目标：

- **PC7 / SERVO_3 车载三工位转盘**：0–270°，预设 0、90、180、270°；按 `Config/race_config.h` 的名义速度 60°/s 做网页过渡。
- **PC6 / PC8 / PC9 升降轴**：0–175 mm，提供 0、24、80、141、170、175 mm 预设。网页用 65 mm/s 的**演示速度**；其位置不是通过步进脉冲实测的真实高度，也没有限位寻零。
- **PB15 / SERVO_2 夹持机构旋转**：0–180°，使用固件的 127.764°/s 名义速度；
- **PB14 / SERVO_1 夹爪开合**：0° 打开、110° 预夹、140° 夹紧；按固件名义 720°/s 演示开合。

四组控制由 `sim2real/web/keyboard_mechanism_preview.js` 在浏览器内保存目标并动画绘制，**没有任何 fetch/ROS2/UART/电机发送**。旧版底盘 W/A/S/D/Q/E 通信和真实 `/odom` 接口没有修改。页面显示的是模拟机构姿态，**绝不能当作电机已动作、实测机构反馈或机械限位合格**。仓库的 `/car/mechanism/state` 有效位仍为独立硬件后续对接事项。

物理造型参考：`Hardware_mechanical/mechanical/俯视图.png`、`前视图.png`、`侧视图.png`；实际针脚与角度参考：`STM32F4/App/app_pick_motion.c`、`STM32F4/Config/race_config.h`。验收命令为 `node sim2real/tests/test_keyboard_mechanism_preview.js` 和 `bash algorithm/car_ws/bash/test_all_offline.sh`。本次仅更改 GitHub 待验收分支，不创建任何 Ubuntu 桌面目录。


**动画流畅性优化：** 本机 ROS2 的位置快照独立以约 75ms 更新，紫色 MOCK 车通过浏览器 `requestAnimationFrame` 按屏幕刷新率绘制（通常 60Hz）。帧间只按**最后一次实际订阅的速度消息**最多外推 120ms，以消除 250ms 一跳的视觉问题；超出这个窗口不再继续前进，刷新/松键/重置也不伪造 ROS2 消息。数据卡片和误差统计约每 300ms 更新。键盘长按不会因系统键盘重复事件重复发送请求；组合键变更可在上一 HTTP 请求结束后立刻补发最新状态。**这只是网页渲染优化，不是实体运动实测或新的电机控制通道。**

**网页自动启动 ROS2（新增）：** 在 Ubuntu 本机启动网站后，打开 `/keyboard` 将自动启动现有的 PC ROS2 发布与 Web 订阅回传两个节点；不需要第二个终端运行 `start_keyboard_local_ros2.sh`。保留原有 WASDQE 按键、限速及消息格式，本机话题依旧是 `/car/teleop/local_test_twist` (`TwistStamped`)。运行环境需有 `/opt/ros/humble/setup.bash`；如果自动启动失败，查看 `/api/keyboard/autostart` 和 `sim2real/data/web_modes/keyboard_local_autostart.log`。设置 `CAR_KEYBOARD_AUTOSTART=0` 可关闭自动启动并继续使用下文的手动方式。网站退出时清理自动启动的两个节点，不会启动 Pi 或 STM32 实体驱动。

**本机模式已改为最简单的电脑 ROS 2 → 网页反馈，不使用任务 FSM 或场地约束。**
该段为最新实现说明；后续文档里提到的 Shift 使能、碰撞停车、加速模型、
软件故障注入属于此前版本/独立测试实验室，不再适用于默认的 `/keyboard`。

```text
W/S = 车体前进/后退             /car/teleop/local_test_twist
A/D = 车体左移/右移  网页按键 ──→ PC ROS2 发布 ──→ PC ROS2 订阅
Q/E = 车体左转/右转                   │
松开=发 0，空格/Esc=清零             ▼
                                 localhost HTTP 回传
                                      │
                                      ▼
                                紫色 MOCK 车直接积分 vx/vy/ω
```

1. **直接按 W/A/S/D/Q/E**，无需按 Shift，也无需勾选启用按钮；组合键照常工作。
2. 速度大小来自平移/角速度滑块，收到的 ROS2 速度立即生效，
   **没有加减速斜坡**，不需要任务开始、到达目标、扫码或机械状态反馈。
3. 红色障碍格、黄色禁行区、场地边界只作为图形背景；
   **不阻止虚拟车运动、不触发碰撞停车、不自动复位**。虚拟坐标离开
   2.4 米场地时可以超出地图可视区，按「手动回到起点」才重置网页预览。
4. 本机页面不显示软件电机卡滞/迟缓故障控制、复杂的实车 PID 误差指标；
   显示按键期望速度、实际 ROS2 订阅速度、消息差值/延迟和纯数学积分位姿。
5. **松键立即发送零指令**；网页关闭/切走焦点发 STOP；
   300ms 未刷新自动归零仅为通信断开保护，不是任务状态机。
6. 未有树莓派时未来链路处于预留状态。该链路原先的安全使能、
   ROS2 候选速度限制和 STM32 UART 禁止规则不做任何放宽。
7. 真实车辆的 `/odom` 仍然为**不可比较**；MOCK 直通积分不是实际里程计。

运行：

```bash
cd ~/Desktop/car
git switch feat/stm32-pty-fsm-realview-20261120
git pull --ff-only
bash algorithm/car_ws/bash/test_keyboard_mapping.sh

# 终端1：网页
python3 sim2real/server.py --host 127.0.0.1 --port 8090

# 终端2：本机 ROS 2 发布 + 订阅
export ROS_DOMAIN_ID=42
bash algorithm/car_ws/bash/start_keyboard_local_ros2.sh http://127.0.0.1:8090
```

访问 `http://127.0.0.1:8090/keyboard`，Ctrl+Shift+R 刷新后直接按 W，
或按住界面 W 按钮。无实车、不接 STM32，确认紫色 MOCK 车响应和速度。
若需要测试旧的运动约束/故障注入代码，用
`python3 sim2real/keyboard_fault_lab.py` 单独运行，不再让其干预映射集。

---

# 新增：映射集双链路切换（目前没有树莓派，优先使用本机模式）

现在另有 **键盘 ROS2 回环驱动的纯软件麦轮底盘**，在 /keyboard
地图显示紫色 MOCK 车、障碍碰撞停车、加减速、模型误差和三种
故障注入，并支持「运行 7 项离线验收」。该模型**从不覆盖真实 /odom**。
详细检查方法：[`KEYBOARD_OFFLINE_TWIN_QA.md`](KEYBOARD_OFFLINE_TWIN_QA.md)。

`/keyboard` 顶部新增两个可切换模式，切换时会清空原键盘速度指令，避免上一模式残留控制请求：

| 模式 | 现阶段用途 | 收到什么反馈 | 能否得出真实底盘误差 |
| --- | --- | --- | --- |
| **本机 ROS 2 回环（默认）** | 现在只有一台 Ubuntu 电脑也能验证完整的 ROS 2 发布→订阅→HTTP→网页闭环 | 本机订阅 `/car/teleop/local_test_twist` 的实际 ROS 2 消息，显示收发速度差及估算话题延迟 | **不能**；这只是电脑 ROS 2 收到指令，不是实际小车运动 |
| **电脑→树莓派→STM32** | 预留接入后端硬件，检查是否收到 Pi 候选话题与真实 `/odom` | Pi `/car/teleop/stm32_candidate`（仅候选）＋独立 REAL `/odom`（必须来自真实 STM32） | 仅真实 /odom 有效且坐标系正确时才能比较；当前无 Pi 应明确显示离线 |

## 现在马上就能做：一台电脑完成本机 ROS 2 回环

终端 1：启动网页：

```bash
cd ~/Desktop/car
python3 sim2real/server.py --host 127.0.0.1 --port 8090
```

终端 2：**一次启动两个互相独立的 ROS 2 节点**。一个从键盘 HTTP 接口发布 ROS 2，另一个真正订阅 ROS 2 再回传网页（不是 Python 直接把键盘值写回网页）：

```bash
cd ~/Desktop/car
export ROS_DOMAIN_ID=42
bash algorithm/car_ws/bash/start_keyboard_local_ros2.sh http://127.0.0.1:8090
```

然后打开 `http://127.0.0.1:8090/keyboard`，选择蓝色的
**「本机 ROS2 回环 · 现在可用」**，勾选键盘使能，再按住
**Shift+W**。映射集会分别显示：键盘期望速度、实际在 ROS 2 话题上
被订阅到的速度、收发差值、估算消息延迟；松开 Shift 应在 300ms
内显示候选零速度。**不需要树莓派，不需要 STM32，也不会让电机运动。**

本机方式明确隔离两个话题：

```text
[键盘 /keyboard] --localhost--> [电脑键盘发布节点]
                               |
                               +--> 只在本机模式发布：
                                    /car/teleop/local_test_twist
                                      |
                                      v
                                  [电脑独立 ROS2 订阅节点]
                                      |
                                      +--> localhost POST
                                           /api/keyboard/ros2_feedback
                                                |
                                                v
                                            [网页 ROS2 接收状态/差值]
```

**误差名词特别注意：**
“ROS2 命令回环差值 = 期望键盘速度 − 本机订阅到的速度”只检验
消息内容一致性，不是运动误差；“本机延迟”按 ROS 时间戳估算，
不包含真正电机执行或 /odom 回传延迟。
“实际车辆速度/位置、REAL /odom 误差”在只有电脑时为**不可比较**。

要查看真实 ROS 2 消息，另开终端：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic echo /car/teleop/local_test_twist
```

## 将来树莓派接入后再选完整链路

切换到 **「电脑→树莓派→STM32 · 预留」** 后，电脑发布节点改为
只向 `/car/teleop/desired_twist` 发布候选指令（原本机话题停止）；
Pi 侧接收并以看门狗约束，发布 `/car/teleop/stm32_candidate`。
电脑上的独立订阅节点可接收这条 Pi 候选话题并显示到达状态。

注意：**Pi 候选回执也不证明 STM32 执行了动作**。
真正的底盘反馈必须是 STM32 编码器/IMU 经已验证的固件通信，
由 ROS2 `/odom` 送回 REAL 频道。没有这些数据，页面保持“没有实车反馈”。

仍使用同一电脑端一键脚本；树莓派上额外运行
`start_keyboard_pi_ros2.sh`，具体部署见下文。禁止直接将候选话题
映射为 `/cmd_vel` 或未验证的 STM32 速度控制帧。

---

# 映射集：电脑 ROS 2 → 树莓派 5 ROS 2 → STM32F4（安全开发基线）

## 当前完成程度

`http://127.0.0.1:8090/keyboard` 现在是可交互的**键盘映射与跟踪误差实验室**，不是空白占位页面。
浏览器与电脑 ROS 2 节点运行在**同一台 Ubuntu 电脑**，树莓派 5 运行 ROS 2 Humble：

```text
[电脑浏览器 /keyboard，按住 Shift + WASDQE]
                 |  localhost HTTP /api/keyboard/intent
                 v
[电脑 ROS2 keyboard_ros2_pc.py]
                 |  /car/teleop/desired_twist  geometry_msgs/TwistStamped
                 |  ROS_DOMAIN_ID 相同、双机 DDS 发现
                 v
[树莓派5 ROS2 keyboard_ros2_pi.py]
                 |  时间戳检查 / base_link / 速度限制 / 300ms 看门狗
                 v
[树莓派 ROS2 /car/teleop/stm32_candidate]
                 |  尚未实现、未授权 CAR-LINK 实体控制 TX
                 X
[STM32F4 真实运动接口：需要电控队友确定并实现]

[STM32 实际编码器/IMU → 串口上报 → 树莓派 ROS2 /odom]
                 |  ROS2 DDS 数据返回电脑（若网络/域已打通）
                 v
[电脑 ros2_visualization_bridge.py --web-channel real]
                 |  localhost HTTP /api/real/telemetry
                 v
[/keyboard 实测速度、残余停车速度与误差 RMS]
```

**本阶段不会驱动实体电机、不会发送实际 `/cmd_vel` 或 UART 帧。**
`/car/teleop/stm32_candidate` 只是一个未接物理设备的候选接口，不意味着
STM32 已接受指令。映射集主要用于先调试键位、电脑/树莓派 ROS2 网络、
消息延迟、里程计反馈与数据误差计算。其用途是赛前安全调试而非比赛时遥控。

## 键盘与车辆动作映射

使用 ROS REP-103 的 `base_link` 平面坐标：
- **W/S**：车体前进/后退（`linear.x` 正/负）。
- **A/D**：车体左移/右移（`linear.y` 正/负；麦轮可以横移）。
- **Q/E**：左转/右转（`angular.z` 正/负）。
- **Shift**：需要持续按住的使能键（deadman）。松开立即请求零速度。
- **空格 / Esc / 失焦 / 切换标签页**：停止并归零；服务端 300ms 无新指令也归零。
- **W+A** 等斜向组合会把平移速度归一化，避免超过滑块设定上限。
- **G 夹爪、R 伸缩**：暂只显示接口预留，**没有**真实机械动作请求，
  因为尚无对齐的 STM32 机械控制权、限位和完成反馈协议。

默认线速度 0.12 m/s，允许滑块范围 0.03–0.25 m/s；角速度默认
0.30 rad/s，允许滑块范围 0.05–0.50 rad/s。这只是**软件候选限幅**，
不能代替真实电控的电机输出限制、碰撞防护、现场人员安全和急停。

HTTP 请求强制 localhost + 同源；每个意图带单调递增 `client_seq`，
重置/松开后的迟到旧请求不能恢复运动。电脑端每 50ms 发布候选速度，
树莓派再进行独立限幅/ROS 时钟校验与 300ms 看门狗，超时归零。
真正连接 STM32 前须另行完成物理硬急停、独占控制权、上电禁止自动运动、
串口 ACK/执行反馈/控制超时及 UART TX 断电测试。

## 误差面板采用哪些数据？

- 期望速度 `(vx_d,vy_d,wz_d)`：根据键位和速度滑块生成的 **base_link**
  目标速度，**不是已下发 STM32 的实测目标**。
- 实际速度 `(vx_a,vy_a,wz_a)`：来自 `/api/real/telemetry` 中
  **REAL:ros2_odom** 且最新数据未超时的 ROS2 `nav_msgs/Odometry`。
- **严格核对**位姿 `header.frame_id=odom` 和速度 `child_frame_id=base_link`。
  若未连接、数据过时、处于 TEST/MOCK、坐标系不对或到来的数据比按键更旧，
  显示“不可比较”，不填造假的误差数字。

```text
Δvx = vx_d - vx_a
Δvy = vy_d - vy_a
Δω  = wz_d - wz_a
速度向量误差 e = sqrt(Δvx² + Δvy²)
RMS = sqrt(平均(e²))   // 只使用新增的有效 odom 样本，重复轮询不计数
```

当前网页以约 4Hz 拉取最新里程计进行比较，可能跳过 15Hz 或更快的
中间采样；该 RMS 是**网页采样 RMS**，不应冒充 STM32 控制周期的
高频跟踪误差。如需高频 PID/电机级测试，应在树莓派 ROS2 直接记录
每帧 `/odom`、`/car/chassis/wheels` 和底盘控制目标到 rosbag。

相对位置偏差是在同一次映射会话内，从首次有效里程计位置开始，
对**期望速度**进行积分，然后与随后里程计的 `x/y/yaw` 比较。
这是“指令积分参考 ↔ odom”的**相对轨迹偏差**，不是绝对定位准确度。
无真值定位时不可用其评价 SLAM / 地图坐标误差。
停车后会继续观察约 1.2 秒的**实际残余滑行**，便于测停车响应。
可通过“导出误差 CSV”保存每次有效样本。
误差曲线同一时间轴包含 `Δvx`、`Δvy`、`Δω`；
角速度单位不同，需要分别查看数值，不应将其直接当作距离误差。

## Ubuntu 电脑端运行

1. 先从最新开发分支同步代码并运行离线验收：

```bash
cd ~/Desktop/car
bash algorithm/car_ws/bash/test_keyboard_mapping.sh
```

2. 在电脑终端启动网站（**本机 localhost**）：

```bash
cd ~/Desktop/car
python3 sim2real/server.py --host 127.0.0.1 --port 8090
```

打开 `http://127.0.0.1:8090/keyboard`。

3. 电脑终端启动 ROS 2 命令候选发布器：

```bash
cd ~/Desktop/car
export ROS_DOMAIN_ID=42
bash algorithm/car_ws/bash/start_keyboard_pc_ros2.sh --server http://127.0.0.1:8090
```

4. 新开一个电脑终端检查发出的 ROS 2 话题：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic echo /car/teleop/desired_twist
```

先在网页选“本机启用键盘 ROS2 候选指令”，按住 Shift + W。
echo 中的 `linear.x` 应为 0.12，松 Shift 后变成 0；没有网页更新
超过 300ms 也应保持 0。

## 树莓派 5 端运行

树莓派需克隆同一版本的仓库、安装 ROS 2 Humble，并与电脑在同一
LAN，确认 ROS 2 DDS 可以发现彼此且 `ROS_DOMAIN_ID=42` 一致。
两台机器时钟要同步（chrony/NTP），否则 Pi 的时间戳检查将拒绝旧消息。
检查 Wi-Fi AP 是否开启客户端隔离、UDP Multicast 防火墙策略、
`ROS_LOCALHOST_ONLY` 不应为 1。

```bash
cd ~/Desktop/car
export ROS_DOMAIN_ID=42
bash algorithm/car_ws/bash/start_keyboard_pi_ros2.sh
```

在树莓派另一个终端：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic echo /car/teleop/stm32_candidate
ros2 topic echo /car/teleop/pi_guard_status
```

**成功标准：** 树莓派能看见候选消息，键位松开/电脑掉线后零速度，
`pi_guard_status` 明确输出 `uart_tx_enabled:false`。
不能将这些反馈解释为实体 STM32 正在运动。

## 实车里程计的只读展示

如果树莓派通过原有、**已确认匹配烧录固件**的 CAR-LINK 或 ULog43
读串口桥接发布了真实 `/odom`，且该话题经 ROS2 网络传到电脑，
在电脑额外终端启动：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
cd ~/Desktop/car
/usr/bin/python3 sim2real/ros2_visualization_bridge.py \
  --server http://127.0.0.1:8090 --web-channel real --odom /odom
```

该桥只读回传 `/odom` 到 REAL HTTP 通道，不会发速度控制。
**电脑到树莓派的话题流通并不保证反向 /odom 已存在。** 没有真实
STM32 里程计就显示 `NO_REAL_ODOM`，不得用网页模拟/TEST 值替代。

## 后续如何接上最后的 STM32F4？

需先与电控确认：
1. STM32F4 实际烧录固件支持哪一套串口命令（原版无树莓派写入口），
   CAR-LINK V1 只是树莓派侧的拟议协议，不能直接假定电控支持。
2. 物理 UART 口/电平、波特率、控制权归属、车体坐标映射/正负方向。
3. 速度命令及 STOP 帧编码、ACK、重发/去重/超时停车、急停硬件、
   上电默认锁定、主控失联以及相机/雷达仲裁。
4. 实物转轮空载小速度闭环，且有人员监护与急停；
   配合编码器、IMU 与 /odom 的速度和符号标定。

通过验收后再增加独立的受控 CAR-LINK TX 适配器。
**不要**把 `stm32_candidate` 直接 remap 到 `/cmd_vel` 或
未验证的串口发送函数。

## 本阶段软件验收

```bash
bash algorithm/car_ws/bash/test_keyboard_mapping.sh
```

运行纯 Python WASDQE/Shift/超时/旧命令拒绝、REAL 遥测隔离、
误差/停车滑行、坐标系校验与 HTTP 跨域测试，
并重跑四模式 HTTP 兼容测试。若安装 Node.js，检查键盘脚本语法。

此功能仍是**待 Ubuntu 验收的开发 PR**。即使测试通过，
也只能说明软件接口有效，不代表实体 STM32 控制闭环完成。
