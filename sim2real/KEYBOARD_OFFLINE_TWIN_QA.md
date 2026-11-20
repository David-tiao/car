# 映射集离线开发：ROS2 回环 → 运动数字孪生 → 故障验收

> **版本说明（2026-11-20）**：本页描述此前的物理模型、障碍及故障注入实验，仅用于独立离线测试；**当前 /keyboard 默认已改为 WASDQE→本机 ROS2→网页直接积分，不运行这些模型或任务状态机**。最新操作以 [KEYBOARD_MAPPING_ROS2.md](KEYBOARD_MAPPING_ROS2.md) 首页“键盘直通”为准。


## 为什么现在就能做？

目前没有树莓派 5、没有接入 STM32F4 的可写电机协议，但已经可以在电脑上验证：

1. 键盘 Shift+WASDQE 映射、控制超时、原始车体速度方向；
2. **电脑 ROS2 发布 → 真正 ROS2 订阅 → 本地网页反馈**，确认收发链路，而不是 JS 自我回显；
3. 用真正订阅到的本机 ROS2 速度消息驱动一个**单独的 MOCK 底盘运动模型**，在已有 2.4m 场地 SVG 上播放连续位移；
4. 校验加速迟缓、模拟电机卡死、接收中断、红色障碍格碰撞和停车后的减速行为；
5. 故障验收 JSON/Markdown 及网页一键 PASS/FAIL。

## 网页中三类数据严格隔离

| 数据 | 计算/采集方式 | 适用场景 | 不能证明什么 |
| --- | --- | --- | --- |
| **键盘期望速度** | 浏览器 WASDQE + Shift | 检查映射和目标限幅 | ROS2 已收到、实车已执行 |
| **本机 ROS2 订阅回环** | 独立 ROS2 subscriber 接收 `/car/teleop/local_test_twist`，上送 localhost | 检查 ROS2 传输、消息内容、估计延迟 | 实体车速与真实闭环 PID |
| **MOCK_KINEMATIC_ONLY 紫色车** | 简单加速度有界麦轮运动模型、地图几何近似 | 预演操作方向、碰撞/失联故障场景 | 真实电机、里程计、雷达 SLAM、碰撞物理 |
| **REAL /odom** | STM32 编码器/IMU 经 Pi ROS2 传来的实测 odom | 实车装好后做速度/位姿误差 | 当前无实车时不应该出现 |

在**本机模式**下，网页真实底盘误差区域明确显示未连接，
不会用 MOCK 运动速度冒充真实 `/odom`。
在**完整链路模式**下，仍必须收到符合来源/时间戳/坐标系校验的
REAL `/odom` 才会生成实际底盘误差。

## 模拟底盘的实现边界

文件：`sim2real/keyboard_dryrun_chassis.py`。

- 位置单位 m，左下角 (0,0)，场地 2.4 × 2.4m。
- 初始位置右上启停区 (2.16,2.16)，右下启停区 (2.16,0.24)，
  **初始朝向 π rad 指向西侧**，只是软件预览假设，不代表真实车体零角度。
- 输入为 `base_link` 车体速度 vx/vy/ω，积分到世界位置。
- 线加速度上限约 0.5 m/s²，角加速度上限约 1.2 rad/s²；
  `slow_response` 故障以 28% 加速度运行。
- 以半径 0.185m 的保守圆近似 260 × 290 mm 底盘；
  只检查**两块红色 480×480 mm 占据格**、四块黄色禁止区、
  原料转盘圆和赛道边界。
- 碰撞即锁存 `OBSTACLE_STOP`，运动归零，必须点击「复位虚拟底盘」；
  本模型**不会自动绕行**，不能替代有车体膨胀的规划器或真实避障。
- 仅在本机模式、Shift 持续按住、收到 750ms 内的新鲜 ROS2 订阅回执时
  产生模拟运动；没有 ROS2、松开 Shift、浏览器超时都会归零。
- 映射集调用 `/api/keyboard/dryrun` 只读查询及固定的模型 reset/fault；
  不接入 /cmd_vel、/api/real、UART 或实体机械机构。

## 现阶段马上验收

先在 Ubuntu 电脑执行纯 Python 离线测试（此步骤**不需要 ROS2**）：

```bash
cd ~/Desktop/car
git switch feat/stm32-pty-fsm-realview-20261120
git pull --ff-only

bash algorithm/car_ws/bash/test_keyboard_mapping.sh
```

将运行原有键盘/网页兼容测试和新增的数字孪生单元测试，
然后执行 7 项故障矩阵。生成：

```text
/tmp/car_keyboard_fault_lab/acceptance.json
/tmp/car_keyboard_fault_lab/acceptance.md
```

如需重新指定输出路径：

```bash
python3 sim2real/keyboard_fault_lab.py --out-dir /tmp/my_teleop_qa
```

之后启动已有网页及电脑 ROS2 本机回环（**这一步需要 ROS2 Humble**）：

```bash
# 终端1：网页
python3 sim2real/server.py --host 127.0.0.1 --port 8090

# 终端2：电脑 ROS2 发布 + 订阅节点
export ROS_DOMAIN_ID=42
bash algorithm/car_ws/bash/start_keyboard_local_ros2.sh http://127.0.0.1:8090
```

打开 `http://127.0.0.1:8090/keyboard`，选默认本机 ROS2 回环，
启用键盘候选指令并按住 Shift+W：

- 映射集蓝色区：ROS2 本机订阅的速度应与键盘目标大致一致（考虑采样异步）；
- 地图紫色 `MOCK` 小车应连续向左运动（初始 yaw=π），
  同时展示“模拟速度”和“模型误差”；
- 不启动第二个 ROS2 订阅进程时，MOCK 小车不应因为 HTTP 按键而运动；
- 在小车前进方向放一个红色格，再前进应触发 `OBSTACLE_STOP`；
- 改变障碍布局/启停区会复位模型，旧碰撞状态不能偷偷延续；
- 切到模拟电机卡滞，期望速度非零时模型实际速度为零、误差非零；
- 切回无故障、松 Shift，约几个控制周期后模型速度归零；
- 点击「运行 7 项离线验收」在网页查看实时 PASS/FAIL 结果；
- 实车速度、实车 RMS、/odom 位姿仍为空，因为还没有接实体车。

## 七项不依赖 ROS2 的确定性回归

`sim2real/keyboard_fault_lab.py`（同样可从网页点击）：

| 检查 | 预期 |
| --- | --- |
| NORMAL_DRIVE | 连续西向位移、有限加速度 |
| MOCK_STALL | 速度为零，模拟期望/响应误差非零 |
| MOCK_SLOW_ACCELERATION | 同时刻的模拟车速小于正常加速 |
| MOCK_LINK_LOSS | 信号失效，模拟底盘输入停止 |
| MAP_CELL23_COLLISION | 红色 CELL23 前锁存障碍停车 |
| ROUTE_WATCHDOG_STOP | 收不到新鲜 ROS2 输入后归零 |
| USER_STOP_DECELERATES | 停车后按加速度限制逐渐归零 |

这些是**软件模型的自检**，不能对外宣传为实车功能完成。
下一步拿到真实 Pi/STM32 后，再对齐真实电机转速协议、里程计
车体坐标、硬件控制权和现场急停，并用真实 odom 取代模型验证。

## 本机 ROS2 节点已启动，但紫色车仍不动：按顺序检查

**注意：** `NO_REAL_ODOM` 属于正常状态，表示目前没有 STM32 实车数据；
它不能说明本机 ROS2 回环是否正常。真实 ROS2 回环看 `ros2_feedback.online`。

现在 `/keyboard` 最上方有「链路诊断」栏，对每个阻断点明确报告原因：

| reason | 代表什么 | 下一步 |
| --- | --- | --- |
| `DEADMAN_NOT_ACTIVE` | 没有有效 Shift 使能，或页面按键输入没有激活 | 勾选键盘使能并按住 Shift+W，或鼠标持续按住「按住本机前进」 |
| `NO_MOVEMENT_KEYS` | 按住使能但没有 W/A/S/D/Q/E | 再按住一个行驶键 |
| `ROS2_ECHO_OFFLINE` | HTTP 期望值有了，但本机 ROS2 回传没送到网页 | 检查两个 ROS2 进程、相同 ROS_DOMAIN_ID、话题 echo |
| `ROS2_ECHO_ZERO` | ROS2 订阅有效，但只有零速度 | 检查按键和指令 TTL、输入焦点、死手使能 |
| `MOCK_OBSTACLE_STOP` | MOCK 模型被红色障碍/黄色禁行区挡住并锁停 | 点击「复位虚拟底盘」，移开挡路障碍 |
| `MOCK_FAULT_SELECTED` | 开启了电机卡滞、迟缓或断链模拟 | 选择「无故障」 |
| `MOCK_MOVING` | 本机 ROS2 回环、mock 积分已产生位移 | 若地图仍没动，排查浏览器 SVG 图层或刷新 |
| `WAIT_MOCK_POLL` | 已有 ROS2 正向消息，但还没完成模型更新 | 检查 `/api/keyboard/dryrun` 定时刷新 |

**第一个操作：** 两个终端分别保持下面命令运行，网页更新 Git 分支后
必须先重启 Python HTTP 服务器，再刷新页面（Ctrl+Shift+R）。

```bash
# 终端1（保持运行）：
cd ~/Desktop/car
python3 sim2real/server.py --host 127.0.0.1 --port 8090

# 终端2（保持运行）：
cd ~/Desktop/car
export ROS_DOMAIN_ID=42
bash algorithm/car_ws/bash/start_keyboard_local_ros2.sh http://127.0.0.1:8090
```

网页本机模式右侧新增 **「按住本机前进 · 松开停车」**；
按住鼠标/触屏按钮即可产生本机 Shift+W 等价的非零候选指令，
松开立刻发 STOP。只能在本机 ROS2 选项使用，不能切成 Pi 模式后驱动，
也**不会发送真实电机控制帧**。

**不想排查键盘，可以用独立端到端自检：**

```bash
# 终端3：关闭或停止网页上的按键操作，避免两个输入源互相打断。
cd ~/Desktop/car
python3 sim2real/keyboard_local_smoke.py --seconds 3
```

该命令只接受 `127.0.0.1` / `localhost` 并要求默认本机 ROS2 模式。
持续发送 0.12m/s 软件候选指令、查询真正的 ROS2 订阅回环、检测模拟
车在地图世界系 x 方向前进后是否移动超过 4mm，结束时一定请求 STOP。
结果中的 `received_any_ros2_echo` 与 `mock_moved_west_over_4mm`
能区分 ROS2 消息未回传、模型碰撞、浏览器没有正确渲染。

直接读取链路诊断 JSON：

```bash
curl -s http://127.0.0.1:8090/api/keyboard/health | python3 -m json.tool
curl -s http://127.0.0.1:8090/api/keyboard/dryrun | python3 -m json.tool
```

若第一个 API 返回 404，说明服务进程仍加载了旧代码：需要**重启**
`server.py`，而不只是 git pull / 浏览器刷新。
`README.md` 和 `sim2real/data/shared_scene.json` 是用户本地文件，
不要运行 `git reset --hard`、`git clean -fd` 或覆盖这些文件。


## 修复「先动一下 → 回原位 → 再也不动」

本次结合 Ubuntu 实际反馈修复了三个不同问题：

1. 前端 `setInterval` 只为物理 Shift 续期，鼠标「按住本机前进」漏掉了 `holdTest`，所以第一次 POST 约 300ms 后服务端自动输出零。现在只要 `deadmanHeld()` 且仍有行驶键按住，就持续刷新 TTL。
2. 地图是原赛场模拟器与本机 ROS2 控制台共用的。以前除了紫色 MOCK，还叠加了一辆**与 ROS2 无关的青色比赛模拟车**；那辆车会在原比赛模拟器重新规划时回到起点。现在只在 /keyboard 隐藏它，地图只留紫色 ROS2-MOCK。
3. 共享场景异步加载/更新可能触发两次 HTTP 复位或乱序状态回读。现增加 `reset_seq` 严格单调保护，忽略过期 HTTP 结果；场景真实变化后安全停止，界面保留 `MAP_RESET_REARM_REQUIRED` 黄色提示，要求**松开再重新按住**，不会无提示地让原来的长按状态失效。

**如何验证：**

```bash
# 更新后先停止旧的 server.py；在终端1重启：
cd ~/Desktop/car
git pull --ff-only
python3 sim2real/server.py --host 127.0.0.1 --port 8090

# 终端2：
export ROS_DOMAIN_ID=42
bash algorithm/car_ws/bash/start_keyboard_local_ros2.sh http://127.0.0.1:8090

# 终端3，停下其他按键操作：
python3 sim2real/keyboard_local_smoke.py --seconds 3
```

浏览器 Ctrl+Shift+R 强制刷新后进入 `/keyboard`。测试时要看**紫色 MOCK**
而不是旧版青色比赛小车。松开 Shift 后
`desired.active=false` 是**预期的安全停车**；
`NO_REAL_ODOM` 是**没有实体硬件时预期的空反馈**。
用浏览器真正持续按住 Shift+W 或鼠标持续按住本机前进按钮时才应看到
`desired.active=true`。在后台场景确实发生改变后，
黄色 `MAP_RESET_REARM_REQUIRED` 会明确要求重新按键。

新增纯离线 `sim2real/tests/test_keyboard_recovery.py` 检查：
300ms 心跳更新、乱序/重复场景复位拒绝、MOCK/REAL 隔离、
紫色车唯一显示及 HTTP 乱序响应保护。

