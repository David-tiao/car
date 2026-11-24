# 当前进度与工程状态（2026-11-22）

**审计基线**：`main@1a7a696b7609585b94618f1284fd5c3120300039`。本文件是代码/文档/CI 快照；不虚构当前设备在线、现场标定和实机比赛成绩。维护时更新日期与基线，不能把历史时点当“永久现在”。

## 1. 项目目标与当前结论

目标：2027 工创智能搬运小车，在 2.4 m × 2.4 m 赛场利用麦克纳姆底盘、OpenMV H7 Plus、STM32F407/C30D、ICM20948、树莓派 5/ROS2 Humble、激光雷达，完成任务码识别、道路规划与障碍判断、精准对位、两批物料的抓取/放置/码垛和返回启停区，提供网页仿真与可观测性。

**项目目前属于“软件模块、协议与模拟验收基础较完整；硬件标定及整车联调尚未闭环”的阶段**。最突出的进展是把相机、STM32、ROS2、网页与软件测试拆成可独立核验的模块；最关键缺口是从真实传感器到实车执行器的可信闭环。

## 2. 交付现状（不要用单一百分比掩盖边界）

| 子系统 | 仓库里已有的内容 | 证据级别与未完成点 |
| --- | --- | --- |
| 竞赛规则、场地地图 | 2.4m 场地、物理占据图/keepout 掩膜、九站道路拓扑、V6 静态核对件 | **代码/离线文件已具备**。V6 为核对版，现场随机障碍与 QR 板不可硬编码；Nav2 keepout filter 启用和真实位置需验证 |
| OpenMV H7 Plus | 四文件 `main.py/config.py/tools.py/serial.py`；二维码、颜色/物块、盘面、数字环；V4 与可选 V5 通信 | **主机协议/静态测试通过**，图像算法已有实现。LAB、透视/相机外参、抓取像素、遮号窗口、光照鲁棒性未有本轮实机验收 |
| STM32F407/C30D | 麦轮逆/正运动学、四轮 PID、编码器、IMU 融合、比赛 FSM、圆环纠偏、抓取 FSM、ULog43 | **已有主机 GCC 和历史 Keil 构建记录**；2026-11-22 实机烧录/轮向/机械/自动竞赛未据此确认 |
| 真实圆环微调 | V4 `ALIGN/STEP_DONE/ARRIVED` 会话协议、2×2 图像-车体标定矩阵反算与限幅 | **代码与离线测试**；`race_config.h` 中 `J00/J01/J10/J11` 均为 0，标定安全门拒绝自动移动 |
| 雷达 / ROS2 | YDLIDAR 驱动相关包、静态地图、占据/禁行分层、开局锁路、Dijkstra 九站/八段路径预览、任务模拟执行 | **路径预览和离线模拟测试**；实际 /scan、TF、雷达定位、障碍锁定、停车目标实测及实车轨迹跟踪没有完整证据 |
| STM32→Pi 遥测 | USART2 ULog43，ROS2 `car_stm32_bridge` 只读解析 /odom 等话题；CAR-LINK V1 模拟桥和 PTY 测试链 | **解码和模拟消息可测**；真实串口、坐标/时间戳、功率与里程计长期可信度待实机核验 |
| Pi→STM32 控制 | 候选 ROS2 主题、软件安全门、指令/回执的 TEST PTY/FSM 实验 | **未接实车可写闭环**；不可声称网页/PC 已能驱动真实底盘或夹爪 |
| 四模式网站 | `/sim`、`/test`、`/real`、`/keyboard`；共享前端资源；API 源标识和离线检测 | **软件/单元测试通过**；运行集真实数据必须经实体硬件提供 |
| 本机键盘底盘 | W/S 前后、A/D 横移、Q/E 旋转，localhost HTTP→PC ROS2→订阅回环→紫色 MOCK 位姿积分 | **PC 软件回环设计和测试**；`/cmd_vel`/STM32 不授权，MOCK 位姿不能代表真实 /odom |
| 机械五轴网页预览 | PC7 转盘、步进升降、PB15 夹持机构翻转、PB14 夹爪开合、升降立柱总成绕竖直轴旋转 | **纯浏览器 MOCK**；第五轴尚未定义 MCU 驱动和限位；立柱 ±180° 是演示要求，不是机械安全行程 |
| 统一离线 QA | 任务路线、串口/CRC、OpenMV、STM32 C、ROS2 接口契约、JS 与网页接口、模拟执行 | [2026-11-21 GitHub CI 成功记录](https://github.com/David-tiao/car/actions/runs/44878008112)；不等于实机验收 |

## 3. 实际使用的四个网页模式

| 模式 | URL | 数据产生者 | 安全声明 |
| --- | --- | --- | --- |
| 纯模拟 SIM | `/sim` | 浏览器本地模拟与既有赛场场景 | 与真实硬件没有控制/遥测必然联系 |
| 测试 TEST | `/test` | 模拟 MCU→Linux PTY→CAR-LINK→ROS2→HTTP | 模拟 ACK、轨迹及轮速不能冒充真实电控执行 |
| 运行 REAL | `/real` | 真实 STM32 串口遥测→ROS2→HTTP | **只读**；未收到可信真实帧时必须离线 |
| 映射 KEYBOARD | `/keyboard` | WASDQE→本机 ROS2 软件回环→MOCK 紫色车；机构只在浏览器 | 候选命令和模型是“软件”，**没有 STM32 电机发送权限** |

统一网页底层见 [`sim2real/FOUR_MODES.md`](../sim2real/FOUR_MODES.md) 和 [`sim2real/web_mode_gateway.py`](../sim2real/web_mode_gateway.py)。REAL 和 TEST 的 `source_type` 必须分离；程序已禁止用 TEST 替代 REAL。

### 五个机械自由度的实际含义

| 网页模型轴 | 机构动作 | 当前 STM32 证据 | MOCK 快捷键 |
| --- | --- | --- | --- |
| `tray_deg` | 三工位原料转盘绕竖直轴 | PC7 / `SERVO_3` | J/L；1/2/3/4=0/90/180/270° |
| `lift_mm` | 夹爪沿立柱上下升降 | PC6 脉冲、PC8/PC9 方向，步进轴 | I/K；T=24mm 演示低位 |
| `holder_deg` | 夹持机构翻转 | PB15 / `SERVO_2` | U/O |
| `grip_deg` | 夹爪开合 | PB14 / `SERVO_1` | N/M；Z/X/C |
| `column_yaw_deg` | **升降立柱、滑台、腕部、夹爪整体绕竖直轴旋转** | **未找到第五轴对应的固件舵机/步进输出** | [ / ]；5/6/7/8/9=-90/0/+90/+180/-180° |

注意：转的是**升降柱总成**，不是尾部转盘底座；柱身仍保持直立；PC7 转盘与其独立。网页版第五轴可旋转到 ±180°，但未检查线束缠绕、机械干涉、真正电机行程。`R` 只把网页模型目标清零，不是安全归零。

## 4. 比赛任务完成程度

预定业务链：

~~~text
二维码有效且唯一 → 解析两批任务码 → 形成规则路线
 → 原料区识别与确认 → 第一批三件逐个抓取
 → 粗加工区三个工位放置/再取回 → 暂存区放置底层
 → 第二批三件逐个抓取 → 粗加工区按任务码放置/取回
 → 暂存区按第一批颜色→底座映射码垛
 → 安全返回指定启停区 → 结果核对
~~~

实现现状：

- OpenMV V5 历史任务规划已经生成 24 项抓放动作 + 1 次返程、第一批颜色与底座映射，后续 `#AT/#OBS/#USE/#RESULT` 握手在相机侧已有实现；历史说明明确需要主控补回执才能完成第 4–25 步。
- 当前 `algorithm/openmv/openmv_all/config.py` 默认选 **V4**，并非将所有旧 V5 25 步流程无修改接入 STM32。V4 对接范围和 `JOB` 的支持以当下相机端 `main.py` 与电控 `vision.c` 的联合测试为准。
- ROS2 `rule_mission_core` 提供九站路线预览，`mission_execution.py` 在 SIM_ONLY 执行 ACK/COMPLETED，并记录抓放统计；这些数字属于**模拟状态机**，不是物料真的放到了规定位置。
- STM32 正式配置 `RACE_PICK_DIRECT_KEY_TEST=1`，说明仍保留直接按键夹取测试入口；不能因此称“已完成自动比赛全流程”。

## 5. 风险与当前阻塞（按联调优先级）

**P0 / 必须在实车运动前解决**

1. 机电安全：电源、接地、急停、轮向、编码器、实际限位、机械行程；五轴中第五轴的驱动、零点和是否允许转 180° 均未知。
2. ROS2/STM32 控制权：`/cmd_vel` 到实际 STM32 可写命令和执行回执尚未确立，现有只读遥测和模拟 PTY 不得当作替代。
3. OpenMV/STM32 V4：QR 接收、`JOB,40/41`、`ALIGN` 等在真实串口上逐步验收；超时、复位、取消、帧分片必须逐项验证。
4. 圆环与抓取标定：相机外参、环中心→底盘位移 2×2 矩阵、夹爪抓取像素、尺寸/方向/机械限位必须实测。
5. ROS2 雷达/地图坐标：`map→odom→base_link→laser_frame`、随机障碍锁路、静态图禁行过滤与停车点实测。

**P1 / 工程效率和质量**

6. 2026-11-21 合并后旧分支的远程删除未通过 GitHub 工具实施，只完成代码与提交历史合并；用 `git branch -r` / GitHub 分支页确认。
7. 根 README 历史遗留单行冲突标记已清理；不据此推断代码文件仍处于 Git rebase。
8. 早期文档混有历史分支、V5 流程、旧 Shift/障碍 FSM；查“当前怎么操作”应以现行源码和本目录的快照为准。
9. 网页对机构的姿态是演示状态，必须继续防止将 MOCK 数值或 PC ROS2 软件回环误表示为实测电机/里程计。

## 6. 近期最应交付的可验证成果

- 真实 OpenMV↔STM32 V4 串口收发试验记录：完整帧、分片、CRC/XOR、重试、超时、会话号。
- 真实 OpenMV 颜色/数字环混淆矩阵、抓取点标定及恶劣光照样本集。
- STM32 实测四轮正反方向、转向误差、驱动安全与控制周期、机械四轴动作及第五轴结构核查。
- Pi 只读 `/odom` 的有效帧和 TF 验证报告，再评估由谁拥有真正的底盘控制权。
- 九站路线的现场障碍/禁行区域确认和循迹闭环日志。
- 一次端到端“扫码→一个物料抓放→完成反馈”的最小闭环，之后扩展至整批与码垛。

详细可执行计划见 [路线图与验收标准](ROADMAP_AND_ACCEPTANCE.md)，代码依据见 [架构与算法](ARCHITECTURE_AND_ALGORITHMS.md)。

## Gazebo 建模与导航

99fcf34 已验证：[双路线运行 CI #45359460287](https://github.com/David-tiao/car/actions/runs/45359460287) 两组独立 Fortress/Humble 实际运行均 PASS；锁定 B2-B3、B3-C3，C3→C2 1/1 路段、终点误差 0.0387m；C3→C2→B2→A2→A1 4/4 路段、终点误差 0.0374m；均零驱动与静止里程计。[原生双架构 CI #45359460641](https://github.com/David-tiao/car/actions/runs/45359460641) x86_64/ARM64 12 包原生构建及 ELF/包前缀、2 测试、地图 probe PASS；[全量离线 CI #45359460151](https://github.com/David-tiao/car/actions/runs/45359460151) PASS。 已解决固定场景的扫描接缝、TF 阻塞等待、整条路径转弯失败及二维节点朝向检查问题。逐边 FollowPath，实际动作成功后验证节点位置和静止里程计，再发下一边；保留原带朝向检查器。当前仍是理想全向运动模型，完整 CAD、连续净空与全部场景、桌面 GUI、真实位姿镜像未验收。

## 跨电脑部署适配

主要入口自动定位工作区并可显式覆盖 ROS/SDK/雷达配置；独立 PC/Pi5/SIM 部署配置，拒绝过期 CMake 路径/架构缓存，检查本地包前缀和原生 ELF。[原生双架构 CI #45359460641](https://github.com/David-tiao/car/actions/runs/45359460641) 两端均通过 12 包构建、7 ELF 校验、2 项软件测试及静态地图 probe；[全量离线 CI #45359460151](https://github.com/David-tiao/car/actions/runs/45359460151) 14 类、300 组软件回归及新增扫描/路径测试通过。不是 Pi5 真机、真实 YDLIDAR、STM32/OpenMV 联调。P4 已解除历史 201 个跟踪构建项（开发分支），本地用户数据、系统 SDK 与 Gazebo 均未修改；更新前必须本地备份。

P5：双后端适配 2c56861（实际运行验收通过）。统一选择、ABI 校验、场景及 bridge 类型转换，Fortress 保留原参数。[全量离线 CI #45384544126](https://github.com/David-tiao/car/actions/runs/45384544126) PASS；[双后端四路线运行 CI #45384544228](https://github.com/David-tiao/car/actions/runs/45384544228) 与 [双架构 CI #45384544069](https://github.com/David-tiao/car/actions/runs/45384544069) 均已通过。只读环境检查不变更系统包；本机 Harmonic 是否具备配套 Humble bridge 需 doctor 确认。GUI/REAL 镜像、实车仍独立待验收。

2c56861 实际运行验收：[四组合 CI #45384544228](https://github.com/David-tiao/car/actions/runs/45384544228) 全部 PASS。Fortress C3→C2 1/1、0.0375m；Fortress C3→A1 4/4、0.0385m；Harmonic C3→C2 1/1、0.0377m；Harmonic C3→A1 4/4、0.0382m。均实测锁定 B2-B3、B3-C3，驱动归零、静止里程计。[双架构 CI #45384544069](https://github.com/David-tiao/car/actions/runs/45384544069) 两组 PASS；[离线 CI #45384544126](https://github.com/David-tiao/car/actions/runs/45384544126) PASS（14 场景、18 锁路、12 移植及原全量回归）。无界面固定场景，不代表本机 GUI、真实镜像或实车验收。

P4：提交 41faa65 仅解除已核对的 201 个历史雷达生成文件（93 build、83 install、25 log，其中 38 个 symlink，Git blob 合计 8,009,261 字节）的跟踪；当前树匹配项为零，源代码与地图保留，旧 Git 提交历史不重写。[原生雷达重建 CI #45402945082](https://github.com/David-tiao/car/actions/runs/45402945082) 在 x86_64 与 AArch64 新目录中使用锁定 SDK V1.2.7 (ad8e30f9e9315d4bd7544df85571072fdbcd31ea) 重建 ROS 驱动，node/client 两个 ELF、动态库、包注册和 COLCON_IGNORE 恢复均 PASS；此前链接失败已修复。[备份离线 CI #45402945365](https://github.com/David-tiao/car/actions/runs/45402945365) PASS。清理后 [离线回归 #45403210364](https://github.com/David-tiao/car/actions/runs/45403210364) PASS（TRACKED=0、ignore 规则、源代码/配置保留及备份、全量回归），[雷达重建 #45403210292](https://github.com/David-tiao/car/actions/runs/45403210292) 双架构均 PASS（node/client ELF、动态库、包前缀、COLCON_IGNORE）。未接真实雷达、未访问 STM32、未清除任何用户机器文件。
