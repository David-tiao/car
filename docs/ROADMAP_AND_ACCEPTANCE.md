# 下一阶段技术路线、联合调试与验收标准

**基线**：2026-11-22，`main@1a7a696b`。本文件是后续工作的**建议验收顺序**，未打勾的并不是“正在异步完成”，也不代表硬件团队已同意某个时间安排。真正上车必须由电控、机械、视觉与 ROS2 负责人共同确认。

## 1. 开发优先级：先最小真实闭环，再完整比赛

### Phase 0 · 固定版本与机电安全

**目标**：任何会让车或机械运动的试验，均可追溯版本、可靠停机。

- [ ] 确认 STM32 工程、OpenMV 四文件、Pi ROS2 工作空间和网页使用的确切 `git rev-parse HEAD`；不是分别加载过时的四个快照。
- [ ] 检查电源/接地/TTL 电平、保险/限流/急停、供电电流与上电默认状态；记录硬件板版本、引脚接线表。
- [ ] 逐项确认四轮电机编号、正方向与编码器极性，避免把正向控制当作倒转。
- [ ] 与机械确认第五轴的旋转中心、驱动型号、限位、归零、线束缠绕允许角度；没有证据不要驱动 180°。
- [ ] 记录一次完整空载启动、停止、断电复位与急停复测，负载测试只能在通过后进行。

**验收证据**：接线图、供电记录、电流、每台 MCU 固件 SHA、现场照片/视频、停机时间、至少三次同条件重测。缺少现场记录维持 `NOT_RUN`。

### Phase 1 · OpenMV ↔ STM32 V4 最小通信

**目标**：先让真正双板 UART3 往返“同一会话”。

- [ ] OpenMV P4/P5 与 STM32 PD9/PD8 交叉、共地；115200 8N1。
- [ ] 真实串口录制 `READY,IDLE`、`JOB,1`、`ACK,JOB,1`、QR 结果、扫码确认和 `IDLE`。
- [ ] 校验错误 XOR、LEN 错误、随机分片、旧 SEQ/重复 SEQ、掉线、重启、CANCEL/RESET。
- [ ] `JOB,40` 先收 `AREA_OK` 再等 `IDLE`，才启动 `JOB,41`；验证相机不在 busy 时接受新任务。
- [ ] `ALIGN,1/2/3,session` 会话/step 逐步验证，停稳后发送 `STEP_DONE`；未标定时必须拒绝真实底盘移动。
- [ ] 统计 30 次以上合法任务接收率、CRC/XOR 不合法拒绝率、延迟和丢帧。

**验收标准**：命令→响应日志双向匹配，重复/过期帧不会推进状态；有故障时安全停机。**先不让车凭视觉自动移动**。

### Phase 2 · 视觉与机械标定

**目标**：图像中的物料/环“坐标”能映射到真实夹爪和底盘动作。

- [ ] 实测六色 LAB 阈值，在赛场照明变化、反光、远近、部分遮挡下采集带真值图像并建立混淆矩阵。
- [ ] QR 合法/非法码、重复码、两批颜色映射和三帧确认测误判率；录像保留对照。
- [ ] 标定相机朝向/固定方式/光心、抓取 `PICK_TARGET_X/Y`、夹爪当前位置与像素偏差。
- [ ] 验证“第一次锁盘三色几何”的现场条件；如确需两色启用，先做数学退化条件与历史盘心恢复策略，不能把三色模型的阈值直接降低。
- [ ] 圆环数字 1/2/3、5×7 模板、旋转与同心暗环验证，遮挡数字使用 `COVERED_WINDOWS` 需按粗加工区/暂存区分别实测。
- [ ] 在安全空载下估计 `J00/J01/J10/J11`（像素/m 的 2×2 雅可比），检查行列式、正负轴、重复误差及限幅；矩阵为零时仍拒绝执行。
- [ ] 校准真实 PB14、PB15、PC7 和升降步进脉冲/mm、低位/高位、机械干涉；每个动作有物理结果判据，不以“舵机目标已发出”代替到位。

**验收证据**：采样图片与标注 CSV、阈值配置 SHA、位姿-像素对应记录、重复性统计、微调方向测试、极限位碰撞/电气防护报告。

### Phase 3 · ROS2 真实里程计与雷达

**目标**：从真实底盘获取可信位姿，再考虑远程控制。

- [ ] Pi5 连接 STM32 USART2 ULog43，验证 43 列与单位、时间戳、连续里程计 `/odom`、轮速与 IMU 航向；不得把 TEST PTY 数据写进 REAL。
- [ ] 实测 `map→odom→base_link→laser_frame` TF 及雷达安装外参，检查 yaw 正向、单位和时间同步。
- [ ] 验证静态物理占据图和 Nav2 keepout mask 分层加载，黄色禁区不作为物理墙，但规则层禁止规划进入。
- [ ] 在随机路障下稳定生成 `/competition/road_status`，必须是真实 `source=LIDAR_CONFIRMED`、`LOCKED_READY`、`frozen=true` 且无冲突。
- [ ] 用真实扫描验证 QR 板与原料盘几何定位/停车观测，不使用示例的 `confidence=0.91` 等伪数据。
- [ ] 固定 9 站路线，分别执行无障碍、有阻断和不可达用例；若边封锁，应返回 NO_ROUTE 而不是凭空跨过障碍。

**验收标准**：闭环线下地图定位和路径结果可以被 ROS2 / RViz 对照复现，终端停止时 UI 离线而不是继续绘制虚假的轨迹。

### Phase 4 · Pi→STM32 的可写控制协议（在物理安全门完成后）

**目标**：由明确单点控制者驱动真实 STM32，而不是从只读桥“顺便发点数据”。

- [ ] 确定该接口是 UART 还是其他总线：哪一端唯一有权发布速度目标、停止、里程计复位、夹爪请求，谁可以覆盖任务状态机。
- [ ] 定义完整帧：协议版本、seq、cmd_id、时间、frame_id、目标、限速、鉴权/授权、超时、校验、拒绝原因、ACK 和实际 COMPLETED。
- [ ] 定义 deadman（松键即停）/急停：松键、通信中断、异常重启、重复帧、过期任务、断电、限位故障如何停止。
- [ ] 先在 TEST PTY 和 STM32 主机 stub 中执行 ACK、重复、取消、超时、断链、急停测试，记录停止时间和命令幂等。
- [ ] 再采用车轮悬空、机械卸载、功率限流、逐轴授权，完成真实空载小速度测试。
- [ ] 严禁直接把网页 `/api/keyboard/intent` 或只读 `/cmd_vel` 观察接口接到电机驱动而跳过控制权审查。

**验收标准**：实机能受控开始、受控停止、故障锁存和恢复，执行回执能从电控返回 Pi；只有这之后才考虑启用真实运动接口。

### Phase 5 · 单件抓取到整车任务

- [ ] 先完整做 **一个物料**：扫码→目标确认→安全移动→夹爪对齐→抓取→回执→放置→再次确认。
- [ ] 再做第一批三件、粗加工三环、暂存底层。
- [ ] 第二批按“第一批颜色→暂存底座环号”码垛，先验证色号/环号映射，再碰实物。
- [ ] 验证物料跌落、颜色不符、环号不一致、路径不可达、二维码错误、视觉丢失、机械卡死、15s 无位移等真实故障时如何停止/记录。
- [ ] 全部任务成功后有可信的返回启动区定位和结束事件，而非靠网页动画自动宣称 `DONE`。

## 2. 可执行的离线基线检查

在已拉取最新代码、没有进行中的 rebase 的 Ubuntu 工作区，先检查状态：

~~~bash
cd ~/Desktop/car
git status --short
git fetch origin --prune
git rev-parse origin/main
git log -1 --oneline origin/main
~~~

安全更新：在工作区干净且当前分支允许更新的前提下，按团队 Git 工作流更新；不要盲目 `git reset --hard` 或 `git rebase --skip`。如果仓库仍有未提交修改，应先保留/提交或按需建立专门工作树。

完整软件离线验收（不需要物理 STM32/OpenMV/Pi）：

~~~bash
cd ~/Desktop/car
bash algorithm/car_ws/bash/test_all_offline.sh
# 快速子集：
bash algorithm/car_ws/bash/test_all_offline.sh --quick
~~~

脚本通过 `sim2real/offline_acceptance_suite.py` 运行任务规划、STM32 C/GCC、CAR-LINK PTY、OpenMV V4 QR/协议、网页与 JavaScript 检查，默认报告目录 `/tmp/car_offline_full_acceptance`。失败输出会保留，绝不把后续某项 PASS 覆盖前面的 FAIL。

模块检查（用于定位问题，而非替代整套 CI）：

~~~bash
cd ~/Desktop/car
bash algorithm/car_ws/bash/web.sh doctor
bash algorithm/car_ws/bash/test_keyboard_mapping.sh
node sim2real/tests/test_keyboard_mechanism_preview.js
node sim2real/tests/test_keyboard_mechanism_mapping.js
python3 -m unittest discover -s algorithm/openmv/tests \
  -t algorithm/openmv/tests -p 'test_v4_*py' -v
~~~

部分脚本需要 `node`、`gcc` 和系统依赖；若没有安装，不应写成“测试通过”，应记录 SKIP 或环境缺失。2026-11-21 的基线 CI 已通过：[Actions #44878008112](https://github.com/David-tiao/car/actions/runs/44878008112)。

## 3. 本机软件回环的验收

~~~bash
cd ~/Desktop/car
python3 sim2real/server.py --host 127.0.0.1 --port 8092
# 浏览器访问 http://127.0.0.1:8092/keyboard
~~~

有 ROS2 Humble 的电脑使用网站自动启动本机 PC ROS2 发布/订阅节点（可通过 `CAR_KEYBOARD_AUTOSTART=0` 关闭自动启动）。验收顺序：

1. 按住 W，页面先显示**捕获 W**；再检查 `/api/keyboard/intent` 是否有效；然后 ROS2 echo 与 `/api/keyboard/dryrun` 变为新鲜数据。松键状态回到零速度；不是网页仍播放动画就算成功。
2. Q/E、A/D、W+A 方向符合机器人 `base_link` 约定；浏览器不把旧回包重放成位姿重置。
3. SIM/TEST/REAL/KEYBOARD 的来源保持隔离；REAL 没有设备时 `online=false`。
4. 机构键：J/L（PC7）、I/K（升降）、U/O（夹持翻转）、N/M（开合）、[ / ]（柱旋转）。第五轴 8/9 的 ±180° 只影响 MOCK，PC7 不应被误带动。
5. 浏览器窗口失焦、断 ROS2、断 HTTP、错序和超时，都应进入待机/离线，而不冒充“仍在真实运动”。

**该验收只证明 PC 的 ROS2/网页软件链，不代表 STM32 电机真的被控制。**

## 4. 硬件验收矩阵与记录模板

建议每个项目必须同时填写三项：代码测试、软件运行、实体测量。

| 功能 | 主机/代码测试 | 软件运行测试 | 实体板/场地 |
| --- | --- | --- | --- |
| STM32 电机 PID / IMU | 已有 GCC/历史 Keil 记录 | 可搭假编码器/日志 | **NOT_RUN**（除非新日志证明） |
| OpenMV QR/物料/环 | V4/算法与协议主机测试 | 可上 USB 图像/串口台架 | **NOT_RUN** |
| 圆环 J 标定 | 逆矩阵/失效门控主机可测 | 可输入模拟观测 | **NOT_RUN**，当前矩阵全零 |
| ROS2 TEST PTY | CRC/协议/话题模拟 | ROS2 Humble Ubuntu 实跑需留日志 | 不适用（TEST 即模拟） |
| ROS2 REAL ULog43 | Parser/单位可离线测 | 真串口连接时需留 bag | **NOT_RUN** |
| Pi→STM32 控制 | 候选合同/TEST FSM | 未完成真实可写桥 | **NOT_RUN** |
| 夹爪四现有轴 | 固件/网页模拟 | TEST/浏览器可演示 | **NOT_RUN** |
| 升降柱第五轴 | MOCK 几何/键位测试 | 浏览器 ±180° 可演示 | **未有 MCU 驱动与安全行程证据** |
| 完整比赛 | SIM_ONLY 任务状态机 | 真实 ROS2 多节点需逐步验收 | **NOT_RUN** |

每次实测填写：

~~~text
测试编号/日期/负责人：
仓库 main SHA / STM32 固件 SHA / OpenMV 文件版本：
硬件版本、供电、电流限制、上电/急停条件：
串口与电机引脚、ROS_DOMAIN_ID、source_type：
目标动作和预期数据、实际日志文件/rosbag/视频：
量化指标（误差、时间、次数、异常码）：
PASS / FAIL / SKIP / NOT_RUN：
失败根因、修复提交、回归结果、下一轮负责人：
~~~

严禁写“车动了，所以协议全通”；要能回答 **谁发命令、谁执行、谁确认、物料和底盘真实发生了什么**。源码和历史参考请查 [当前进度](PROJECT_STATUS.md)、[通信对齐](PROTOCOLS_AND_DATA_FLOW.md) 与 [错误复盘](ISSUES_AND_LESSONS.md)。

## Gazebo / URDF 初版验收（独立于网页）

目标环境 Ubuntu 22.04 / ROS2 Humble / Gazebo Fortress，需本机已有 ign、ros_gz、colcon。执行位置为仓库根目录。先确认当前分支与安装环境。

```bash
cd algorithm/car_ws
source /opt/ros/humble/setup.bash
python3 src/car_gazebo/tests/test_scene.py
colcon build --packages-select car_gazebo
source install/setup.bash
colcon test --packages-select car_gazebo
colcon test-result --verbose
```

静态预览：显式指定模拟障碍坐标，不会改动现行 PGM/YAML。输出目录必须是新目录，生成器拒绝覆盖已有文件。

```bash
OUT=$(mktemp -d)
ros2 run car_gazebo generate_scene.py --zones "$PWD/maps/competition_v1/zones.json" --output "$OUT" --obstacle 1.2 1.2
ros2 launch car_gazebo gazebo_view.launch.py world:="$OUT/car_preview.sdf" partition:=car_preview_local
```

预期：赛场 2.4m × 2.4m，黄色/蓝色区、两个工位、原料盘、一个黑柱及四轮静态车；模型暂时不运动。前台 Ctrl+C 退出自己的 launch，不清理其他项目进程。

真实镜像：确保实际雷达、定位/里程计链已经提供**真实**且新鲜的 map→base_link TF，运行在 REAL ROS domain，不能与模拟 TF 共用。先在工作空间另生成镜像场景：

```bash
OUT=$(mktemp -d)
ros2 run car_gazebo generate_scene.py --zones "$PWD/maps/competition_v1/zones.json" --output "$OUT" --mode mirror
ros2 launch car_gazebo gazebo_view.launch.py world:="$OUT/car_mirror.sdf" partition:=car_mirror_local
```

另一个已 source 同一工作空间的终端：

```bash
IGN_PARTITION=car_mirror_local ros2 run ros_gz_bridge parameter_bridge /world/car_mirror/set_pose@ros_gz_interfaces/srv/SetEntityPose
```

确认服务存在，再在同一真实 ROS domain 启动（不要启动任何真车运动）：

```bash
ros2 service type /world/car_mirror/set_pose
ros2 run tf2_ros tf2_echo map base_link
ros2 run car_gazebo mirror_pose.py --ros-args -p enable_real_mirror:=true -p use_sim_time:=false
```

成功判据：服务类型一致；新鲜真实 TF 更新时镜像跟随 x/y/yaw，断 TF 时停止刷新且终端告警；没有 /cmd_vel 写入、模拟 /clock 或雷达回灌。源数据归属仍须现场确认。Gazebo 运行、服务调用、URDF/RViz 显示和实车镜像均尚未验收。官方接口依据：[Fortress world](https://gazebosim.org/docs/fortress/sdf_worlds/)、[Humble SetEntityPose](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_interfaces/srv/SetEntityPose.srv)。
