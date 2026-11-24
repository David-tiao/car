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

## Gazebo / URDF 验收（独立于网页）

目标环境 Ubuntu 22.04 / ROS2 Humble / Gazebo Fortress，需本机已有 ign、ros_gz、colcon。执行位置为仓库根目录。先确认当前分支与安装环境。

```bash
cd algorithm/car_ws
source /opt/ros/humble/setup.bash
python3 src/car_gazebo/tests/test_scene.py
colcon build --packages-select car_navigation car_gazebo
source install/setup.bash
colcon test --packages-select car_gazebo
colcon test-result --verbose
```


### 推荐入口：一键 Gazebo + RViz2

前提：Ubuntu 22.04、ROS2 Humble、与 Humble 匹配的 Gazebo Fortress、Nav2、RViz2。若尚未安装相应包，在已配置 ROS apt 源的 Ubuntu 中执行：

```bash
sudo apt install ros-humble-ros-gz ros-humble-nav2-bringup ros-humble-rviz2 ros-humble-robot-state-publisher python3-colcon-common-extensions
```

构建按上一段命令执行；不必重新编译 STM32 或启动真实雷达。自仓库根目录进入工作空间：

```bash
cd algorithm/car_ws
bash bash/gazebo.sh doctor
bash bash/gazebo.sh start A1
```

成功判据：环境检查通过，Gazebo 三维场地与 RViz2 俯视图打开，收到连续真实仿真时钟、激光、里程计、TF、轮关节，两个模拟障碍锁定、路线一致，Nav2 controller ACTIVE，终端显示 PASS READY；**此时小车静止，不自动启动任务**。默认 C3=(2.05,2.05)，目标 A1=(0.35,0.35)。两个模拟障碍来自固定可复现测试夹具，不是现场真实障碍。

准备开始运动时，使用下节同一 SIM 域的显式 Trigger 服务。关闭本次会话使用 Ctrl+C；不会停止其他项目的进程。若启动失败，查看终端打印的 `launch.log`、`acceptance.log` 和生成的 SDF/URDF。日志默认位于 `~/.cache/car_gazebo/run.*/`；可通过 CAR_GAZEBO_RUN_DIR 指定自有目录。重复实例或已占用域 84 直接拒绝，不自动杀掉未知进程。

仅查看当前 SIM 节点或执行可重复的无界面短路线验收（二者不能同时启动一套新仿真）：

```bash
bash bash/gazebo.sh status
# 先结束 start 会话，再运行 test；需要可用 OpenGL/OGRE2 渲染环境。
bash bash/gazebo.sh test C2
```

test 启动 server-only Gazebo，自动进行显式 SIM 路径测试，不启动真实串口或真实机器人；必须有 PASS READY 和 PASS ROUTE，后者要求 Nav2 SUCCEEDED、确实发生移动、目标误差≤0.06m 且零速度、静止里程计。Linux 无桌面测试可使用已安装的 xvfb/xauth/Mesa：

```bash
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s "-screen 0 1280x720x24" bash bash/gazebo.sh test C2
```

下列分步命令用于排查某个子模块或独立 REAL 镜像，并非正常 SIM 一键启动的必需步骤。不要与一键脚本同时启动另一套仿真。

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

本次离线检查（2026-11-24，开发分支）：Python 3.13.5 实际执行 py_compile，场景单元测试 4/4 通过；生成 preview/mirror XML 后实际解析，重复输出拒绝覆盖均通过。生成器、测试、镜像脚本、launch 及 zones.json 的 Git blob SHA 已与远程核对。ROS2 Humble / colcon / Fortress 当前环境缺失，构建、运行和真实数据联调保持 NOT_RUN。新增 CI 检查入口使用 Python 3.10，CI 结果以实际 workflow run 为准。

### SIM 全向运动与虚拟雷达（手动测试，运行待验收）

只在独立终端使用仿真域；先确认域 84 未被真实硬件/网页或其他实验占用。Ubuntu 22.04 / Humble / Fortress，已安装 robot_state_publisher、ros_gz_bridge、RViz2、colcon，并完成 car_gazebo、car_navigation 构建。每个 SIM 终端从仓库根目录进入 algorithm/car_ws：

```bash
cd algorithm/car_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1
OUT=$(mktemp -d)
ros2 run car_gazebo generate_scene.py --zones "$PWD/maps/competition_v1/zones.json" --output "$OUT" --mode sim --obstacle 1.2 1.5 --obstacle 1.5 2.05
ros2 launch car_gazebo gazebo_view.launch.py mode:=sim world:="$OUT/car_sim.sdf" partition:=car_sim_local
```

预期：Gazebo 中可见四轮车体与两个显式模拟障碍柱，启动时静止；/clock、/odom、/scan、/joint_states 和 TF 连续更新。不能同时启动 real 或 TEST 的桥接。若无激光帧、GPU 渲染失败、frame_id/时间戳不匹配，adapter 拒绝非零速度；检查终端日志，不删除校验。

在另一个同样 source 和 export 的 SIM 终端（不启动任何硬件）：

```bash
ros2 topic echo /odom --once
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 run tf2_ros tf2_echo map laser_frame
ros2 topic pub --rate 10 /car_gz/cmd_vel geometry_msgs/msg/Twist '{linear: {y: -0.1}}'
```

预期：车体沿自身右侧横移，按 Ctrl+C 停止发布后 watchdog 输出零；持续时间应短，观察车体不离开场地。不要通过直接 Gazebo topic 绕过 watchdog。该检查只验证仿真全向运动，不验证真实麦轮物理接触或安全停机。

复用现有地图和锁路算法时，另一个 SIM 终端运行 map publisher（只启动一个 /map publisher）；将规则掩膜话题 remap 至现行道路拓扑输入。确认真实雷达节点不在此域。

```bash
ros2 run car_navigation car_map_publisher --ros-args -p use_sim_time:=true -p map_yaml:="$PWD/maps/competition_v1/competition_navigation_map.yaml" -p mask_yaml:="$PWD/maps/competition_v1/competition_keepout_mask.yaml" -r /car/navigation/keepout_mask:=/keepout_filter_mask
```

另两个相同 SIM 域终端：

```bash
python3 maps/competition_v1/simple_lock_stage/lock_obstacles.py --expected 2
python3 maps/competition_v1/topology_stage6/road_topology.py --start C3 --goal A1 --expected-obstacles 2
```

另开 RViz2（Fixed Frame=map，Display 添加 /map、/keepout_filter_mask、/scan、RobotModel、/competition/locked_road_path、/competition/locked_obstacle_markers）：

```bash
rviz2 --ros-args -p use_sim_time:=true
```

只有扫描实测到两个可靠仿真障碍且道路状态 LOCKED_READY 才应出现锁路路径；看不到两个目标时保持等待，不能硬编码锁路结果。锁路脚本采用原有 CLI，未变更源码；SIM TF/消息时间兼容性需在真实 ROS2 环境验证。手动模式下路径只是预览；使用下节 Nav2 专用模式才可执行，不能同时启动两套 map/lock/topology。不要将本节命令用于 REAL 域。

### SIM Nav2 单段锁路执行（分步诊断）

已安装 ROS2 Humble 的 Nav2 controller、DWB、lifecycle manager 与前述 Gazebo 依赖，重新构建 car_gazebo、car_navigation；仅在域84运行。为使真实仿真起点与拓扑 C3 对应，使用 --start 2.05 2.05；启动 Nav2 速度源（手动 cmd_vel 不再被 adapter 接收）：

```bash
cd algorithm/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select car_navigation car_gazebo
source install/setup.bash
export ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1
OUT=$(mktemp -d)
ros2 run car_gazebo generate_scene.py --zones "$PWD/maps/competition_v1/zones.json" --output "$OUT" --mode sim --start 2.05 2.05 --obstacle 1.2 1.5 --obstacle 1.5 2.05
ros2 launch car_gazebo gazebo_view.launch.py mode:=sim command_source:=nav2 world:="$OUT/car_sim.sdf" partition:=car_sim_local
```

在另一 SIM 终端（同一工作空间与域）启动一次导航，不再重复执行上节 map/lock/topology 手动启动命令：

```bash
cd algorithm/car_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1
ros2 launch car_gazebo sim_nav.launch.py map_dir:="$PWD/maps/competition_v1" start:=C3 goal:=A1
```

等待两个仿真障碍确认并冻结道路，观察 RViz2、/competition/road_status 与局部代价地图。另一个 SIM 终端执行：

```bash
ros2 topic echo /competition/road_status --once --qos-durability transient_local
ros2 action info /follow_path
ros2 service call /car_gz/follow_locked_route std_srvs/srv/Trigger '{}'
ros2 topic echo /car_gz/route_status
```

预期：服务只有在真实仿真扫描锁定、路径一致、起点误差≤0.06m、controller ready 时受理；REQUESTED→FOLLOWING→SUCCEEDED（或 REJECTED/FAILED/STOPPED）；Gazebo 沿锁路路线运动。检查结束位姿与 A1=(0.35,0.35) 的误差。取消：

```bash
ros2 service call /car_gz/cancel_locked_route std_srvs/srv/Trigger '{}'
```

断开原始锁路/道路状态/路径/仿真数据，或发生冲突，速度门应输出零并取消；未开始不得产生非零 nav_checked_cmd。SIM 时钟回退后必须重启导航，不允许复用旧锁。此为仿真验收预期，不代表已经运行成功。原有网页与实体控制链不变。

离线全量软件回归：[Actions #45335849607](https://github.com/David-tiao/car/actions/runs/45335849607)，对应 56b503a，已通过。实际 ROS2/Fortress 无界面集成测试 [运行级 CI #45335849586](https://github.com/David-tiao/car/actions/runs/45335849586) 已完成，结论 FAIL。依赖安装、两个 ROS2 包构建、colcon test 均 PASS；运行检查收到时钟、激光、里程计、TF、地图及 ACTIVE 控制器，但 60s 内未取得一致的锁路路径（`READINESS failed: confirmed live locked path`，锁路状态 `CONFLICT`）。未发起 FollowPath，原因尚未定位，不能视为运动已通过。保留原锁路与速度门，不降低预期障碍数量或跳过检查。[运行日志制品](https://github.com/David-tiao/car/actions/runs/45335849586/artifacts/14893073841)。下一步检查激光反射点簇及地图过滤，识别额外候选来源后复测。GUI 联合显示和真实小车映射仍需单独现场验收。

## 跨电脑部署与路径验收

适用于 Ubuntu 22.04 ROS2 Humble（PC x86_64、Pi5 ARM64 的相应系统与依赖）。所有示例从任意位置开始：先进入你自己的 car 仓库根目录，下面的 `$PWD` 指该目录，而不是固定用户路径。不要在 Conda 或 venv 终端启动 ROS；不要求安装 workplace。若本地有未提交文件，先检查并保留；禁止 reset --hard 或强制覆盖。

```bash
git status -sb
git fetch origin
git rev-parse HEAD
git rev-parse origin/main
```

预期：检查当前分支、未提交项与远端提交；仅当合并不会覆盖本地修改时执行正常快进。迁移或换架构时，保留旧安装及重要忽略文件，使用新工作区构建，不复制旧二进制。在已准备好的 Humble 环境进行软件检查：

```bash
python3 algorithm/car_ws/bash/test_portability.py -v
cd algorithm/car_ws
bash bash/build.sh core
source bash/env.sh
bash bash/web.sh doctor
```

预期：路径测试通过，核心软件包在当前架构构建，web doctor 通过；实际串口、真实 TF 与电机不由这些命令验证。Pi5 不需要 Gazebo/RViz2 桌面依赖才能构建核心包；仅在需要的 PC 上构建 car_navigation/car_gazebo。当前 ARM64 真机仍待验证。

### 路径覆盖（可选）

| 变量/参数 | 默认 | 范围 |
| --- | --- | --- |
| CAR_WS | 脚本推导的 car_ws | 已接入 common.sh 的入口及 V6 |
| CAR_ROS_SETUP | /opt/ros/humble/setup.bash | 已接入 common.sh、V6、部分网页 ROS 入口 |
| CAR_YDLIDAR_SDK_DIR | 自动发现 SDK 配置目录 | 雷达修复，最高优先级，目录含 ydlidar_sdkConfig.cmake |
| CAR_YDLIDAR_SDK_PREFIX | 项目 SDK / CMAKE_PREFIX_PATH / 系统前缀 | 雷达修复 |
| CAR_LIDAR_CONFIG | car_ws/config/x3.yaml | lidar.sh / V6 |
| CAR_SIM2REAL_ROOT | 仓库内 sim2real | web.sh |
| CAR_GAZEBO_RUN_DIR | ~/.cache/car_gazebo | SIM 日志与世界文件 |
| CAR_V6_STATE_DIR | XDG_STATE_HOME 或 ~/.local/state 下的 competition_v6_viewer | V6 本地状态及日志 |

显式覆盖建议使用绝对路径且双引号保护含空格目录；未设置时无需填写任何路径。不同 ROS 工作区不要共用已生成的安装目录或运行状态目录。设备路径通过自有 YAML / 参数配置；不能根据机器名猜测串口。

### 雷达迁移

先结束已有雷达会话并确认实际设备、SDK 与新架构。仅检查 SDK 路径（不编译、下载或打开硬件）：

```bash
bash bash/repair_car_ws_lidar.sh doctor
```

如果 SDK 不在默认路径，设置 CAR_YDLIDAR_SDK_DIR 为实际 CMake 配置目录后重新执行 doctor。已有 SDK、已核实备份且驱动未运行时可执行：

```bash
bash bash/repair_car_ws_lidar.sh build
bash bash/lidar.sh doctor
```

预期：仅重建雷达驱动 overlay，旧 overlay 备份保留，安装失败或校验失败时恢复旧 overlay。doctor 检查实际可执行文件及串口权限，不启动雷达。SDK 缺失时 build 拒绝下载；只有明确审查依赖后使用 --allow-sdk-build，仍仅写项目内目录。不卸载系统 SDK。历史已跟踪的 201 个雷达构建/安装/日志文件暂保留，新增 ignore 规则不影响已跟踪文件；另行审核清理。

### 旧 Nav2 配置

旧两份 YAML 是参数模板，不能直接从任意工作目录交给 Nav2。需要保留比赛版行为树（失败等待，不主动旋转或倒车）时，用现有 XML 生成临时参数文件：

```bash
OUT="$(mktemp -d)"
python3 bash/render_nav2_params.py --output "$OUT/nav2_competition.yaml"
```

预期：输出新文件，行为树路径自动解析为当前工作区绝对路径，其他参数保持不变。可指定 --params 或 --behavior-tree，缺失/错误路径、重复输出会失败。之后仅在已验证的 Nav2 启动入口中使用生成的参数；不因此启用实车控制。当前 Gazebo 单段 FollowPath 使用独立 nav2_sim.yaml，不依赖该历史配置。

### 验收范围

独立离线测试覆盖带空格的临时物理目录、符号链接、不同 cwd、显式 ROS/SDK 覆盖、nounset 恢复、环境拒绝、坏雷达符号链接及行为树配置生成；模拟 ROS 环境只用于验证路径逻辑，不代表真实 ROS2/雷达。还在实际 checkout 的符号链接外执行 web doctor 和 Bash 语法检查。真实 x86_64/ARM64 构建、串口、Gazebo 版本以及运行级路径执行独立验收，不能用路径测试替代。

### 原生 PC/Pi5 部署及验证

从任意路径进入自己的 car 仓库，先进入 algorithm/car_ws；不要求 workplace 或特定用户名。独立终端，不激活 Conda/venv。部署目标自动按 uname -m 选择，以下显式 pi5 只用于 ARM64 Ubuntu 22.04/Humble 树莓派5（PC 将 pi5 改为 pc）；不是跨编译器。

```bash
cd algorithm/car_ws
bash bash/deploy.sh plan pi5
bash bash/deploy.sh doctor pi5
```

预期：显示实际工作区、12 个核心包及依赖；doctor 仅检查已安装工具、系统 Python rclpy 和 ROS 依赖，不检查硬件或修改环境。缺少依赖按各 package.xml/rosdep 提示独立审核，不默认安装。已配置 rosdep 数据库的机器可预览依赖：

```bash
mapfile -t PKG_PATHS < <(bash bash/deploy.sh paths pi5)
rosdep check --from-paths "${PKG_PATHS[@]}" --ignore-src --rosdistro humble
```

预期：输出已满足或缺失的依赖，不安装。新工作区或经核验保留旧产物后，显式构建和验证：

```bash
bash bash/deploy.sh build pi5
bash bash/deploy.sh verify pi5
bash bash/deploy.sh test pi5
```

预期：PASS BUILD、PASS VERIFY、PASS TEST（包注册路径属于本工作区，关键 ELF 机器类型正确、软件测试成功、静态地图 probe 返回有效样例）。默认 isolated install，保留现有启动脚本兼容性。旧 CMakeCache 的源码/构建路径或编译器架构异常时拒绝继续，不自动删除；在干净工作区重新构建。此流程不需要 RViz2/Gazebo，不含 YDLIDAR SDK 编译及串口设备测试，不启动机器人。Pi5 上按上节雷达 doctor/build 另行验证。

PC 若需要仿真并具备 Fortress 与配套 Nav2/RViz2，可选择 `bash bash/deploy.sh doctor sim` 和 `build sim`；Harmonic 不自动替换，尚未提供兼容后端。实际路径闭环仍通过 gazebo.sh 独立验收。路径含空格已通过离线入口测试，完整 colcon 工具链的空格路径支持尚未验证，实际构建推荐普通目录名。

### 历史构建产物审计

```bash
python3 bash/audit_lidar_artifacts.py --list
```

预期：列出明确匹配旧 build_lidar_repair、install_oneclick_lidar 及对应备份、log_lidar_repair 的 Git 跟踪项、符号链接和缺失项计数。仅访问 Git 索引及文件存在性，不执行 rm/git rm，也不检查其他源码目录。当前基线有 201 个已跟踪历史项；将来解除跟踪前，必须先通知队员核验工作副本、备份旧 overlay 及重要本地文件，在新位置重新构建，避免正常 git pull 删除仍被依赖的跟踪文件。旧提交历史保留，不重写历史。

双架构工作流 `.github/workflows/native_deploy.yml` 在原生 Ubuntu 22.04 x64/ARM64 容器环境中安装仅核心包依赖、将 Git 源码解包到临时新物理路径、在另一 cwd 执行 doctor/build/verify/test。CI 结果以对应提交记录为准；不等同 Pi5 真机、雷达、Gazebo GUI 或实车控制验收。
