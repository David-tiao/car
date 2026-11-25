# 二维码板 / 原料盘区域雷达扫描（只读实验）

本模块复用 X3 的 /scan，不接管串口、不发布 TF 或 /cmd_vel，不改变原障碍聚类、道路锁定、Nav2 或 STM32。
比赛规则图1与现有 zones.json 给出的搜索先验：地图左下为 (0,0)，右侧二维码板中心 y 约 1.1~1.3m；上侧原料盘中心 x 约 1.1~1.3m、半径 .15m。规则图1标注圆盘伸入赛场 75~85mm，因此按 2.4m 边界推算中心 y=2.465~2.475m；现有 zones.json 的 y=2.4m 仅为旧地图名义先验，仍纳入扫描范围。区域边界不是现场测量结果。二维码 A4 横放，不能把 80mm 二维码的尺寸当作雷达板宽。

## 规则依据与限制

2027 智能搬运初赛规则（附件2-1，第3、4、6页）：2.4m 方形场地、400mm 十字车道、随机 Φ50×100mm 障碍物；A4 横放的垂直二维码板、码本体 80×80mm（含边框）；原料圆盘 Φ300mm、总高度 80~100mm、物料中心轨迹 Φ200mm。板和原料区位置允许变化。本模块只使用物理尺寸和搜索先验，不将名义位置作为测量值，不解码二维码，不改变比赛禁行区域和自主运行边界。

ROI 默认：qr_roi=[2.25,0.90,2.55,1.50]；raw_roi=[0.90,2.15,1.50,2.65]（米）。直线候选长度 .12~.35m、至少5点、RMS≤.015m；圆弧半径 .12~.18m、至少6点、覆盖角≥45°、RMS≤.015m。均为待实测的工程阈值，不是赛规公差。

## 启动

在 car/algorithm/car_ws 下，确保同一 ROS_DOMAIN_ID、同一时间源、系统 Python，无 Conda/venv。若 V6 或其他程序已经运行雷达，不重复启动。

推荐统一入口（先保证小车静止、真实外参和地图位姿已发布）：

```bash
cd algorithm/car_ws
bash bash/lidar.sh test-regions
# 已有 V6 或雷达发布 /scan：
bash bash/lidar.sh regions
# 或者，没有其他驱动时同时启动雷达与观察器：
bash bash/lidar.sh start-regions
```

两种在线模式二选一。前置检查拒绝同一 ROS 域内已发现的区域发布者及不符合预期的 /scan 发布者数量；联合模式额外检查串口和驱动。启动后 12 秒内必须获得有效扫描及首末时刻 TF，并满足静止门控，才报告 READY；不要求目标存在。联合模式终端 Ctrl+C 或任一子进程退出时，只清理本次拥有的子进程，日志保留在 `$XDG_STATE_HOME/car-lidar/region.*`（未设置时为 `~/.local/state/car-lidar/region.*`）。不自动杀其它程序；有限的 ROS 发现时间和本用户锁不能保证其他用户/主机全局互斥。原 `start` 和 `doctor` 行为保持不变。前置 TF 不由脚本生成。

以下为分步调试方式。终端 A（仅在没有现有驱动时）：
```bash
bash bash/lidar.sh doctor
bash bash/lidar.sh start
```

终端 B：使用已实测的 base_link->laser_frame 与独立可信的 map->base_link 静止测试位姿（或实际 map->odom->base_link）。现有 V6 的 stationary_tf 可以在两个实测值齐全时提供静止 TF；不得移动使用固定 map 位姿的小车。禁止用零值冒充实测值、重复发布同一子帧，或把仿真 TF 当实车 TF。

终端 C：
```bash
source /opt/ros/humble/setup.bash
python3 maps/competition_v1/lidar_stage5/scan_regions.py
```

终端 D：
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /competition/lidar/regions
```

可用 ROS 参数 qr_roi 和 raw_roi 覆盖搜索范围（xmin,ymin,xmax,ymax，米），但圆心先验及半径阈值仍按本赛场固定；改变赛场须同时审核拟合条件。

## 输出

std_msgs/String JSON，正常 status=OBSERVATION_ONLY，含 scan stamp_ns 和 qr_board、raw_disk。顶层 identity_verified=false、stationary_test_only=true；任何候选都不是控制授权。
- NO_RETURN：区域内无有效回波；不代表物体不存在。
- RETURN_ONLY：有回波，几何不满足条件。nearest_region_return_m 为车体原点到最近可见区域回波的平面距离，非目标确证距离。
- GEOMETRY_CANDIDATE：一个几何候选；AMBIGUOUS：多个候选；不等于身份确认。
- candidates: center_map_m、center_base_m、center_distance_m、bearing_rad、nearest_candidate_return_m、rms_m、point_count。二维码候选还有 observed_span_m（观测线段长度）、tangent_yaw_map_rad（无向直线方向）、perpendicular_line_distance_m（车体原点到拟合无限直线的垂距，不是板中心距离）、wall_or_board_unresolved=true；原料盘候选有 radius_m 与 arc_rad。
- 二维码候选中心是观测线段均值，不一定是实体板中心。盘心为拟合估计，不是盘沿最近距离。
- INVALID_SCAN / NONMONOTONIC_SCAN / TF_OR_SCAN_STALE / SCAN_STALE / INVALID_GEOMETRY / MULTIPLE_SCAN_PUBLISHERS / SCAN_QUEUE_OVERFLOW / MOTION_UNSUPPORTED：无可用目标结果，消费者必须丢弃旧数据并自行检查时间戳，不可据此控制车。
- 数据无视觉确认、无遮挡保证、连续多帧目标稳定门控或停车控制授权。不能据此解除原障碍锁定或自动发车。
- 静止门控：首末时刻分别查询 map->laser_frame 和 map->base_link，平移变化≤5mm、yaw变化≤0.3°（初值）。以稳态时钟监控超时，不用最新 TF 补历史扫描。只比较首末时刻，不能检测中途运动后回到原位，更不能识破伪静止 TF；仅允许物理静止测试，不是运动去畸变。时间戳回退后重启节点再测试，禁止将旧测量当新结果。

## 验收

```bash
python3 maps/competition_v1/lidar_stage5/scan_regions.py --self-test
```

内置纯 Python 测试覆盖直线、圆弧、退化、歧义、空区域、坐标转换、无效扫描、21组射线模拟（原料盘 3 个横向位置 × 3 个纵向位置 × 2 种角采样密度，另含3组二维码板）、正反方向接缝、时序、运动门控和墙面负例；断言关闭时明确失败。统一离线入口 `bash bash/test_all_offline.sh` 先运行区域测试再执行原全量套件。测试不等同实机验收。

实车依次：确认雷达安装高度和水平姿态；测量真实外参、地图静止位姿；量尺对照不同距离和随机目标位置，记录回波数、候选数、距离误差及误识别；遮挡、断开扫描、去掉 TF、用墙面或其它圆形物体作负例；确认现有道路锁定和控制接口无回归。若扫描平面错过板面/盘体，不改阈值冒充成功。只有在视觉身份确认、连续帧可靠性和实际停车误差验收后，才考虑接入运动闭环。

## 前一提交的交付与证据（2026-11-25）

授权范围仅四个既有文件：本文件、scan_regions.py、bash/lidar.sh、bash/test_all_offline.sh。不修改 X3 驱动、外参、地图、道路锁定、STM32/OpenMV 或通信协议；不合并 main。

- Python 3.13.5：隔离目录复制 GitHub 源码并核对 blob 9001a65f65f78c05e764fbaf24d82ae7d22d608d，`python3 -m py_compile scan_regions.py` 与 `python3 scan_regions.py --self-test` 退出码0；优化模式拒绝测试退出码1（预期）。
- Bash 5.2.37：核对 lidar.sh blob 23eadcdcfea7aa938e0419f8acdd2c5ddfb027bd、test_all_offline.sh blob 3de0fe423fe6824afb4004d8b8776d094bc8af8d；两个 `bash -n` 退出码0；从其它目录运行 `bash lidar.sh test-regions` 退出码0；非法模式退出码2、缺 ROS setup 退出码1（预期）。
- 隔离测试替身：9项观察器状态断言（有效、非单调、非法、重复源、运动、TF过期、断流、无有效范围、队列溢出）通过；8项模拟启动测试通过（doctor/start 兼容、attach 有/无扫描、重复源拒绝、串口占用拒绝、联合子进程退出和启动失败），两次核查模拟驱动进程已退出。替身仅为临时测试夹具，不提交、不代表 ROS2 API 或硬件实际运行。首次清理测试因模拟驱动未及时写入 PID 无法断言，修改夹具后复验通过，未以该次失败证明真实清理行为。
- UNVERIFIED：当前隔离环境缺 ROS2 Humble、无法直接 git clone 完整仓库，未执行全量离线套件、ROS2 集成、X3 实机扫描及实际标定。实际板面/墙面重合和盘沿被遮挡仍不可辨识；尚不支持移动中扫描去畸变或车辆停车控制。
- 最终状态 INCOMPLETE：软件几何/语法及模拟分支测试通过，实机验收未完成。不得用于自动驾驶控制。

## 规则图1边界复核

规则第3页图1的原料盘伸入场地尺寸为75~85mm；在原料盘半径150mm、上边界 y=2.4m 的坐标约定下，中心推算为 y=2.465~2.475m。此推算不是实测外参，也不改变原地图。将原料 ROI 上界由2.58m调整到2.65m以覆盖圆盘外缘，并补充两种角采样密度的射线回归（总21组）。上述旧提交的本地测试记录不覆盖本次变更；最终验收以本次提交的测试证据为准。

## Workplace 实车外参标定辅助（只读采集 + 离线计算）

复用 workplace 的 ROS 2 Humble 系统 Python 环境与 Car 唯一主要工作副本；不创建虚拟环境、不安装依赖、不硬编码个人绝对路径。原始扫描属于本地采集产物，不进入 Git。以下示例目录按 workplace 推荐布局，实际路径以本机 dev projects 检查为准。

```bash
source ~/workspace/00_system/workplace/scripts/activate.sh ros
cd ~/workspace/05_project_github/car/algorithm/car_ws
bash bash/lidar.sh test-regions
```

1. 将车停稳并禁止底盘运动，雷达固定且水平。车体 +X 前方和 +Y 左侧分别放置两面平整垂直墙，精确对齐车体轴向，两墙法线垂直。尺量 **base_link 原点** 到前墙、左墙的垂距（米），不是雷达外壳到墙的距离。确保雷达扫描平面打到墙面，清空遮挡。
2. 使用已有雷达 /scan；没有驱动时另开终端执行原 lidar.sh start。不要重复启动。用 RViz LaserScan（Fixed Frame=laser_frame）确认两个墙的法线大致角度，按雷达 +X 为0°、逆时针为正；选取各墙单独回波的角度窗口，避免包含墙角、桌腿、其他物体。
3. 在本地显式建立产物目录后采集30帧，不依赖外参或 map TF，不发布任何话题：

```bash
mkdir -p ~/workspace/12_artifacts/car/lidar
bash bash/lidar.sh capture ~/workspace/12_artifacts/car/lidar/walls-01.json --frames 30
```

采集只接受单个已发现 /scan 发布者、laser_frame、递增且新鲜的时间戳、有效量程和扫描时间；无效回波编码为 JSON null；未收齐帧则失败且不创建正常结果文件，已有文件不会覆盖。人工保持物理静止，原始采集无 TF 不能自动验证静止。

4. 将下列变量替换为**实测值**（没有实测值不要运行），同一份采集可反复离线计算：

```bash
bash bash/lidar.sh calibrate ~/workspace/12_artifacts/car/lidar/walls-01.json \
  --front-m "$FRONT_M" --left-m "$LEFT_M" \
  --front-deg "$FRONT_DEG" --left-deg "$LEFT_DEG" --half-deg 20
```

两墙分别做最小二乘正交直线拟合，法线朝墙；前墙和左墙测得的 rho 与从 base_link 测得的 D 相减得 x、y，两个法线角相对车体坐标方向计算 yaw。每帧至少12点、跨度≥0.25m、残差≤15mm，法线垂直偏差≤2°；10~200帧全部通过才给出中位数候选，最大平移离散≤10mm、yaw离散≤1°。工程阈值不代表标定准确度。Z、roll、pitch 保持 null，不推测机械尺寸。

5. 换距离、重新摆位独立采集验证，核对尺量、正负号、车体轴向及残差。只有人工确认两墙实测且安装水平后，才使用原 phase5_laser_tf.sh 发布全部六个实测参数。当前工具不会发布 TF、修改配置、触发控制或将候选标为已验收。

自检新增已知平面外参射线模拟、坏时间戳/少帧、非法角域、非有限回波编码测试，由原 test-regions 和全量离线入口执行。真实采集和标定未验收。
