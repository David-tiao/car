# 二维码板 / 原料盘区域雷达扫描（只读实验）

本模块复用 X3 的 /scan，不接管串口、不发布 TF 或 /cmd_vel，不改变原障碍聚类、道路锁定、Nav2 或 STM32。
比赛规则图1与现有 zones.json 给出的搜索先验：地图左下为 (0,0)，右侧二维码板中心 y 约 1.1~1.3m；上侧原料盘中心 x 约 1.1~1.3m、名义 y=2.4m，半径 .15m。区域边界不是现场测量结果。二维码 A4 横放，不能把 80mm 二维码的尺寸当作雷达板宽。

## 启动

在 car/algorithm/car_ws 下，确保同一 ROS_DOMAIN_ID、同一时间源、系统 Python，无 Conda/venv。若 V6 或其他程序已经运行雷达，不重复启动。

终端 A（仅在没有现有驱动时）：
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

std_msgs/String JSON，正常 status=OBSERVATION_ONLY，含 scan stamp_ns 和 qr_board、raw_disk。
- NO_RETURN：区域内无有效回波；不代表物体不存在。
- RETURN_ONLY：有回波，几何不满足条件。nearest_region_return_m 为车体原点到最近可见区域回波的平面距离，非目标确证距离。
- GEOMETRY_CANDIDATE：一个几何候选；AMBIGUOUS：多个候选；不等于身份确认。
- candidates: center_map_m、center_base_m、center_distance_m、bearing_rad、nearest_candidate_return_m、rms_m、point_count。二维码候选还有 observed_span_m（观测线段长度）、tangent_yaw_map_rad（无向直线方向）；原料盘候选有 radius_m 与 arc_rad。
- 二维码候选中心是观测线段均值，不一定是实体板中心。盘心为拟合估计，不是盘沿最近距离。
- INVALID_SCAN / NONMONOTONIC_SCAN / TF_OR_SCAN_STALE / SCAN_STALE / INVALID_GEOMETRY：无可用目标结果，消费者必须丢弃旧数据并自行检查时间戳，不可据此控制车。
- 数据无视觉确认、无遮挡保证、时间连续稳定门控或停车控制授权。不能据此解除原障碍锁定或自动发车。

## 验收

```bash
python3 maps/competition_v1/lidar_stage5/scan_regions.py --self-test
```

内置纯 Python 合成测试覆盖直线、圆弧、退化、歧义、空区域、坐标转换、无效扫描。需要在实际环境运行；测试不等同实机验收。

实车依次：确认雷达安装高度和水平姿态；测量真实外参、地图静止位姿；量尺对照不同距离和随机目标位置，记录回波数、候选数、距离误差及误识别；遮挡、断开扫描、去掉 TF、用墙面或其它圆形物体作负例；确认现有道路锁定和控制接口无回归。若扫描平面错过板面/盘体，不改阈值冒充成功。只有在视觉身份确认、连续帧可靠性和实际停车误差验收后，才考虑接入运动闭环。
