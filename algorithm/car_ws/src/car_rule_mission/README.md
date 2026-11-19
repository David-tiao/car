# 智能搬运赛 · 比赛路线规划（增量源码，仅放在 `car_ws/src/`）

**安装位置：** `/home/tao/Desktop/car/algorithm/car_ws/src/car_rule_mission/`

本模块只提供 2027 工创智能搬运比赛的**九站规则路线预览**，不控制电机；保留原仓库所有文件，不覆盖原有地图、Dijkstra、雷达驱动、STM32 或 OpenMV 代码。

## 复用现有逻辑

- 道路拓扑、`canonical()`、`plan_mission()`、Dijkstra：只读导入 `car_ws/maps/competition_v1/topology_stage6/road_topology.py`。
- 九站任务顺序：只读导入 `car_ws/maps/competition_v1/oneclick_stage/mission_export.py` 中的 `ZONES` 与 `STAGES`。
- 订阅 `/competition/road_status`，严格验证真实锁路的 `LOCKED_READY`、`ready`、`frozen`、`source=LIDAR_CONFIRMED`、无冲突。
- `/competition/station_poses` 仅为预留输入。**当前未实现雷达对转盘和二维码板的实际定位发布者。**

## 安装（在仓库根目录运行）

下载 `car_rule_mission_src_patch.zip` 到 `~/Downloads/` 后：

```bash
cd /home/tao/Desktop/car
unzip -l ~/Downloads/car_rule_mission_src_patch.zip
unzip -n ~/Downloads/car_rule_mission_src_patch.zip -d .
```

确认新算法只进入 `algorithm/car_ws/src/car_rule_mission/`。**不需要 `colcon build`**：当前是可直接运行的 Python 脚本（含 ROS 2 节点），并非 ament 包，没有注册为 `ros2 run` 入口。若以后需要打包为正式 ROS 2 包，也只在 `src/` 内做结构整理。

如果此前已安装旧压缩包，其三个旧 Python 文件可能仍在 `maps/competition_v1/oneclick_stage/`。新压缩包**不自动删除**它们；核实新版本通过测试后再人工清理旧副本，以免误删你们已修改的文件。

## 离线测试

```bash
cd /home/tao/Desktop/car/algorithm/car_ws
python3 src/car_rule_mission/test_rule_mission.py
python3 src/car_rule_mission/rule_mission_preview.py --demo --start-zone upper_right --block B3-C3
```

预期测试 `Ran 12 tests` 和 `OK`；演示应输出 `status: DEMO_ROUTE_ONLY`、9 个顺序节点、8 段路线，所有道路段避开 `B3-C3`。模拟数据绝不转换为真实锁路结果。

不可达测试：

```bash
python3 src/car_rule_mission/rule_mission_preview.py --demo --start-zone upper_right --block B3-C3 --block C2-C3
```

应返回 `status=NO_ROUTE`、空路线、退出码 3。

## 实际 ROS 2 路线预览（只读）

前提：真实 `/scan`、开局静态 `map -> base_link -> laser_frame` TF 标定、锁路节点均已验证；只有原始扫描预览（`RAW SCAN / NO MAP TF`）时不会有真实锁路结果。

```bash
cd /home/tao/Desktop/car/algorithm/car_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source install_oneclick_lidar/setup.bash
python3 src/car_rule_mission/rule_mission_preview.py --ros --start-zone upper_right
```

新增只读话题：

- `/competition/rule_mission_status`：规划状态及 JSON 结果
- `/competition/rule_mission_path`：`nav_msgs/Path`，RViz 可以订阅预览

未锁路保持 `WAIT_FOR_LOCK`；真实锁路后为 `PREVIEW_WAIT_STATIONS`；如果可靠工位观测也就绪则为 `PREVIEW_WITH_STATIONS`。这些状态都**不允许驱动 STM32F4**。当锁路结果冲突、无路可走、冻结后改变时，清空路径预览并停止该次规划。

## 未来工位定位输入（JSON 格式演示，数据非实测）

```json
{
  "source": "LIDAR_STATIC",
  "frame_id": "map",
  "frozen": true,
  "stations": {
    "qr_board": {"valid": true, "x_m": 2.35, "y_m": 1.20, "yaw_rad": 0.0, "confidence": 0.91},
    "material": {"valid": true, "x_m": 1.20, "y_m": 2.35, "confidence": 0.93}
  }
}
```

上面的点位和置信度**只是结构示例**；工位中心不是实际停车目标。当前筛选置信度阈值 0.70、中心至节点 0.75 m 门限都是未标定的工程初值，不能当成比赛要求。二维码板法向可能有 180° 歧义；后续要实测停车点、相机和夹爪外参、底盘运动反馈。

## 范围外

暂未实现：扫描点转盘圆拟合、二维码板线拟合、精确停车目标计算、STM32 双向命令/回执。所有输出含 `commands_enabled=false`、`NOT_FOR_NAV2_FOLLOWPATH=true`，不能直接用于真实行驶。
