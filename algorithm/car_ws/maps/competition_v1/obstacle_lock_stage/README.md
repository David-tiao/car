# 工创比赛：双静态障碍物逻辑分区与锁定（实验版）

## 本次新加的代码（不覆盖原工程）

本目录不改 `topology_stage6/road_topology.py`、已生成的静态地图、原有 Nav2 参数文件或 STM32 驱动。

- `obstacle_lock_core.py`：复用第六阶段田字形 **9 个节点、12 条路段**。每段道路视为一个逻辑“大格子”，空间聚类代表独立物体。把两个可靠物体锁定到其道路格子，不因后续扫不到而解锁。不能识别的边界物体、第三个可靠目标会触发冲突，不自动放行。
- `obstacle_lock_node.py`：ROS 2 Humble 节点。从 `/scan` 经 TF `map <- laser_frame` 将距离点转换到比赛地图坐标，结合 `/map`（固定基础图）与 `/keepout_filter_mask`（规则禁行区）过滤候选物体，再输出障碍物锁定、粗略道路级路线和用于后续 Nav2 集成的规划地图。
- `test_obstacle_lock.py`：无需 ROS 的单元测试，覆盖 **2 个对象锁定 / 1 个不出发 / 3 个冲突 / 锁定后第三个冲突 / 2 个物体同一路段 / 交叉口不猜测 / 先后看到两个物体**。

## 规则要点

1. **比赛已知有两个随机的静态障碍物**，因此校验的是可靠识别到 **2 个独立物体（空间目标）**，**不是强行要求 2 条道路被封闭**。两个物体可能在同一路段：2 个物体 + 1 个封闭路段是合理状态。位于路口/格子边界时，一个物体可能影响多个路段，不能用“最多两个格子封闭”作为避障安全证明；本版对此类不明确观测给出 `CONFLICT`，不自动猜位置。
2. 为减少单帧噪声造成错误锁定，默认至少读取 20 帧有效扫描、每个物体至少在 4 帧中观察到才锁定（实际帧数随雷达频率变化）。这些数字只是保守测试默认值，不是已标定的置信度。
3. 锁定后绝不因为某次看不到物体就解除。程序重启意味着 **下一场比赛重新初始化**，锁定结果不会自动加载。`last_lock_report.json` 只是审计记录，不会自动用于下一场。
4. 如果可信观测达到 3 个对象，或持续发现路口/格子归属模糊，输出 `CONFLICT`，地图退回全障碍的保守状态，**不向机器人发送运动命令**。
5. 初始扫描可能受遮挡，只能识别当前能看到的目标。若无法确认两个对象则保持未就绪，不会假装两处都已识别。需要保证起点位置和转向扫描策略符合比赛规则。
6. 本程序不通过激光直接判断物体颜色、身份。静态环境中其它小型反射物也可能成为候选；应通过固定物理地图、允许投放位置及实测物体尺寸进一步收紧检测参数。

## 第一步：安装到你唯一主工程（仅增加文件）

将 `algorithm_obstacle_lock_overlay.zip` 放进 `~/Downloads` 后：

```bash
unzip -o ~/Downloads/algorithm_obstacle_lock_overlay.zip -d ~/Desktop
```

目标路径：

```text
~/Desktop/algorithm/ydlidar_ws/maps/competition_v1/obstacle_lock_stage/
```

## 第二步：先在 Ubuntu 上离线自测（**不需要启动雷达**）

```bash
python3 ~/Desktop/algorithm/ydlidar_ws/maps/competition_v1/obstacle_lock_stage/test_obstacle_lock.py
```

预期输出 `Ran 7 tests` + `OK`。这些是算法逻辑测试，不代表实机雷达测量准确。

## 第三步：运行 ROS 2 节点（只读取和显示）

前提：`/scan` 已发布，`/map` 和 `/keepout_filter_mask` 已分别发布 **且坐标与分辨率匹配**；实时 TF 必须能够查询 `map -> laser_frame`（依赖定位、里程计、雷达静态 TF）。以前阶段的固定测试 TF **不可用于运动中的实车**。

```bash
source /opt/ros/humble/setup.bash
python3 ~/Desktop/algorithm/ydlidar_ws/maps/competition_v1/obstacle_lock_stage/obstacle_lock_node.py \
  --ros-args -p start:=C3 -p goal:=A3
```

另一个终端观察：

```bash
ros2 topic echo /competition/obstacle_lock_status --once --qos-durability transient_local
ros2 topic echo /competition/planning_map --once --qos-durability transient_local --field info
```

**新增话题：**

| 话题 | 类型 | 用途 |
|---|---|---|
| `/competition/obstacle_lock_status` | `std_msgs/String` JSON | `COLLECTING`, `WAIT_TWO_OBSTACLES`, `LOCKED`, `CONFLICT`, `WAIT_MAP`, `WAIT_TF_OR_SCAN`; `ready` true 时才可进入下一层规划 |
| `/competition/planning_map` | `nav_msgs/OccupancyGrid` | 保留静态地图和禁行掩膜，锁定两处道路拦截带；未就绪时全图不允许规划 |
| `/competition/locked_road_path` | `nav_msgs/Path` | Dijkstra 道路中心点的粗略候选路线，**不是** Nav2 连续可行驶路径 |
| `/competition/locked_obstacle_markers` | `visualization_msgs/MarkerArray` | RViz 中标示已锁定目标的近似位置 |

在 RViz2 中显示 Map `/competition/planning_map`，Path `/competition/locked_road_path` 和 MarkerArray `/competition/locked_obstacle_markers`。

## 第四步：接入 Nav2（**先不改现有参数，单独联调**）

验证新规划地图和车体尺寸后，可以配置 **Global Costmap** 的 StaticLayer 订阅：

```yaml
# 放在 global_costmap.global_costmap.ros__parameters.static_layer 下
map_topic: /competition/planning_map
map_subscribe_transient_local: true
```

保留 `/map` 给 AMCL / 地图服务；保留现有的 `KeepoutFilter` 及局部障碍层。**不要同时为 `/competition/planning_map` 配置第二个 map_server**；该话题由本节点发布。后续应同时调整 Local Costmap 的约束/实时障碍处理，并让比赛行为树只有在 `obstacle_lock_status.ready=true` 时才发起 Nav2 任务。

> 注意：当前程序输出的是“道路大格子锁定 + 带状栅格封路”的初版。通道半宽默认 0.29m、拦截带沿道路厚度 0.13m，必须核实真实车宽、障碍物直径、道路边界、TF 精度和定位误差。不要把本程序的 `ready` 当成真实电机安全许可。**没有实车里程计/定位/STM32 控制闭环之前，只允许离线或静止测试。**

## 为什么不直接修改原有 `road_topology.py`？

当前工程仍有第六阶段 Costmap 轮询封路程序。新程序采用按**两个物体**确认并锁定，与原始 “Costmap 中一条边出现两个 lethal 栅格” 不同。为避免两种逻辑同时发布同一个话题，新程序使用不同的 `/competition/locked_road_path` 话题。旧道路节点无需启动；日后验证通过后再决定替代它。
