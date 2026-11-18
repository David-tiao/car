# 工创智能搬运小车 ROS 2 主工程（统一版）

本目录是统一后的主工程。后续雷达、地图、道路拓扑、Nav2 行为树和 ROS 2 算法都放在这里。OpenMV 文件夹暂时保持独立，不在本次整理范围内。

## 统一路径

```text
~/Desktop/algorithm/
├── README.md
├── task.md                         # 主项目接口与阶段状态交接文档
└── ydlidar_ws/                     # 唯一 ROS 2 colcon 工作空间
    ├── src/
    │   ├── ydlidar_ros2/            # 当前启用的雷达驱动，ROS 包名 ydlidar
    │   ├── slam_gmapping/            # GMapping ROS 2 包源码
    │   └── openslam_gmapping/        # GMapping 算法库源码
    ├── maps/                        # 比赛地图、禁行掩膜、道路拓扑及阶段文件
    ├── behavior_trees/              # Nav2 默认树和比赛导航树 XML
    ├── config/                      # 参考配置
    ├── reference_drivers/           # 未启用的备用厂商驱动源码
    ├── docs/                        # 雷达指南、历史架构笔记和错误记录
    └── ydlidar.sh                   # 统一路径下的雷达构建/启动/检查脚本
```

## 雷达驱动选择（重要）

当前 `src/` 中只保留一个启用的雷达驱动：

- 源码目录：`ydlidar_ws/src/ydlidar_ros2/`
- ROS 2 包名：`ydlidar`
- 启动文件：`ydlidar_launch.py`
- 数据话题：`/scan`
- 配置文件：`ydlidar_ws/src/ydlidar_ros2/params/ydlidar.yaml`

选择它的原因：旧 `radar/fishbot_ws` 中这个包此前已经成功编译，且项目调试记录中已收到 `/scan`，`frame_id` 为 `laser_frame`。

`ydlidar_ros2_driver` 是 algorithm 原工程中的另一套厂商驱动源码，现放在 `ydlidar_ws/reference_drivers/`，不在 `src/` 下，不会被 colcon 自动编译。不要同时启动两套驱动，以免重复发布 `/scan` 或 TF。若以后决定切换驱动，再单独迁移并验证。

## 第一次使用

```bash
cd ~/Desktop/algorithm/ydlidar_ws
source /opt/ros/humble/setup.bash
chmod +x ydlidar.sh
./ydlidar.sh build
```

编译完成后启动雷达：

```bash
./ydlidar.sh start
```

另开终端检查：

```bash
cd ~/Desktop/algorithm/ydlidar_ws
./ydlidar.sh check
```

RViz2：

```bash
./ydlidar.sh rviz
```

## 行为树文件

- `behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`：Nav2 原始默认导航树，保留作为参考。
- `behavior_trees/competition_nav_to_pose.xml`：此前拟定的保守比赛导航树，暂不执行任何 Spin/BackUp 主动移动恢复。

注意：把 XML 放进目录不代表 Nav2 已经使用它。仍需在实际 Nav2 参数文件的 `bt_navigator.ros__parameters.default_nav_to_pose_bt_xml` 中指定该文件，并确认所需 Nav2 BT 插件已加载。当前工程尚未自动改写用户机器上的 Nav2 参数。

## 地图与比赛算法

`maps/competition_v1/` 保存比赛静态地图、Keepout 掩膜、启停区、道路拓扑及阶段性实现。目录中的阶段说明和 README 应先阅读；不要把示例/阶段脚本当作已经完成的完整 Nav2 运动闭环。

`task.md` 明确记录了当前状态：雷达、地图、道路拓扑和规划路径有已实现/部分验证内容；STM32F4 速度闭环、编码器里程计及真实运动反馈仍需按实际硬件完成。

## 构建注意事项

- 本次整理移除了旧的 `build/`、`install/`、`log/`、`build_x3/`、`install_x3/`、Git 元数据和 Python 缓存。这些是可再生成的构建产物，不是源码。
- 需要在新路径下重新编译。首次先只编译已验证的雷达包：`./ydlidar.sh build`。不要一上来就全工作空间编译。
- GMapping 包源码保留，但它是否作为最终 SLAM 方案，仍需结合现有 TF、里程计和比赛需求决定。
- `config/slam_toolbox_mapper_reference.yaml` 是旧配置参考，里面 `base_frame` 当前为 `base_footprint`；使用前必须与实际 TF 树核对，不要直接当作已验证配置。
