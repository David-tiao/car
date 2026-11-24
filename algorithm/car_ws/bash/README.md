# 工创小车统一启动入口（ROS 2 Humble）

- `bash start.sh`：桌面 V6 地图、雷达、RViz 预览，关闭窗口自动清理本实例进程
- `bash manage.sh start|stop|restart|status|doctor|logs`：生命周期管理，`doctor --offline` 只查文件，不代表驱动已运行
- `bash lidar.sh doctor`：检查当前车载雷达驱动、真实串口，不启动
- `bash lidar.sh start`：单独在前台运行 YDLIDAR X3（不要和 V6 同时启动）
- `bash competition.sh doctor|start|stop|restart|status|logs`：调用原 `competition_v1/scripts/ai.sh`，复用锁路算法

- `bash gazebo.sh doctor|start [goal]|test [goal]|status`：独立 Gazebo/RViz2/Nav2 SIM，域84本地隔离；start 仅检查就绪，test 明确执行仿真路径，日志及运行结果见 [Gazebo 验收入口](../../../docs/ROADMAP_AND_ACCEPTANCE.md#gazebo--urdf-验收独立于网页)。

**安全边界**：雷达/地图预览不控制 STM32。真实障碍物锁路需要测量的静止 TF；移动导航还需要动态定位与底盘反馈。

**项目文件**：启动入口均归入 `car_ws/bash`；算法仍在 `maps/competition_v1`，配置在 `config/x3.yaml`；运行记录与日志保留在 `~/.local/state/competition_v6_viewer/`。

**提示**：仅迁移脚本不等于雷达驱动已编译。移动 ROS 工作空间后，需通过 `bash lidar.sh doctor` 在线检查 `ydlidar_ros2_driver` 注册状态。

**可移植部署**：从脚本真实目录自动定位工作区；不依赖 Desktop/workplace。可使用 CAR_WS、CAR_ROS_SETUP 等显式环境覆盖。运行 `python3 test_portability.py -v` 检查离线路径契约；`bash repair_car_ws_lidar.sh doctor` 只检查 SDK；`build` 使用现有 SDK，缺失时不会默认下载；`--allow-sdk-build` 是单独授权的本地 SDK 构建。操作及边界见 [跨电脑部署与路径验收](../../../docs/ROADMAP_AND_ACCEPTANCE.md#跨电脑部署与路径验收)。

- `bash deploy.sh plan|doctor|build|verify|test [pc|pi5|sim]`：原生部署、指定包集合及构建验证（不自动安装依赖或启动硬件）。默认自动选择本机 pc/pi5；sim 需要匹配的 Fortress 或 Harmonic 模拟器及 ROS bridge。
- `python3 audit_lidar_artifacts.py --list`：只读列出旧雷达 Git 跟踪产物，不删除、不修改索引。参见跨电脑部署与路径验收章节。

固定场景运行验收（99fcf34）：`bash gazebo.sh test C2` 与 `bash gazebo.sh test A1` 均实际通过，分别 1/1、4/4 路段，误差 0.0387m、0.0374m，零驱动与静止里程计。[双路线运行 CI #45359460287](https://github.com/David-tiao/car/actions/runs/45359460287)。前提 Fortress/Humble；GUI 与真实镜像尚待现场验证。

`CAR_GAZEBO_BACKEND=auto|fortress|harmonic`（默认 auto）：按已加载 ROS overlay 的实际桥接库 ABI 选择、检查 Gazebo major；不匹配即拒绝，无安装/替换行为。Harmonic 需要单独配对 Humble bridge，详见操作文档及 [双后端四路线运行 CI #45384544228](https://github.com/David-tiao/car/actions/runs/45384544228)。

2c56861 实际运行验收：[四组合 CI #45384544228](https://github.com/David-tiao/car/actions/runs/45384544228) 全部 PASS。Fortress C3→C2 1/1、0.0375m；Fortress C3→A1 4/4、0.0385m；Harmonic C3→C2 1/1、0.0377m；Harmonic C3→A1 4/4、0.0382m。均实测锁定 B2-B3、B3-C3，驱动归零、静止里程计。[双架构 CI #45384544069](https://github.com/David-tiao/car/actions/runs/45384544069) 两组 PASS；[离线 CI #45384544126](https://github.com/David-tiao/car/actions/runs/45384544126) PASS（14 场景、18 锁路、12 移植及原全量回归）。无界面固定场景，不代表本机 GUI、真实镜像或实车验收。
