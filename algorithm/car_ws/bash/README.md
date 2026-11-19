# 工创小车统一启动入口（ROS 2 Humble）

- `bash start.sh`：桌面 V6 地图、雷达、RViz 预览，关闭窗口自动清理本实例进程
- `bash manage.sh start|stop|restart|status|doctor|logs`：生命周期管理，`doctor --offline` 只查文件，不代表驱动已运行
- `bash lidar.sh doctor`：检查当前车载雷达驱动、真实串口，不启动
- `bash lidar.sh start`：单独在前台运行 YDLIDAR X3（不要和 V6 同时启动）
- `bash competition.sh doctor|start|stop|restart|status|logs`：调用原 `competition_v1/scripts/ai.sh`，复用锁路算法

**安全边界**：雷达/地图预览不控制 STM32。真实障碍物锁路需要测量的静止 TF；移动导航还需要动态定位与底盘反馈。

**项目文件**：启动入口均归入 `car_ws/bash`；算法仍在 `maps/competition_v1`，配置在 `config/x3.yaml`；运行记录与日志保留在 `~/.local/state/competition_v6_viewer/`。

**提示**：仅迁移脚本不等于雷达驱动已编译。移动 ROS 工作空间后，需通过 `bash lidar.sh doctor` 在线检查 `ydlidar_ros2_driver` 注册状态。
