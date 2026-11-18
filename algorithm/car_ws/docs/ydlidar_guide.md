# YDLIDAR X3 启动指南（统一工作空间）

系统：Ubuntu 22.04 + ROS 2 Humble

统一工作空间：`~/Desktop/algorithm/ydlidar_ws`

## 1. 编译已验证的雷达驱动

```bash
cd ~/Desktop/algorithm/ydlidar_ws
source /opt/ros/humble/setup.bash
chmod +x ydlidar.sh
./ydlidar.sh build
```

## 2. 启动雷达

```bash
cd ~/Desktop/algorithm/ydlidar_ws
./ydlidar.sh start
```

保持该终端运行。新开终端检查数据：

```bash
cd ~/Desktop/algorithm/ydlidar_ws
./ydlidar.sh check
```

## 3. RViz2 查看

```bash
cd ~/Desktop/algorithm/ydlidar_ws
./ydlidar.sh rviz
```

RViz2 中设置 Fixed Frame 为 `laser_frame`，添加 LaserScan，Topic 选择 `/scan`。

## 4. 重要说明

- 当前统一工程选用 `src/ydlidar_ros2`，其 ROS 包名为 `ydlidar`，因为它对应之前已成功编译并收到 `/scan` 的驱动。
- 另一个厂商驱动 `ydlidar_ros2_driver` 放在 `reference_drivers/`，不参与当前 colcon 构建。不要把它复制进 `src/` 与 `ydlidar` 同时启动。
- 雷达参数文件位于 `src/ydlidar_ros2/params/ydlidar.yaml`。端口、波特率、frame_id 必须按实机检查。
- 驱动 launch 文件还会发布 `base_link -> laser_frame` 静态 TF；其平移参数需按雷达实际安装位置校准。
