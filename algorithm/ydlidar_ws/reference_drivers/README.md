# 备用雷达驱动源码

`ydlidar_ros2_driver/` 来自原 algorithm 工作空间。它的 ROS 包名是 `ydlidar_ros2_driver`，与当前启用的 `ydlidar` 是两套不同驱动。

为避免同一雷达被两个节点同时打开、重复发布 `/scan` 或重复发布 TF，这套驱动被移出 colcon 的 `src/`，仅作为备用源码保存。当前启用的驱动是 `../src/ydlidar_ros2/`（包名 `ydlidar`）。

如需切换，先停止当前驱动，再把选定驱动单独放入 `src/`，核对串口参数、frame_id、TF 发布方式，重新编译并测试 `/scan`。
