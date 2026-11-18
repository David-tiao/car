# 本次统一整理记录

输入：用户上传的 `algorithm.zip` 与 `radar.zip`。

1. 以 `algorithm/ydlidar_ws` 作为唯一 ROS 2 工作空间根目录。
2. 保留 algorithm 中的比赛地图、地图生成/Keepout/道路拓扑脚本、`slam_gmapping` 与 `openslam_gmapping` 源码、`task.md` 和错误记录。
3. 从 radar 中迁入已调试的 `ydlidar` 驱动源码（原目录 `fishbot_ws/src/ydlidar_ros2`，ROS 包名 `ydlidar`）、雷达启动脚本、雷达说明和两份行为树 XML。
4. 把 algorithm 原有的 `ydlidar_ros2_driver` 保存在 `reference_drivers/`，不放入 `src/`，避免两套驱动同时进入当前构建。
5. 旧的 build/install/log、嵌套包构建缓存、`.git` 和 Python 缓存未迁入；它们可重新生成。源码、地图和阶段文件保留。
6. OpenMV 未纳入本次整理。

注意：这是按上传压缩包制作的统一工程副本，不会直接改动用户电脑上的 Desktop 文件夹。
