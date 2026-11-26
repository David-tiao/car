# car 开局静态方案：分段执行审查包 (SIM_ONLY)

状态：INCOMPLETE，独立审查模块，未接入真实控制链路。开发基准：David-tiao/car main 02c1cd150bfd6cd4106b1bf6519d7ee6b3d17ce1。上一轮未提交的 static_startup.py / test_fusion.py 未找到，不覆盖它们，也未运行其 48 项测试。

## 内容
- segment_executor.py：纯 Python 状态机。固定 map->odom SE(2)，接受外部已规划的 map 坐标正交路线，转换为 odom 目标，产生 SIM_ONLY 的 0x83 位置意图。每个轴段结束后必须从最新实测点重新提交路线；不把路网锚点当作真实停车位置。不实现新的 Dijkstra，不自动证明通道安全。
- segment_ros_test.py：ROS2 Humble 试验适配器，使用现有 car_interfaces 的 ChassisTelemetry、CommandFeedback、LinkStatus；仅 ROS_DOMAIN_ID=73 和 /car/test/segment/*。没有串口、实际 /cmd_vel 或真实执行端。适配器编译语法检查通过，ROS2 运行未验证。
- demo_config.json：模拟参数（包括成功状态码 0、map_to_odom、停车容差、超时），均不代表实机确认值；不能拿来控制实际小车。
- test_segment_executor.py：31 个 unittest，实际执行通过；test_results.txt 保存原审查包实际输出，上传前再次执行同一套测试，31 项通过。

## 工作流
IDLE -> READY -> WAIT_ACK -> WAIT_RESULT -> WAIT_SETTLE -> NEED_REPLAN（下一轴）/ WAIT_VISION（到达工位） -> NEED_REPLAN（QR 后前往 RAW）/ DONE（RAW 视觉确认后结束审查范围）。异常进入 FAULT；stop_required 仅表示模拟停止需求，不发送物理 STOP。

QR 阶段的聚合回执必须包含合法四段任务码及 pitch_ready=true。此标志是模拟输入，不是已实现的舵机控制或真实 OpenMV V4 回执。RAW 阶段仅代表外部视觉确认，不代表成功抓取。尚未接入上一轮的地图冻结、工位识别及安全许可；map_id 仅用于本核心内部锁定输入标识，不是地图哈希或安全认证。规划与 footprint 检查由外部提供，ROS 测试桥中的 fixture_corridors_valid 只能标记模拟夹具，不用于真实通行证明。转向不在本版本范围：各轴段固定 yaw，只做麦轮平移假设。当前严格要求路线起点等于最新实测 Pose，真实部署需通过确定的误差和安全通道策略处理停车噪声、姿态微调、重复重规划和零长度路线。通信复位/时间回绕不恢复运动，需开启新会话；后续应核验固件源时间、会话控制、消息状态码和看门狗实现。超时阈值仅模拟值。未实现急停发送、控制权申请/释放、真实命令 TX，也未验证 STM32 B1 命令接收能力。

## 离线验证（不连接硬件）
```bash
# 在 car 仓库根目录执行
cd algorithm/car_ws/src/car_rule_mission/car_segment_review
python3 -m unittest -v test_segment_executor
python3 -m py_compile segment_executor.py segment_ros_test.py test_segment_executor.py
```

## ROS2 隔离测试接口（需已有 car_interfaces 构建环境；本次未运行）
```bash
source /opt/ros/humble/setup.bash
source /path/to/car/algorithm/car_ws/install/setup.bash
export ROS_DOMAIN_ID=73
python3 segment_ros_test.py --config demo_config.json
```
不要启动硬件 bridge 或把模拟话题 remap 到真实话题。ROS 测试节点本身没有自动模拟器；需要外部测试夹具持续输入：

| 话题（/car/test/segment/ 前缀） | 类型 | 方向 |
| --- | --- | --- |
| link | car_interfaces/msg/LinkStatus | 输入 |
| telemetry | car_interfaces/msg/ChassisTelemetry | 输入 |
| feedback | car_interfaces/msg/CommandFeedback | 输入 |
| plan | std_msgs/msg/String JSON | 输入 |
| vision | std_msgs/msg/String JSON | 输入 |
| intent | std_msgs/msg/String JSON | 输出 |
| state | std_msgs/msg/String JSON | 输出 |

plan 示例（模拟：初始 odom 0,0,0，map 1,1,0）：
```json
{"mode":"SIM_ONLY","fixture_corridors_valid":true,"map_id":"demo-not-measured","stage":"QR","goal":{"x":1.5,"y":1,"yaw":0},"route":[{"x":1,"y":1,"yaw":0},{"x":1.5,"y":1,"yaw":0}]}
```
输入新鲜的 link（connected=true, heartbeat_fresh=true, peer_session_id 非零）和 telemetry（imu_valid=true、速度接近零）后提交 plan。等待 intent，使用相同 cmd_id 反馈 0x90（original_command_type=0x83）和 0x94；成功状态码采用 demo_config。再提供连续 3 条来源时间递增且晚于完成事件的停车样本（位置为目标的 odom 坐标、target_reached=true）。QR 回执示例：
```json
{"mode":"SIM_ONLY","stage":"QR","token":1,"success":true,"task_code":"156+123+516+231","pitch_ready":true}
```
下一段必须用最新 map pose 重新规划后提供 RAW plan，且保留原 map_id。完整自动模拟参见 test_two_stages 单元测试。注意这是接口联调夹具，不是比赛程序。

## 下一步
取得上一轮 car_static_startup_review.zip，确认 freeze/plan/replan 的真实数据结构及障碍物覆盖，连接规划器回调；复核固件和 ROS bridge 的控制权与 B1 双向接口，独立做 PTY 串口闭环；实测外参/停稳阈值，再进行架空轮与低速实机测试。不得跳过固件安全验证。本次只在独立审查目录中新增文件，不修改原工程代码，不启动硬件，不合并主分支。
