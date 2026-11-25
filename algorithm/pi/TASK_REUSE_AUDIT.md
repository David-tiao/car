# Pi5 六项任务：现有工程复用审查

检查日期：2026-11-23。任务来源：[`Hardware_mechanical/pi5/pi_task.md`](../../Hardware_mechanical/pi5/pi_task.md)。对照 `car/main` 实际目录、[`docs/PROJECT_STATUS.md`](../../docs/PROJECT_STATUS.md)、[`docs/ARCHITECTURE_AND_ALGORITHMS.md`](../../docs/ARCHITECTURE_AND_ALGORITHMS.md)、[`docs/ROADMAP_AND_ACCEPTANCE.md`](../../docs/ROADMAP_AND_ACCEPTANCE.md)；其中进度文档基线为 2026-11-22。**本次只做源码与文档审查，未接入 Pi5、OpenMV、STM32 或雷达真机，没有新增实机 PASS。**

## 任务与复用决策

| Pi Task | 已发现的可复用资源 | 当前能力 / 不应误判之处 | Pi 专属下一步 | 实机验收 |
|---|---|---|---|---|
| 1. C2 连通 | 私有仓库 [`David-tiao/C2_schooling`](https://github.com/David-tiao/C2_schooling) V1.20，含 Ubuntu Master、ARM64 Agent、只读监控、配对/撤销与 ARM 验收手册 | Ubuntu 本机部分软件回归已有历史结果；**Pi 真机接入未验收**；C2 不是比赛遥控通道 | 用 Ubuntu 电脑作为 Master，Pi5 接入授权 Agent；先读 `docs/guides/PI_ORANGE_MONITORING.md` | Agent 身份配对、TLS/心跳、CPU/内存/磁盘显示、断线/撤销可观察；正式比赛禁用任何人工远控 |
| 2. QR 识别与显示屏 | [`algorithm/openmv/openmv_all/`](../openmv/openmv_all/) 四文件相机项目；[`STM32F4/Hardware/vision.c`](../../STM32F4/Hardware/vision.c) V4；本目录 `pi5_qr_material_vision.py` 可做 Pi CSI 独立预览 | OpenMV 有二维码识别/三帧确认和任务码解析逻辑；Pi 脚本只读取并显示**文本**，无比赛格式校验、无车载 LED 字高验收；**相机实装测试未知** | 确定正式任务码唯一来源，优先复用 OpenMV→STM32；Pi 备用扫描先独立做图片/屏幕/反光/角度测试；补正确任务码车载显示 | 80×80mm 实体二维码板不同摆位读码；`156+123+516+231` 等合法/非法输入验证；车载亮屏可显示任务码与抓放计数，字高≥12mm |
| 3. 物料识别与测距 | [`algorithm/openmv/openmv_all/config.py`](../openmv/openmv_all/config.py) / `tools.py`：6 色 LAB、圆形/盘面、跟踪；本目录 `pi5_color_distance.py` 单目估距演示 | OpenMV 视觉代码已有六色识别，但灯光、转盘、外参与抓取像素未标定；Pi 的 HSV 演示**只覆盖红绿蓝**，基于表观宽度推算距离，对朝向/尺度/遮挡敏感；并非深度相机或神经网络 | 先建立六色 × 位置 × 光照 × 角度的带真值照片集；做相机内外参、色彩与目标形状校准；距离方法需独立量尺评估 | 六种颜色及两批目标识别混淆矩阵；错误拒绝；不同距离的估距误差分布；抓取对位重复性，而非只看窗口画框 |
| 4. 部署 ROS2 Humble（Docker） | [`algorithm/car_ws/src/`](../car_ws/src/) 多个 ament/ROS2 包；[`ros2_vision_freertos`](https://github.com/David-tiao/ros2_vision_freertos) 统一 Ubuntu 22.04 + ROS2 Humble 环境说明；已有个人保存的 Pi5 Humble 离线部署脚本仅供参考 | 仓库有源码**不代表 Pi5 已安装**；目前 Pi 文档记载为亚博智能预装 Raspberry Pi OS，不是 Ubuntu 22.04；Docker 与 native apt 是两种路径 | 在 Pi5 先确认 `aarch64`、OS、内存/存储、相机/串口设备；若采用 Docker，选 ARM64 Ubuntu22.04/Humble 并评估 `--network host`、DDS、设备映射、权限、持久化、容器退出清理；不要照搬 x86 镜像 | Pi5 上 `ros2 doctor` / `ros2 topic list`、构建所有需要的包、跨机 DDS、断线与开机恢复；锁定镜像 digest 与代码 SHA |
| 5. 运行雷达与规划路径 | `car_ws/src/ydlidar_ros2_driver`、`car_ws/config/x3.yaml`、`car_ws/bash/lidar.sh` 与 `competition.sh`、`car_ws/maps/competition_v1/` 锁路拓扑与 Dijkstra、`car_rule_mission` 预览 | 已有驱动/地图/离线预览；新工作区真实 `/scan`、`laser_frame→base_link→map`、随机障碍路段封锁、真实停车点仍需现场验证。驱动目录含 `COLCON_IGNORE`，不要误以为源码存在就能被 colcon 自动编译 | 先验证真实扫描和 TF，再验证盲区/路段状态；保留开局静止扫描→可靠锁路→固定 Dijkstra 路线思路，不另起导航工作空间 | `/scan` 连续与时间戳、障碍已/未知/封锁判据、`LOCKED_READY` 真来源、无路 `NO_ROUTE`、连续几次实测与安全停机 |
| 6. 与下位机通信 | [`通信协议.md`](../../通信协议.md)、`car_ws/src/car_stm32_bridge` ULog43 / `/odom`、`car_ws/src/car_serial_link` CAR-LINK 模拟测试与 `STM32F4/` | 目前 STM32 USART2→Pi ULog43 **仅单向只读**；CAR-LINK PTY 和 ACK 模拟不是实体双向控制；OpenMV↔STM32 的 V4 UART3 与 Pi 链路不可混发 | 首先完成真实串口只读桥并核验 43 列与单位；再与电控共同确定**独立** Pi→STM32 写协议、物理 UART、唯一控制权、限速/时效/ACK/完成/急停 | 先真实只读 `/odom` 可核验；之后独立上线 ACK、重复帧、超时/断线停机、车轮离地小速度测试；**未完成前严禁实体控制** |

## 现有项目主要模块（避免造第二套）

- **STM32F4**：`Control/mecanum_chassis.c` 麦轮正逆运动学、`Control/Motor.c` 四轮环、`App/` 比赛/抓取 FSM、`Hardware/vision.c` V4 与 `bsp_ulog.c` ULog43。
- **OpenMV**：`algorithm/openmv/openmv_all/main.py/config.py/tools.py/serial.py` 正式硬件视觉链，当前配置优先 V4，不自动把旧 V5 25 步视为已适配。
- **Pi / ROS2**：`car_ws/src/car_stm32_bridge`、`car_navigation`、`car_rule_mission`、`ydlidar_ros2_driver` 等；赛道物理占据图与规则 keepout 需要分层。
- **Web**：`sim2real/` 下 `/sim`、`/test`、`/real`、`/keyboard` 四模式；`/real` 原则上只能用真机来源。
- **机械**：第五轴 `column_yaw_deg` 目前只在网页 MOCK 展示，真实驱动/极限位需确认。
- **C2**：面向授权设备管理、设备健康监测，不是比赛控制闭环的替代品。

## P0 优先项（按依赖顺序）

- [ ] **安全与规则**：车体出发前尺寸 ≤300×300×400mm；车载显示字高≥12mm；单个车载电源、向下补光、车道边界、急停/电源安全、无人为遥控。
- [ ] **Pi5 可用**：安全账号、SSH 稳定、必要时 VNC、硬件/架构/存储确认、摄像头与串口清单。
- [ ] **C2 第一台 Pi**：只读监测与断线恢复；确认比赛时不触发非自主操作。
- [ ] **二维码最小真实闭环**：OpenMV→STM32→车载屏的合法任务码；录制异常帧/非法码日志。
- [ ] **雷达最小真实闭环**：Pi5 Humble→YDLIDAR `/scan`→静止 TF→障碍锁路→Dijkstra，明确未观测道路为未知。
- [ ] **STM32 遥测**：实际 USART2→Pi5→`/odom` 连续且时间/坐标可信。
- [ ] **安全控制协议后置**：与电控定稿、模拟负向测试、实物急停和单动作授权后，才允许 Pi 向底盘发送运动请求。
- [ ] **单件实物验收**：扫描→确认物料→抓取到车身→放到指定环→反馈→统计；再扩展两批与返航。

## 特别提醒

1. `car/README.md` 目前含 `<<<<<<< HEAD` 等合并冲突标记，需**单独**清理（本目录提交不修改它）。
2. 2026-11-21 的 CI 及其他主机单测属于软件证据，不能冒充 2026-11-23 的 Pi5 实机验收。
3. Pi5 预装镜像资料中曾出现默认密码/热点口令。应更换，不复制口令到新的教程或日志。
4. 旧文档可能出现 UART4 / V5 或时间、坐标单位的不同说法；以现场固件 SHA、当前源码和双方实测为准。
