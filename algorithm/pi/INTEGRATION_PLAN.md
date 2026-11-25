# Pi5 系统集成方案与实机门槛

日期：2026-11-23。目的：复用已有模块，**不动现有 `algorithm/car_ws`、`STM32F4`、`algorithm/openmv`、`sim2real` 的运行入口**。

## 1. 组件边界（现在 vs 未来）

```text
[比赛 QR/颜色/圆环]  ---> OpenMV H7 Plus --V4 USART3--> STM32F407
                            (真实视觉)             | 控制麦轮/抓取机构
                                                   |
                       STM32 USART2 ULog43 (当前：单向只读)
                                                   v
YDLIDAR X3 --/scan--> [Pi5 ROS2 Humble] <----- car_stm32_bridge /odom
                         |         |
              静态 TF/锁路/Dijkstra    设备诊断与状态
                         |         |
            路线/目标「预览」          (可选 C2 Agent -> Ubuntu Master 管理)
                         |
       Pi→STM32 控制通路【待设计/待安全验收；现在没有实体写权限】
                         |
           真实执行回执、超时、限位、急停必须由电控确认
```

- **Pi5**：传感器、ROS2 数据处理、地图/固定道路规划、系统健康；不承担电机 1ms 实时控制。
- **STM32**：唯一已明确的实体底盘/机械执行控制者，必须有自主限速、急停和掉线安全机制。
- **OpenMV**：正式 QR/六色/数字环/抓取视觉路径，当前 V4。Pi CSI 相机独立 Demo 不应与其同时争夺任务码的权威发布者。
- **C2**：授权设备健康管理（Master 在 Ubuntu，Agent 在 Pi），仅作开发/测试期辅助；比赛运行不允许人为遥控。
- **Ubuntu PC/Web**：开发、仿真、历史日志、只读实车监测；`/keyboard` 与 `/sim` 不是底盘的实体控制链。

## 2. 环境路线（不要混装）

### A. 原生方案：Ubuntu 22.04 ARM64 + ROS2 Humble

适合统一 `ros2_vision_freertos` 中的技术基线、桌面 ROS 和直接硬件访问；但更换当前 Pi5 Raspberry Pi OS 必须**先做启动盘镜像备份**，先核对板级驱动、摄像头兼容性、供电、散热和 SSH 图形桌面。

### B. 容器方案：现有 Raspberry Pi OS + Ubuntu 22.04 ARM64/Humble 容器

符合 Pi Task 中“Docker 容器”方向，暂不要求更换宿主机，但需要逐项核对：
- Pi 是否为 `aarch64/arm64`，宿主内核与 Docker 版本是否可用，存储/内存是否足够；
- ROS2 DDS 跨设备发现：Docker host 网络、`ROS_DOMAIN_ID`、RMW 实现与同网段通信；
- USB 雷达和 STM32 串口映射、设备命名、权限、组与热插拔恢复；
- CSI/Picamera2 可能依赖宿主驱动/设备映射：**相机先在宿主本机验证**，不要假定容器里开箱即用；
- 运行时日志、数据盘、启动顺序、容器退出重启、看门狗与不会误发动作；
- 实际 ROS2 包 `colcon` 构建；例如 `ydlidar_ros2_driver` 目录存在 `COLCON_IGNORE`，驱动路径要按旧工程构建流程核验。

两条方案**只选一个作为当前赛前稳定基线**。不要同时把 native 和 Docker 环境抢同一串口，ROS2 的 topic 在容器/宿主重复启动也会造成混淆。

## 3. 推荐分阶段计划

| Gate | 做什么 | 进入下一阶段的证据 |
|---|---|---|
| G0 规则/硬件 | 检查体积/单电源/车载显示/急停、固定代码 SHA 与接线图 | 实测尺寸和供电表、急停复测、版本表；没有车轮安全即不可动 |
| G1 基础远程连接 | 现有 Pi OS：安全密码、网线 SSH、热点和 VNC 可靠性；记录真实 `uname -m` 等 | 断开/重启可恢复，IP/主机名明确；仅在开发期远程操作 |
| G2 Pi 本体传感器 | 执行 `pi_healthcheck.sh`；独立 Pi CSI 读取/QR/颜色/单目估距实验 | 相机预览帧/位置与真实标尺误差记录；红绿蓝 Demo 不等于六色正式能力 |
| G3 ROS2 Humble | 选 native 或 Docker，复用现有 car_ws 包、检查 ROS2 环境及跨机 DDS | Pi5 真实 arm64 运行、依赖/包/日志和 ROS2 网络验收 |
| G4 雷达与静态规划 | 先单独 `/scan`，量安装外参，`map→base_link→laser_frame` 起点静止 TF，再锁路与 Dijkstra | 一次静止扫描有视野覆盖报告，障碍真来源/未知道路/NO_ROUTE 结果可信，真实路径**仅预览** |
| G5 真实只读电控 | STM32 ULog43→Pi ROS2 `/odom` + IMU/轮速，以真实来源暴露到 `/real` | 43 列、单位、时间戳、方向、断连离线及日志齐全，绝不造数据 |
| G6 写控制授权 | 与电控协作设计独立命令帧/序号/时限/ACK/完成/执行者唯一性/急停，先 PTY，再空载小速度 | 异常/重放/断链即安全停止的证据；仅之后允许轮子触地 |
| G7 比赛闭环 | 先一件，再三件两批，真实 QR→取料→运输→圆环→码垛→返回 | 真实过程录像、统计、位置与单轮时长；不能用网页 MOCK 成绩代替 |

## 4. 现有命令 / 文件入口（仅作为检查路径）

从仓库根目录执行：

```bash
bash algorithm/pi/pi_healthcheck.sh
bash algorithm/car_ws/bash/lidar.sh doctor
bash algorithm/car_ws/bash/test_all_offline.sh --quick
```

其中 `lidar.sh doctor` 应在所需环境准备好后检查；**诊断 PASS 不等于真实扫描 PASS**。`test_all_offline.sh` 是 PC 软件离线回归，不会验证物理轮子、UART、电池、显示器或相机标定。

以下路径为明确的复用入口：
- `car_ws/bash/lidar.sh`：X3 驱动单独运行与诊断；
- `car_ws/bash/competition.sh` 与 `maps/competition_v1/`：锁路与规划；
- `car_ws/src/car_rule_mission/rule_mission_preview.py`：只读路线预览（不是 ROS2 ament 注册包）；
- `car_ws/src/car_stm32_bridge`：ULog43 单向遥测；
- `car_ws/src/car_serial_link`：CAR-LINK 与 PTY 测试，尚未证明实际 STM32 可写；
- `sim2real/`：只读真实画面与测试模式，不变更已有入口。

## 5. 接口约定应一次定清

- 时间：采集时间（源设备时钟）与 ROS stamp 分开；明确丢包、延迟与重放处理。
- 坐标：`map` (m) / `odom` / `base_link` / `laser_frame` / `camera_frame`，正向与 yaw 统一，图像像素不是毫米。
- 消息来源：`REAL` / `TEST` / `SIM` 明确隔离；未经观测的道路不能映射为“无障碍”。
- 任务：QR 编码解析与第二批同色码垛映射保持唯一真值；`ACK` ≠ `COMPLETED`，故障时不推进状态机。
- 运动控制：所有命令由**单点仲裁**发到 STM32；速度/超时/撤销/异常/停止的优先级要先于路径规划。
- 观测：ROS2 状态应包含版本、frame、timestamp、freshness/health、设备源；GUI 的演示状态不作为执行反馈。

## 6. 本次未做的事情

未刷机、未安装 Docker/ROS2、未插拔或配置串口、未修改 STM32/相机固件、未启用电机、未声称 C2 实机连通，也未覆盖原有 ROS2 地图/驱动。所有新文档及实验代码只进入 `algorithm/pi/`，后续在有实机证据时逐项更新门槛。
