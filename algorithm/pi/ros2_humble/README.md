# Raspberry Pi 5 部署 ROS 2 Humble（Docker）和 STM32 只读桥

> 代码与说明全部位于 `algorithm/pi/ros2_humble/`，不修改已完成的 C2、QR、YOLO 或 STM32 固件。先复用 `algorithm/car_ws` 中已经存在的 C++ `car_stm32_bridge`。**这里提供部署工具，不代表已在你的 Pi5 上运行或连过 STM32 实机。**

## 为什么首选 Docker？

已有硬件资料 `Hardware_mechanical/pi5/pi5.md` 记录 Pi5 **原先使用过 Raspberry Pi OS**，计划未来更换 Ubuntu 22.04。实际系统和内核还需 `doctor.sh` 实时核对，不能凭历史文档推断今天的状态。

**ROS 2 Humble 官方二进制包对应 Ubuntu 22.04 Jammy，ARM64 可用**。Pi5 现有 Raspberry Pi OS（Debian）不宜直接添加 ROS Humble 的 Ubuntu apt 源；从 Docker 官方 ROS 镜像 `ros:humble-ros-base-jammy` 可以把 Jammy+ROS 环境隔离在容器里，同时保留宿主机 rpicam、CSI 和桌面显示驱动。

- ROS 2 Humble [Ubuntu Deb 安装官方文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- ROS 2 官方 [Raspberry Pi 安装指南](https://docs.ros.org/en/humble/How-To-Guides/Installing-on-Raspberry-Pi.html)
- Docker [ROS 官方镜像](https://hub.docker.com/_/ros)：`humble-ros-base-jammy` 提供 ARM64。
- 如果你的 Pi5 **已是有良好驱动支持的 Ubuntu 22.04 ARM64 系统**，也可以遵循上述官方安装文档直接原生安装 Humble，然后复用下文的 `colcon build` + `ros2 launch`，不用改车体源码。不要未经确认重刷目前可用的系统。

## 本目录内容

```text
algorithm/pi/ros2_humble/
├── Dockerfile        # Ubuntu Jammy / ROS Humble / 编译依赖
├── run.sh            # image / build / test / replay / monitor / echo-odom 等
├── doctor.sh         # 安全只读检查 OS、CPU、Docker、TTY
├── README.md         # 此部署指南
└── STM32_LINK.md     # 真实 ULog43 链路、接线与双向协议规划
```

## 0. 树莓派前提条件

1. **树莓派 5 的 64 位 OS**，能够正常 SSH 登录，已备份系统卡，供电和剩余磁盘空间足够（建议至少数 GB，完整 workspace 依赖可能更多）。这套 Dockerfile 只部署 ROS；不修改 Pi5 内核、启动设置或 CSI 驱动。
2. Linux Docker daemon 已正确安装和启用。Raspberry Pi OS 在系统对应 apt 仓库提供 `docker.io` 时，可以手动考虑 `sudo apt update && sudo apt install docker.io`，然后检查 `sudo docker info`。**不要在未确认系统前直接执行网络 pipe-to-shell 安装脚本**。需要按系统版本查阅 [Docker Engine Debian 安装文档](https://docs.docker.com/engine/install/debian/)。
3. 本工具以普通用户执行 `docker`；如果用户没有 Docker 权限，先由管理员处理。**Docker daemon 权限近似 root 权限**；允许某用户进入 docker 组属于安全决定，本脚本不替你改 sudoers/用户组。
4. 下载镜像和 apt 依赖需要联网。离线赛场先把镜像和 named volume 编译结果准备好，再断网验证。
5. 项目仓库完整 checkout，包含 `algorithm/car_ws/src/{car_interfaces,car_stm32_bridge,car_debug,car_bringup}`。如果你从本次连续开发 PR 的分支检出，建议优先合并审查通过的 PR 到 main，再部署稳定版。

## 1. 检查环境（**只读，不需要 Docker 运行**）

在仓库根目录：

```bash
bash algorithm/pi/ros2_humble/run.sh doctor
```

查看 `OS`、`Machine architecture`、Docker daemon，以及 `/dev/serial/by-id/` 和 `/dev/ttyUSB*`。运行时不会打开串口、执行 GPIO 操作或重置系统。

## 2. 构建容器及 ROS2 C++ 工作区

```bash
# 在 Raspberry Pi OS 主机执行，Docker 自动选 linux/arm64 镜像
bash algorithm/pi/ros2_humble/run.sh image

# 只编译现有 car_bringup 所需的 package 依赖（含接口、C++ ULog43）
bash algorithm/pi/ros2_humble/run.sh build

# 运行当前 package 的 colcon test
bash algorithm/pi/ros2_humble/run.sh test
```

构建脚本使用 `algorithm/car_ws/src:ro` 只读 bind mount，**不会改动现有 C++ 工程**；build/install/log 进入命名 Docker volume `car_pi5_ros2_humble_out`。镜像名默认 `car-pi5-ros2-humble:local`。源码变更后须重新 `run.sh build`，容器临时启动结束也不会删除 build volume。

出于安全考虑，不使用 `--privileged`、不自动建立开机服务、不默认开放 `/dev`、不向 STM32 发数据。默认 `ROS_DOMAIN_ID=42`、`ROS_LOCALHOST_ONLY=1`（仅本机 ROS discovery），避免无意加入其它网络上的机器人 DDS 域。

如果需在同一台 Pi5 上开两个终端监视 ROS2 的 topic，两端都应使用同一 ROS domain；`run.sh` 给所有临时容器传同样的环境变量和 `--network host --ipc host`，以便容器间 DDS 发现。

## 3. 第一关：合成 ULog43 → ROS2（**无需 STM32 硬件**）

终端 A：

```bash
bash algorithm/pi/ros2_humble/run.sh replay
```

这会启动现有 `car_bringup` 的 `offline_debug.launch.xml`，运行模拟 ULog43 CSV 回放和 C++ 监控，不映射任何串口。

终端 B（另开 SSH）：

```bash
bash algorithm/pi/ros2_humble/run.sh topics
bash algorithm/pi/ros2_humble/run.sh echo-odom
```

**注意：**回放数据文件有结束位置；第二终端可能错过已回放的帧。可以保持终端 B 预先执行 `echo-odom`，再在 A 启动回放；或在容器 shell 中改用较长间隔和重新启动。这里的 /odom 是**合成数据**，不得当作真车定位。

验收项：`/odom`、`/car/chassis/telemetry`、`/car/chassis/wheels`、`/car/debug/ulog43`；报文类型、频率和诊断状态符合 `car_interfaces`，没有控制主题向电机发送任何命令。

## 4. 第二关：STM32F4 → Pi5 真串口，只读

先看 [STM32 链路与接线](STM32_LINK.md)。目前的真实 ULog43 发送口是 **STM32F407 USART2 PD5 TX (115200/8N1)**，不是 OpenMV 的 USART3；用 **3.3V TTL USB-UART** 的接收端（RX）+ 共地做第一轮监听。确认线路实际已接到/已启用的 microSD UART 日志口，确保电气安全。

```bash
bash algorithm/pi/ros2_humble/run.sh doctor
# 用 doctor 输出的实际 USB-UART 设备名替换:
bash algorithm/pi/ros2_humble/run.sh monitor /dev/serial/by-id/YOUR-ADAPTER
```

再开另一个终端：

```bash
bash algorithm/pi/ros2_humble/run.sh echo-odom
```

桥节点 C++ 源码通过 `O_RDONLY` 打开 `/dev/car_ulog`，`run.sh monitor` 只给 Docker 这一台设备的**读权限**；不使用 `--privileged`，不会向 STM32 发送串口命令。完成后按 Ctrl+C 停止。

**设备名注意**：`/dev/ttyUSB0`、`/dev/ttyACM0` 和 `/dev/serial/by-id/xxx` 都可能出现，具体以真正接入的 USB-TTL 型号为准；雷达和 STM32 USB 串口也可能同时出现，必须通过拔插或 `udevadm info` 逐个确认身份。不允许直接盲接错误端口。

## 5. 进阶调试

```bash
# 容器内交互式 ROS 环境，不映射 STM32 设备
bash algorithm/pi/ros2_humble/run.sh shell

# 进入容器后
ros2 pkg list | grep -E '^car_'
ros2 topic list -t
ros2 node list
ros2 topic hz /odom
ros2 topic echo /odom --once
```

如需同一局域网另一台 PC 订阅，**不要直接默认改成 ROS_LOCALHOST_ONLY=0**：需要先决定 ROS_DOMAIN_ID、DDS 实现、PC/Pi 双方网络接口、网络安全策略、防火墙，以及是不是比赛允许的外部通信。当前默认只在 Pi5 内交换数据。

### 常见故障定位

| 现象 | 优先检查 |
| --- | --- |
| `docker: permission denied` | daemon 是否在运行、当前用户的 Docker 授权；不要简单把容器开放为 `--privileged` |
| `colcon` 找不到 `car_interfaces` | 是否完整拉取 `algorithm/car_ws/src`，检查 `--packages-up-to car_bringup` 的源代码依赖 |
| `ros2 launch` 找不到包 | 先 `run.sh build`，确认同一个 `PI_ROS_VOLUME`，不能在执行构建前直接监视 |
| `/odom` 不出现 | 桥是否找到真实串口、是否读到 43 列数字、baud、实际 STM32 USART2 是否发送、USB-UART RX 是否接 PD5 TX；不能用 OpenMV V4 数据代替 ULog43 |
| `/odom` 在回放出现但真串口没有 | 软件回放成功不证明硬件串口；检查地线、电平、USB 端口归属和原 microSD 日志是否占用 |
| `/odom` 突然不更新 | STM32 断线/停发、异常行、源时间倒退；现有桥不会自动虚构最新的 odom |
| ROS2 能收到信息但电机不动 | **正常**；这是只读 ULog43 监视桥，不是 Pi→STM32 电机执行器 |

## 6. 后续任务，不提前声称验收

- [ ] 在真实 Pi5 上 `doctor`、Docker build、colcon/launch 成功。
- [ ] 复用回放文件确认 C++ 43 列解析、真实 message 类型和 /odom。
- [ ] 真实 STM32F407 USART2 只读 UART 串口接线和 10 分钟持续观测。
- [ ] 对照实际编码器、轮速、姿态/方向实测，复核 `pulses_per_meter` 等软件参数；`publish_tf=false` 默认保持，统一 TF 只留一个权威发布源。
- [ ] 与雷达 `/scan`、Nav2/路径规划保持分层；真实底盘可写链尚未实现，暂不把 /cmd_vel 接到任何串口。
- [ ] 明确电控空余的全双工 UART/USB CDC、DMA/ISR 所有权和急停/授权机制后再开发可写协议与执行回执（见 `STM32_LINK.md`）。

**结论**：安装的是 ROS 2 **Humble**；读取/发布的是现有 **STM32F4 ULog43** 真遥测（硬件连接后）；不是 ROS 2 控制 STM32 电机的完整实车闭环。现有的 C2 通信不受影响。
