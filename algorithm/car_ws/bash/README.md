# car_ws/bash — 统一 ROS2 脚本入口

所有**新建和日常使用的 Shell 管理脚本**统一放在 `algorithm/car_ws/bash/`，避免再从 `~/Downloads/...` 执行。脚本根据自身路径自动识别 `car_ws`，也可通过环境变量 `CAR_WS` 指定其他工作空间。STM32/OpenMV 固件、ROS2 C++ 源码和 XML Launch 文件均不移动、不修改。

> 本包只新增 `bash/`，不会覆盖 `src/`，不会进行 Git 操作，不会向真实 STM32 发送运动指令。已有历史地图目录中的 `.sh` 可能依赖其相对目录结构，**不会自动移动**，以免破坏旧地图工具；用 `inventory.sh` 可列出它们，后续完成依赖重构后再集中。

## 日常命令

在你的 Ubuntu 上（已安装 ROS2 Humble 且已编译工程）：

| 文件 | 用途 | 命令 |
|---|---|---|
| `env.sh` | **source** 加载 ROS2 环境 | `source ~/Desktop/car/algorithm/car_ws/bash/env.sh` |
| `build.sh` | 编译自研 C++ 包 | `bash ~/Desktop/car/algorithm/car_ws/bash/build.sh core` |
| `build.sh` | 仅编译串口功能包 | `bash ~/Desktop/car/algorithm/car_ws/bash/build.sh serial` |
| `build.sh` | 编译工作空间全部可发现包 | `bash ~/Desktop/car/algorithm/car_ws/bash/build.sh all` |
| `start_serial_demo.sh` | **模拟 STM32 二进制串口** + ROS2 + 诊断 | `bash ~/Desktop/car/algorithm/car_ws/bash/start_serial_demo.sh` |
| `smoke_serial.sh` | 一键测试模拟串口到 ROS2 的 9 个话题 | `bash ~/Desktop/car/algorithm/car_ws/bash/smoke_serial.sh` |
| `start_ulog_demo.sh` | 模拟 CSV 回放 + ROS2 + 诊断 | `bash ~/Desktop/car/algorithm/car_ws/bash/start_ulog_demo.sh` |
| `smoke_ulog.sh` | 旧 ULog43 离线回归测试 | `bash ~/Desktop/car/algorithm/car_ws/bash/smoke_ulog.sh` |
| `check_topics.sh` | 查看节点与话题，检查实时数据 | `bash ~/Desktop/car/algorithm/car_ws/bash/check_topics.sh serial` |
| `test_codec.sh` | 运行纯 C++ 协议单元测试 | `bash ~/Desktop/car/algorithm/car_ws/bash/test_codec.sh` |
| `start_live_readonly.sh` | 未来接串口时**只读**连接 | `bash ~/Desktop/car/algorithm/car_ws/bash/start_live_readonly.sh /dev/ttyUSB0` |
| `install_phase.sh` | 从旧 ZIP 安装未安装的阶段包（已有包不覆盖） | `bash ~/Desktop/car/algorithm/car_ws/bash/install_phase.sh serial` |
| `inventory.sh` | 查看已有 `.sh` 脚本分布 | `bash ~/Desktop/car/algorithm/car_ws/bash/inventory.sh` |

## 使用顺序

1. `bash ~/Desktop/car/algorithm/car_ws/bash/build.sh core`
2. `bash ~/Desktop/car/algorithm/car_ws/bash/smoke_serial.sh`（确保没有同时手动运行相同离线 demo，避免话题重复发布）。
3. `bash ~/Desktop/car/algorithm/car_ws/bash/start_serial_demo.sh`（保持运行）。
4. 另开终端：`bash ~/Desktop/car/algorithm/car_ws/bash/check_topics.sh serial`
5. 可再测试 ULog43，但**必须先停止串口 demo**（避免重复发布 `/odom`）。

## 关键区别

- `start_serial_demo.sh` 和 `smoke_serial.sh`：真正经过 Linux 虚拟串口和 CAR-LINK 二进制协议，模拟 STM32，**不需要小车**。
- `start_ulog_demo.sh` 和 `smoke_ulog.sh`：只读历史 CSV 模拟回放，**不需要小车**。
- `start_live_readonly.sh`：保留将来使用的连接入口；目前 STM32 还未实现 CAR-LINK 固件，不能直接连接就期待有数据。
- `env.sh` 必须执行 `source` 而不是 `bash`，否则不会改变当前终端环境。
- 运行中的 Launch 要用 `Ctrl+C` 正常结束。

## 仓库管理

推荐将 `bash/` 纳入 Git，并在之后新增脚本时放在同一目录。现有地图工具脚本和包内测试辅助脚本不强行改位置；如需要物理迁移，必须先核对相对路径和引用方，再做一次迁移提交。
