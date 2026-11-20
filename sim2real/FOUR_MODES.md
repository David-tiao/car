# 工创 Sim2Real 四运行集 · 安装与验收

本补丁保留项目根目录的 `sim2real/` 和原有 `car` 快捷命令，**不修改 STM32/OpenMV 固件**。测试和实车通过独立 HTTP 缓存隔离，网页保留原地图、路径算法和诊断内容。新增的页面路由为：

| 页面 | 地址 | 数据来源 | 控制权限 |
|---|---|---|---|
| 纯模拟集 | `http://127.0.0.1:8080/sim` | 本地 JS 仿真算法 | 仅浏览器内部模拟 |
| 测试集 | `http://127.0.0.1:8080/test` | 模拟 MCU → Linux PTY → ROS 2 → HTTP | 只读 |
| 运行集 | `http://127.0.0.1:8080/real` | 实际 STM32 → 串口 → ROS 2 → HTTP | 只读 |
| 映射反馈 | `http://127.0.0.1:8080/keyboard` | 电脑 ROS2 `/cmd_vel` → 网页估算轨迹 | **只读**；不控制 STM32 |

原 `car` 命令仍在 8080 端口启动独立网站；首页 `/` 等价于纯模拟模式，四模式仍从原网页导航切换。`/sim`、`/test`、`/real` 复用既有前端；`/keyboard` 使用独立的极简反馈页，不加载任务规划、地图限位或状态机。

## 1. 安装后，在 Ubuntu 上验收

```bash
cd ~/Desktop/car
bash algorithm/car_ws/bash/web.sh doctor
cd sim2real
python3 -m unittest discover -s tests -p 'test_four_modes.py' -v
# 安装了 Node.js 时额外验证四个前端入口
node tests/test_mode_router.js
```

预期：doctor 打印各项检查通过；原四模式测试通过，另执行 `python3 -m unittest discover -s sim2real/tests -p 'test_keyboard_feedback.py' -v` 验证键盘反馈积分与隔离接口。这些离线测试不代表 ROS2 或实机已经接通。

## 2. 纯模拟集（原有快捷命令）

```bash
cd ~/Desktop/car
car
```

预期：浏览器仍打开 `http://127.0.0.1:8080/`，原 Sim2Real V13 仿真功能和场景仍然可用，新增四模式导航。选择“纯模拟集”后地址变为 `/sim`。此页面不使用测试 / 实车遥测。

## 3. 测试集（需要 Ubuntu ROS 2 Humble）

```bash
cd ~/Desktop/car
bash algorithm/car_ws/bash/web.sh build
bash algorithm/car_ws/bash/web.sh test
```

预期：自动启动或复用网站，并启动 mock MCU、CAR-LINK 只读桥接及 ROS 2→HTTP 桥接。终端输出 `/test` 地址。模拟 MCU 发布的 `/odom` 经 HTTP 出现在测试页，页面显示 `TEST`，实时数值随时间变化。默认使用 `ROS_DOMAIN_ID=73` 与常用实车域隔离。停止使用 `Ctrl+C`，不会杀死由 `car` 原先启动的网站。

检查测试通道：

```bash
curl -s http://127.0.0.1:8080/api/test/telemetry
curl -s http://127.0.0.1:8080/api/real/telemetry
```

预期：第一条在模拟 MCU 运行后出现 `"online":true` 和 `"source_type":"TEST"`；第二条**不会**接收测试数据。如果尚未构建 ROS 2 包，启动器会直接报错并给出处理方向。

## 4. 运行集（必须有真实硬件和真实串口）

默认拒绝没有 `CAR_REAL_PORT` 的运行。当前固件若只提供 ULog43，使用 `CAR_REAL_PROTOCOL=ulog43`；**只有 STM32 真正实现 CAR-LINK 才能使用 `carlink`**。

```bash
cd ~/Desktop/car
CAR_REAL_PORT=/dev/ttyUSB0 CAR_REAL_PROTOCOL=ulog43 bash algorithm/car_ws/bash/web.sh real
```

预期：未接真实串口时明确拒绝，不会用模拟数据冒充真实。连接可用并确有有效遥测时，`/real` 页面显示实时里程计；`/test` 不受影响。`/dev/ttyUSB0` 只是命令示例，务必替换为本机真实设备；程序拒绝 `/tmp/car_link_*` 和 `/dev/pts/*` 模拟串口。

真实运动数据是否可信，仍需确认物理接线、固件协议、有效帧、里程计单位、坐标配准和里程计发布节点；**ROS 2 数据存在 ≠ 实车已经验收**。

## 5. 映射集

在电脑上启动原有网页 `car`，然后访问 `http://127.0.0.1:8080/keyboard`。从本机访问该页面将自动启动 ROS2 Humble 只读订阅，不需单独运行桥接脚本。现有键盘控制节点继续使用原来的按键映射，网页只从 `/cmd_vel` 获取 vx、vy、wz 并积分成**估算**位置；不需要地图、避障、导航或任务状态机。默认订阅 `/cmd_vel`（Twist）。其他话题可在启动 `car` 前通过 `CAR_KEYBOARD_TOPIC=/你的话题 car` 指定；`TwistStamped` 可设置 `CAR_KEYBOARD_STAMPED=1 car`。自动订阅只处理 ROS2 → 网页反馈，不会启动原键盘控制发布节点。也保留 `bash algorithm/car_ws/bash/web.sh keyboard --topic /你的话题 --stamped` 手动入口。连续超过默认 0.5 秒未收到速度指令，估算速度归零；此行为不向电机发送停车指令。`POST /api/keyboard/command` 仍返回 HTTP 403。网页显示不能证明实体运动。比赛运行期间禁止遥控。

## 6. 架构与边界

- `sim2real/web_mode_gateway.py`：测试和实车 HTTP API、隔离缓存、输入校验；保留旧 `/api/*` 接口。
- `sim2real/web_mode_runner.py`：测试/实车一键启动与进程组清理，自动复用现有 `car` 网站。
- `sim2real/web/mode_router.js`：四模式入口和状态提示。
- `sim2real/keyboard_feedback_gateway.py`：只读键盘反馈通道及数据校验。
- `sim2real/keyboard_feedback_autostart.py`：本机打开 `/keyboard` 后启动一个 ROS2 只读订阅，不修改底盘控制。
- `sim2real/ros2_keyboard_feedback_bridge.py`：订阅已有 ROS2 速度话题，仅用于估算轨迹。
- `sim2real/web/keyboard.html` 和 `keyboard.js`：独立自由运动反馈网页。
- `algorithm/car_ws/bash/web.sh`：`sim|test|real|keyboard|doctor` 入口，保留原 `sim` 行为。
- `sim2real/ros2_visualization_bridge.py`：`--web-channel test|real` 将 ROS 2 数据送到指定隔离通道。
- `sim2real/server.py`：只添加独立路由拦截，不替换原有功能。

为了安全，接口要求发布方来自 HTTP loopback，并在测试页显示源标签。**这不构成硬件身份的密码学认证**：如果本机有权限运行恶意进程，仍可能伪造 REAL API。因此实车调试应在可信主机上运行，ROS 网络与串口来源需人为核对。

网页的 5×5 规划、标记坐标和实际 `/odom` 坐标需要单独标定；网页“轨迹误差”是相对于规划路径的偏差，不是已知真实坐标的定位误差。

## 7. 回退

安装器会输出 `BACKUP` 路径，原文件按仓库相对路径完整备份。需要回退时停止网页服务，将该目录内的原文件复制回项目，再删除本补丁新增的 `web_mode_gateway.py`、`web_mode_runner.py`、`web/mode_router.js`、`FOUR_MODES.md` 和 `tests/test_four_modes.py`（只在确认这些文件均由本补丁新增时删除）。也可以使用 `git diff` 审核修改，确认后用 Git 版本控制回退。
