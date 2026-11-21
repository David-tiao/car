# 工创 Sim2Real 四运行集 · 安装与验收

本补丁保留项目根目录的 `sim2real/` 和原有 `car` 快捷命令，**不修改 STM32/OpenMV 固件**。测试和实车通过独立 HTTP 缓存隔离，网页保留原地图、路径算法和诊断内容。新增的页面路由为：

| 页面 | 地址 | 数据来源 | 控制权限 |
|---|---|---|---|
| 纯模拟集 | `http://127.0.0.1:8080/sim` | 本地 JS 仿真算法 | 仅浏览器内部模拟 |
| 测试集 | `http://127.0.0.1:8080/test` | 模拟 MCU → Linux PTY → ROS 2 → HTTP | 只读 |
| 运行集 | `http://127.0.0.1:8080/real` | 实际 STM32 → 串口 → ROS 2 → HTTP | 只读 |
| 映射集 | `http://127.0.0.1:8080/keyboard` | WASDQE → 本机 PC ROS2 → 网页直接积分（无任务 FSM/障碍锁停）；未来 Pi 链路预留 | **实体 STM32 TX 禁止**，PC ROS2 回环可测试 |

原 `car` 命令仍在 8080 端口启动独立网站；首页 `/` 等价于纯模拟模式，新增四模式切换菜单。**不要把四套前端复制成四份项目**。所有页面共用 `sim2real/web/index.html`、`app.js`、`planner.js` 等已有资产。

## 1. 安装后，在 Ubuntu 上验收

```bash
cd ~/Desktop/car
bash algorithm/car_ws/bash/web.sh doctor
cd sim2real
python3 -m unittest discover -s tests -p 'test_four_modes.py' -v
# 安装了 Node.js 时额外验证四个前端入口
node tests/test_mode_router.js
```

预期：doctor 打印各项检查通过；`unittest` 中 6 项测试全部 `ok`，最后 `OK`。这验证页面路由、接口隔离、数据有效性、跨站拒绝及键盘命令禁用，不代表 ROS 2 或实机已接通。

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

**新方式：** 在 Ubuntu 本机启动现有网站（`car` 默认为 8080，或 `python3 sim2real/server.py --host 127.0.0.1 --port 8090`），然后从该电脑打开对应端口的 `/keyboard`。网站会自动通过原有 `start_keyboard_local_ros2.sh` 启动 ROS2 PC 发布节点和独立订阅回传节点。保留原 WASDQE 按键映射、现有网页与 `local_test_twist` 话题，不创建新的 `/cmd_vel` 控制通道。需预先安装 ROS2 Humble；缺失时查看 `/api/keyboard/autostart` 或 `sim2real/data/web_modes/keyboard_local_autostart.log`。只有本机访问会启动进程，LAN 访问只显示页面。关闭网站会结束其自动启动的节点。可以设置 `CAR_KEYBOARD_AUTOSTART=0` 禁用自动启动，沿用原手动脚本。**此功能仅为本机软件回环，绝不授权 STM32 电机驱动。**

打开 `http://127.0.0.1:8080/keyboard` 后，默认就是**本机键盘直通 ROS2**：
直接按 W/S 前后、A/D 左右平移、Q/E 转向，**无 Shift 使能、无任务 FSM、
无加速斜坡、无地图障碍锁停、无自动复位**。电脑 ROS2 的
`/car/teleop/local_test_twist` 由独立订阅节点回传给网页，
虚拟紫色车只对订阅到的车体速度做数学积分，不是实车里程计。
松开按钮立即归零，丢失通信的 300ms 看门狗保留。

可手动切换到未来的 PC→Pi→STM32 候选链路，那个模式的安全约束保持原样。
**实体 UART 尚未连接，网页不会发送 `/cmd_vel`、电机或机构命令。**
原有 `POST /api/keyboard/command` 仍返回 HTTP 403。

详细安装、验收、误差公式和安全边界见
[`KEYBOARD_MAPPING_ROS2.md`](KEYBOARD_MAPPING_ROS2.md)。

## 5.1 V15 显示质量优化（独立验收分支）

原有四个入口、TEST/REAL 数据源、WASDQE 键盘映射与现场地图的 SVG 几何均不改变。本轮只增加最后加载的 `sim2real/web/operator_polish.css`，优化顶部模式导航、地图外框、按钮、MOCK 紫色面板、ROS2 回环速度卡片及窄屏排版。通过 `focus-visible` 加强键盘可操作性，遵循系统 `prefers-reduced-motion` 设置，不引入外部字体或额外前端依赖。

`/keyboard` 标题旁新增小型 **ROS2 回环状态标识**：只有从当前所选链路实际收到在线回执时才显示“在线”；Pi 候选链路不等于实体电机反馈。原速度差曲线采用按设备像素比适配的高分辨率 Canvas 绘制（像素比最高 2），改善高 DPI Ubuntu 屏幕上的线条与刻度清晰度。图表的 ROS2 数据及交互频率仍为原有实现，**这不是额外的实体运动控制或更高频的实际里程计采样**。

验收：`node sim2real/tests/test_operator_polish.js`；随后运行 `bash algorithm/car_ws/bash/test_all_offline.sh`。视觉效果还需在 Ubuntu 浏览器实际观察，因为静态测试不能替代真实屏幕截图、视口断点和帧率检测。若只想回退样式，从 `web/index.html` 移除最后的 `operator_polish.css` 引用，并把键盘面板的新增状态标签及 Canvas 高清适配代码按此分支的 diff 还原。

## 6. 架构与边界

- `sim2real/web_mode_gateway.py`：测试和实车 HTTP API、隔离缓存、输入校验；保留旧 `/api/*` 接口。
- `sim2real/web_mode_runner.py`：测试/实车一键启动与进程组清理，自动复用现有 `car` 网站。
- `sim2real/web/mode_router.js`：四模式入口和状态提示。
- `algorithm/car_ws/bash/web.sh`：`sim|test|real|keyboard|doctor` 入口，保留原 `sim` 行为。
- `sim2real/bridges/ros2_visualization_bridge.py`：`--web-channel test|real` 将 ROS 2 数据送到指定隔离通道。
- `sim2real/server.py`：只添加独立路由拦截，不替换原有功能。

为了安全，接口要求发布方来自 HTTP loopback，并在测试页显示源标签。**这不构成硬件身份的密码学认证**：如果本机有权限运行恶意进程，仍可能伪造 REAL API。因此实车调试应在可信主机上运行，ROS 网络与串口来源需人为核对。

网页的 5×5 规划、标记坐标和实际 `/odom` 坐标需要单独标定；网页“轨迹误差”是相对于规划路径的偏差，不是已知真实坐标的定位误差。

## 7. 回退

安装器会输出 `BACKUP` 路径，原文件按仓库相对路径完整备份。需要回退时停止网页服务，将该目录内的原文件复制回项目，再删除本补丁新增的 `web_mode_gateway.py`、`web_mode_runner.py`、`web/mode_router.js`、`FOUR_MODES.md` 和 `tests/test_four_modes.py`（只在确认这些文件均由本补丁新增时删除）。也可以使用 `git diff` 审核修改，确认后用 Git 版本控制回退。
