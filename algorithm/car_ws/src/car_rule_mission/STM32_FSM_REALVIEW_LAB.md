# 第五阶段 C：双引擎 STM32 状态机测试台（TEST 集）

## 更新后的产品设计

本轮按需求将模拟实验**从 `/real` 运行集撤出并放到 `/test` 测试集**。

网页顶部首先显示 2400×2400 mm 的**原有交互场地地图**，其网格、障碍物编辑、规划线、原生机器人与机械仿真全部保留。在地图下方增加测试台，可在两个模式间切换：

| 模式 | 实际执行后端 | 状态步骤 | 可执行真实电机？ |
| --- | --- | --- | --- |
| **串口模拟 · CAR-LINK PTY** | 原 `mission_test_session.PtyMissionSession`、`mission_wire_sim.MockPeer`、匿名 `pty.openpty()`、CAR-LINK 帧/ACK/COMPLETED | 随路线变化，通常 42 命令 | **不能** |
| **C 代码状态机 · GCC** | GCC 编译原 `STM32F4/App/race_task_plan.c` + `race_task_gate.c`，用独立测试适配器在主机调用原生 C | 34 个 C 任务动作 | **不能** |

模式切换后点「初始化任务」才会启动新测试会话。
可以「执行一步」「连续运行」「重置」，也支持指定二维码、起点、阻塞道路、ACK 丢失和动作失败故障。

**两种模式是真正不同的后端，不是前端切换两个标签。**
C 模式运行的是原生 C 计划与反馈门控，但尚不包含 `AppRace_Poll`、HAL、
机械执行器和真实通讯中断。主机没有安装 GCC 时 C 模式会明确报错。
旧 PTY 模式永远不会被 C 模式覆盖或删除。

## 可视化移动的实际含义

每当后端**完成一条行驶指令**，网页将在上方原有赛道图的独立 SVG 图层
`testMockVehicleLayer` 上，用橙色车体逐帧插值移动到下一个道路节点，
在动画中画出虚线轨迹。不会瞬间跳点。

- PTY 模式：后端原本就返回离散道路节点，使用**上一个节点 → 本次完成节点**。
- C 模式：`RaceTaskPlan` 只有 `GO_ROUGH/GO_STORAGE/GO_MATERIAL/RETURN_START` 等站点动作；使用原有 Dijkstra 道路拓扑计算相邻节点，逐路段绘制。
- 其他如 `PICK`、`PLACE_ROUGH`、`STACK_STORAGE` 不会虚构底盘位移，只在状态面板更新动作和计数。
- `FAULT` 时，后端返回原节点，不会在画面上播放没有完成的移动。
- 该图标全程标有 **MOCK**：**不代表真实机器人位置、/odom、LiDAR 测量或底盘实机运动。**

网页仍不请求 `/api/real/*`。所有模拟控制固定走
`/api/test/stm32_fsm_lab`、仅限 loopback + 同源 POST，
服务器只允许预定义的 `start/step/run/reset`，
不接受任何电机指令、不打开 `/dev/tty*`。
原 `/test` 的 ROS 模拟遥测、V4 QR 实验、CAR-LINK PTY 回放都保留；
`/real` 只读真实反馈不变。

## 界面分层

- **蓝色：** 顶部四模式切换、侧栏“设置目录”、当前菜单选择。
- **珊瑚红：** 任务模块、机械模块、状态机面板、设置内容区域。
- **青色：** 赛场地图主区、路径规划与状态指示。
- **橙色：** TEST-only 模拟小车移动轨迹和 MOCK 标识。

样式由新文件 `sim2real/web/field_hierarchy_v2.css` 覆盖，
不改原有 `app.js` 绘制/算法参数；`field_ui_hierarchy.js` 只是
**移动已有 DOM 节点到页面上方**，保留原地图 SVG ID 和所有监听器。

## 在 Ubuntu 验收

```bash
cd ~/Desktop/car
git fetch origin
git switch feat/stm32-pty-fsm-realview-20261120
git pull --ff-only
bash algorithm/car_ws/bash/test_stm32_fsm_realview.sh
```

预计测试覆盖已有 Python PTY 仿真、C 原生 34 动作反馈门控、
切换两种模式的 34/42 命令差异、运行集隔离、可动画化的道路节点数据，
以及网页资源/脚本语法。

再开一个终端：

```bash
cd ~/Desktop/car
python3 sim2real/server.py --host 127.0.0.1 --port 8090
```

访问 **`http://127.0.0.1:8090/test`**：

1. 第一屏确认赛道地图已提到最上方，地图编辑、障碍开关、原有路线仍能操作。
2. 地图下方点击「串口模拟 · CAR-LINK PTY」，选择正常、初始化，再单步或连续执行。观察橙色小车沿路网移动、反馈日志变化。
3. 重置并切换「C 代码状态机 · GCC」。初始化后应显示 34 个计划动作；前三步为原料区取料，第四步 GO_ROUGH 会沿道路到粗加工区。
4. 注入第 3 步 FAILED 或第 2 步无 ACK，验证最终 FAULT、没有后续虚假位移或计数。
5. 进入 `/real`，确认没有模拟测试面板，真实遥测仍只读。

如已有旧的 `car` 网页服务占用 8090，请先停止旧进程或更换端口。
浏览器缓存如果仍显示旧布局，使用强制刷新。

## 已知限制

- 这不是 Webots/Gazebo 物理仿真：移动是按离散节点的**连续可见动画**，
  不提供真实电机响应、碰撞测量、滑移、IMU/PID 校验或夹爪实体运动。
- C 模式只执行经过编译的 `RaceTaskPlan` / `RaceTaskGate` 业务调度；
  不是整个 STM32 固件、不是 OpenMV 真实识别，也没有真实视觉/雷达反馈。
- 本次不改变官方比赛任务动作计划、STM32 的 GPIO/UART/电机 PID 配置，
  不绕开 `RACE_PICK_DIRECT_KEY_TEST` 或相机未标定保护。
- **本阶段 PR 的自动验收尚需由 Ubuntu 执行**；不能在测试前宣称 PASS。
