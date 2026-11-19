# CARX 工创小车统一管理系统使用说明

> 项目：David-tiao/car  
> 系统：Ubuntu + ROS 2 Humble  
> 主工作空间：`/home/tao/Desktop/car/algorithm/car_ws`  
> 统一脚本：`algorithm/car_ws/bash/carx.sh`  
> 文档版本：V1.0  
> 更新日期：2026-11-19

---

## 一、CARX 是什么？

CARX 是工创小车项目的统一命令行管理入口。

以前启动和测试项目，需要分别记住：

- `car`
- `bash algorithm/car_ws/bash/web.sh test`
- `python3 sim2real/test_acceptance.py`
- `python3 sim2real/test_route_qa.py`
- 其他 ROS 2、雷达、地图及日志命令

现在将这些操作整合到 `carx` 命令中。

**以后日常开发只需要记住 `carx`。**

CARX 本身不重新实现底盘控制算法，而是调用项目中已有的脚本。

### 设计原则

1. STM32F4 是底盘控制与运动学的标准，不修改 STM32F4 固件。
2. 原有 `car` 网页启动命令保持不变。
3. TEST 使用模拟 STM32 和 ROS 2，不直接控制真实电机。
4. REAL 仅用于已经确认的真实通信链路的只读监控。
5. 键盘向真实 STM32 发送运动指令暂时禁用。
6. 所有测试和诊断功能都应有明确的执行入口和验收结果。

---

## 二、目录结构

```text
car/
├── STM32F4/                     # 原始 STM32F4 工程，保持不变
│
├── sim2real/
│   ├── launcher.py
│   ├── server.py
│   ├── web_mode_runner.py
│   ├── web_mode_gateway.py
│   ├── test_path_bridge.py
│   ├── test_telemetry_bridge.py
│   ├── test_acceptance.py
│   ├── test_route_qa.py
│   ├── web/
│   └── data/web_modes/          # TEST 日志与验收报告
│
└── algorithm/
    └── car_ws/
        ├── src/                 # ROS 2 功能包
        ├── build/
        ├── install/
        ├── log/
        │
        └── bash/
            ├── carx.sh          # CARX 统一管理入口
            ├── CARX.md          # 本文档
            ├── web.sh           # 网页 / TEST / REAL 启动器
            ├── build.sh         # ROS 2 编译
            ├── manage.sh        # V6 地图管理
            ├── lidar.sh         # 雷达管理
            ├── competition.sh   # 比赛流程管理
            └── lib/common.sh    # 公共环境函数
```

说明：

`carx.sh` 是统一入口，其余脚本作为底层功能模块保留。

不要为了减少文件数量直接删除 `web.sh`、`manage.sh` 或 `build.sh`，因为其他模块仍可能调用它们。

---

## 三、第一次使用

打开 Ubuntu 终端：

```bash
cd ~/Desktop/car
```

检查统一脚本是否存在：

```bash
ls -l algorithm/car_ws/bash/carx.sh
```

检查脚本语法：

```bash
bash -n algorithm/car_ws/bash/carx.sh
```

直接运行帮助：

```bash
bash algorithm/car_ws/bash/carx.sh help
```

如果已经将 `carx` 函数配置到 `~/.bashrc`，可以直接输入：

```bash
carx
```

如果提示：

```text
carx: command not found
```

先运行：

```bash
source ~/.bashrc
```

仍无法使用时，可以始终通过完整路径调用：

```bash
bash ~/Desktop/car/algorithm/car_ws/bash/carx.sh help
```

---

## 四、命令总表

| 命令 | 功能 | 是否控制真实电机 |
|---|---|---|
| `carx` | 显示帮助 | 否 |
| `carx test` | 启动完整 TEST 测试集 | 否 |
| `carx check` | CAR-LINK / ROS 2 通信验收 | 否 |
| `carx qa` | 规划路径跟踪质量验收 | 否 |
| `carx status` | 查看网页、路径与测试进程状态 | 否 |
| `carx logs` | 查看 TEST 日志 | 否 |
| `carx stop` | 请求停止 TEST 域中的相关测试进程 | 否 |
| `carx site` | 单独启动网页服务 | 否 |
| `carx doctor` | 检查网页相关软件环境 | 否 |
| `carx build` | 编译 CAR-LINK ROS 2 包 | 否 |
| `carx build core` | 编译核心 ROS 2 功能包 | 否 |
| `carx build all` | 编译整个 ROS 2 工作空间 | 否 |
| `carx map status` | 查看 V6 地图管理器 | 否 |
| `carx lidar doctor` | 检查雷达环境 | 否 |
| `carx competition doctor` | 检查比赛流程环境 | 否 |
| `carx real` | 启动经验证设备的真实串口只读监控 | 否 |

注意：本表描述的是统一脚本当前设计的功能范围，不代表所有真实硬件已经通过实机验收。

---

## 五、最常用的 TEST 测试流程

### 5.1 启动完整测试集

在终端 1：

```bash
carx test
```

它会调用：

```text
carx.sh
   │
   └── web.sh test
          │
          ├── 模拟 STM32
          ├── CAR-LINK 虚拟串口
          ├── ROS 2 串口桥接
          ├── 网页遥测桥接
          ├── TEST 诊断桥接
          └── TEST 规划路径桥接
```

启动器会检查网页服务。如果已有兼容的网站进程，则复用该服务；否则尝试启动网页服务。

正常情况下会显示：

```text
[ISOLATION] TEST domain=73
[WEB] http://127.0.0.1:8080/test
[START] test_ros: ...
[START] test_web_bridge: ...
[START] test_inspection_bridge: ...
[START] test_path_bridge: ...
[READY] Open http://127.0.0.1:8080/test
```

出现 `[READY]` 表示启动流程已完成，不等于运动跟踪验收通过。

### 5.2 打开测试网页

浏览器访问：

http://127.0.0.1:8080/test

TEST 页面包含：

- 青色规划路线
- 模拟小车位姿
- 实际运动轨迹
- 四轮目标速度与实际速度
- ROS 2 /odom
- 通信链路状态
- 视觉和任务模拟事件

其中 TEST 数据来自模拟 MCU，不代表真实 STM32 已经接入。

### 5.3 通信链路验收

保持 TEST 正在运行。

打开终端 2：

```bash
carx check
```

默认运行约 15 秒的通信检查。

也可以指定等待时长：

```bash
carx check 30
```

检查内容包括：

- 模拟 MCU 虚拟串口
- CAR-LINK 接收
- ROS 2 里程计
- 轮速与底盘数据
- 心跳与通信状态
- 视觉、任务及机构模拟反馈
- TEST / REAL 接口隔离

成功时会输出：

```text
[SUCCESS] /test browser acceptance can continue.
```

注意：

**通信验收通过，只能说明数据链路正常，不代表轨迹跟踪已经达到精度要求。**

### 5.4 路径跟踪验收

继续保持 TEST 和网页运行。

执行：

```bash
carx qa
```

默认采集 60 秒。

也可以运行更长时间：

```bash
carx qa 120
```

主要检查指标：

| 指标 | 含义 | 初步测试阈值 |
|---|---|---|
| RMS 路径误差 | 整段路线的均方根偏差 | ≤ 0.08 m |
| P95 路径误差 | 95% 采样点不超过的误差 | ≤ 0.12 m |
| 中心越界 | 小车中心是否越出场地 | 0 次 |
| 里程计跳变 | 是否出现异常位移跳跃 | 0 次 |
| 行驶距离 | 是否产生足够的有效运动 | 默认至少 0.15 m |
| 路径进度 | 沿参考线投影估计的进度 | 辅助指标 |

这些阈值属于 TEST 阶段的工程目标，不是 STM32F4 的实测精度保证。

路径可能自交、重复或重规划，因此进度估计不能单独作为任务完成判据。

### 5.5 停止测试

推荐在运行 `carx test` 的前台终端按：

```text
Ctrl+C
```

若先前启动的 TEST 进程出现残留，可以使用：

```bash
carx stop
```

此操作针对当前配置的 TEST ROS 域中的项目测试进程，不用于停止任意系统 ROS 2 节点。

---

## 六、检查运行状态

执行：

```bash
carx status
```

用于查看：

1. HTTP 网页是否在线。
2. TEST 规划路径接口是否在线。
3. TEST 诊断接口是否有数据。
4. 模拟 MCU 进程数量。
5. 串口桥接进程数量。
6. 路径桥接进程数量。

正常情况下，运行中的 TEST 应只有一组主要模拟节点。

如果有多个模拟 MCU 和多个串口桥接同时运行，需要检查是否有残留进程。

多个 `/odom` 发布者会增加数据混杂和轨迹异常的风险。

---

## 七、查看日志

默认查看日志末尾 20 行：

```bash
carx logs
```

查看末尾 50 行：

```bash
carx logs 50
```

日志文件位于：

```text
sim2real/data/web_modes/
├── site.log
├── test_ros.log
├── test_web_bridge.log
├── test_inspection_bridge.log
└── test_path_bridge.log
```

各日志用途：

| 日志 | 主要用途 |
|---|---|
| `site.log` | 网页服务器启动或接口异常 |
| `test_ros.log` | 模拟 STM32、ROS 2 Launch、串口桥接 |
| `test_web_bridge.log` | ROS 2 里程计转网页 |
| `test_inspection_bridge.log` | 轮速、任务、通信诊断采集 |
| `test_path_bridge.log` | 网页规划路径转 ROS 2 |

`carx logs` 会查看四个 TEST 相关日志；网站日志可单独通过 `tail` 查看。

---

## 八、编译与环境

### 编译 CAR-LINK

```bash
carx build
```

其底层调用为：

```bash
bash algorithm/car_ws/bash/build.sh serial
```

### 编译核心包

```bash
carx build core
```

### 编译整个工作空间

```bash
carx build all
```

本项目使用：

```text
ROS 2 Humble
```

工作空间：

```text
algorithm/car_ws
```

注意：

编译时不应启动其他会同时操作相同构建目录的 `colcon build` 进程。

如果遇到 `ament_cmake_python_symlink` 的目录冲突，应先检查旧构建产物，而不是修改 STM32F4。

---

## 九、其他功能入口

### 9.1 原独立网页

原来的启动命令：

```bash
car
```

保持不变。

统一管理台也提供：

```bash
carx site
```

用于调用既有的网页启动模块。

### 9.2 V6 地图与雷达

查看地图管理器状态：

```bash
carx map status
```

查看雷达环境：

```bash
carx lidar doctor
```

启动实际雷达之前需要确认设备、串口、驱动与其他运行实例，不能同时重复启动同一雷达。

### 9.3 比赛流程

检查比赛流程环境：

```bash
carx competition doctor
```

这部分沿用现有比赛脚本，不改变原有地图算法。

### 9.4 REAL 模式

REAL 仅用于真实 STM32 数据的只读监控。

使用前必须确认真实设备已经存在、串口配置正确、设备协议与桥接程序一致。

**不要把 TEST 的虚拟串口当成真实 STM32 的控制接口。**

### 9.5 键盘控制

当前不允许通过 CARX 向真实 STM32 发送键盘运动控制命令。

后续可以开发 TEST 专用键盘模拟，但必须与 REAL 通道隔离。

---

## 十、常见问题

### Q1：为什么启动 TEST 后终端没有退出？

TEST 需要持续运行模拟 MCU、串口桥接和 ROS 2 节点。

这是正常现象，不是卡死。

需要停止时在当前终端按 `Ctrl+C`。

### Q2：为什么出现 Connection refused？

表示对应的 HTTP 地址没有正在监听的服务器。

首先执行：

```bash
carx status
```

如果网页不在线，重新启动：

```bash
carx test
```

### Q3：为什么提示已经有模拟 MCU？

可能是之前的 TEST 进程仍然存在。

先停止旧的测试终端；确认是 TEST 残留进程后再执行：

```bash
carx stop
```

然后重新运行：

```bash
carx test
```

### Q4：通信检查全部通过，为什么轨迹仍然偏？

通信检查只验证消息能否传递。

轨迹偏差还可能由以下因素导致：

- 模拟底盘跟踪控制参数
- 网页坐标与 ROS 2 坐标不一致
- 多个里程计发布者
- 路径重新规划
- 航点推进和转弯控制
- 模拟电机响应模型与真实 STM32 的差异

应使用：

```bash
carx qa
```

获得实际误差统计，再分析控制问题。

### Q5：验收报告在哪里？

路径质量报告保存在：

```text
sim2real/data/web_modes/test_route_qa_*.json
```

可用于对比不同控制参数的测试结果。

### Q6：CARX 会修改 STM32F4 吗？

不会。

CARX 是本项目的启动、编译、检查和诊断管理入口。

STM32F4 作为底层运动学与控制参数标准，不会由本管理台自动修改。

---

## 十一、开发安全边界

### STM32F4

以现有固件实现为标准：

- `STM32F4/Control/Motor.c`
- `STM32F4/Control/mecanum_chassis.c`
- `STM32F4/Control/nav.c`
- `STM32F4/App/app_runtime.c`
- `STM32F4/Hardware/bsp_ulog.c`

### TEST

可以修改模拟器、适配器和验收程序，但应保证：

- 轮序与 STM32F4 一致。
- 物理量与单位一致。
- 模拟目标和实际反馈分开。
- 路径规划与实际里程计可以量化比较。
- 不产生真实电机控制输出。

### REAL

真实联调前需独立验收：

- 物理串口
- 通信协议
- 设备身份
- 反馈时效
- 断线保护
- 控制权限
- 紧急停止机制

未经验证，不开放运动指令发送。

---

## 十二、最简单的日常使用方式

正常开发只需两个终端。

**终端 1：**

```bash
carx test
```

浏览器打开：

http://127.0.0.1:8080/test

**终端 2：**

```bash
carx check
carx qa
```

需要排查问题时：

```bash
carx status
carx logs 50
```

需要查看全部命令时：

```bash
carx help
```

---

## 十三、后续扩展规划

CARX 以后可以进一步增加：

- TEST 运动学一致性验收
- OpenMV 视觉数据诊断
- STM32 ULog43 数据对照
- 雷达定位与建图检查
- 规划路径与实际轨迹自动对比
- 多进程生命周期自动管理
- 测试报告汇总

所有新增功能都应该通过 `carx.sh` 统一调用，避免在不同目录反复创建功能重叠的启动脚本。

---

**维护约定：统一入口在 `bash/carx.sh`，说明文档在 `bash/CARX.md`，真实 STM32F4 固件保持不变。**
