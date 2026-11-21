# CAR-LINK V1 测试集：TEST V2 · 操作与验收

**数据性质：仅模拟 STM32。** 没有连接实车，网页事件、姿态、轮速均为 mock MCU 通过 PTY 发送的 CAR-LINK 帧经 ROS2 转换的结果。模拟的 COMMAND_ACK/PROGRESS/COMPLETED 并非接收过真正的控制指令。

## 一、架构

```text
car (原始快捷命令) -> Sim2Real 网页 8080 -> /test
                                    ^
                                    | HTTP GET
Mock MCU [CAR-LINK V1 binary + CRC16]
   -> PTY /tmp/car_link_demo_uart
   -> car_serial_bridge [ROS 2 /odom /car/*]
      -> ros2_visualization_bridge.py --web-channel test
         -> POST /api/test/telemetry -> 原始地图轨迹、速度
      -> test_telemetry_bridge.py
         -> POST /api/test/inspection -> 专用诊断台
```

新增：`sim2real/test_inspection.py` （只读检查 API 和输入校验）、`sim2real/test_telemetry_bridge.py` （8 个 ROS2 话题订阅）、`sim2real/web/test_console.js`（只在 /test 加载）、`sim2real/tests/test_test_suite.py`（离线测试）。既有 `sim2real/server.py` 通过 `web_mode_gateway.py` 承载这两个 API；其它模式不会读取模拟测试诊断快照。

## 二、Ubuntu ROS 2 Humble 启动

注意，源码中 `car_web_test` 是 ROS2 **适配包**，不是独立的 HTML 网页；网站仍保存在 `sim2real/`。实车、纯模拟集和映射集不在本次修改范围。

```bash
cd ~/Desktop/car
# 当需要编译时，先编译依赖（包含 car_serial_link、car_interfaces）
bash algorithm/car_ws/bash/web.sh build
# 不带参数的 car 仍然使用旧快捷方式，不必改 alias
car
# 在第二个终端一键启动模拟串口 → ROS 2 → HTTP
cd ~/Desktop/car
bash algorithm/car_ws/bash/web.sh test
```

预期：第二个终端显示 `[TEST] Collecting wheels, vision, mission, link CRC...`、`[READY]`，地址 `http://127.0.0.1:8080/test`。浏览器测试台将显示在线，地图上的小车依据模拟 `/odom` 的周期轨迹运动（并非网页内部任务动画），四轮速度、CRC、任务事件持续刷新。`ROS_DOMAIN_ID` 默认测试域 73，模拟源不应发到 REAL 通道。终端只在启动器当前实例所属进程组内清理子进程；原 `car` 服务不被终止。

**自动端到端验收（在新终端执行）：**

```bash
cd ~/Desktop/car/sim2real
python3 test_acceptance.py --timeout 15
```

预期：`[PASS]` 显示串口有效帧、/odom 序号增长和 8 个 ROS 2 话题在线，最后输出 `[SUCCESS]`。如果 ROS2 未运行，此命令会以非零退出并列出未满足项，不会伪造成功。

**快速检查：**

```bash
cd ~/Desktop/car
curl -s http://127.0.0.1:8080/api/test/inspection
curl -s http://127.0.0.1:8080/api/test/telemetry
curl -s http://127.0.0.1:8080/api/real/telemetry
```

预期：第一条 `source_type=MOCK`、`online=true`，包含 `topics` 和 `samples` 的轮速及链接状态；第二条 `source_type=TEST`、`online=true`，包含不断变化的模拟位姿；第三条在无实车桥接时 `online=false`。**网络上任何已知主机本地服务仍需信任；TEST/REAL 采用逻辑隔离与 ROS_DOMAIN_ID 隔离，不是密码学硬件认证。**

停止测试启动器（Ctrl+C）后，约 2 秒内两个测试 API 应改为 `online=false`；网页应显示离线。历史最后一帧可能保留以便检查，但绝不能继续声称实时在线或自动移动。

## 三、测试和日志

```bash
cd ~/Desktop/car/sim2real
python3 -m unittest discover -s tests -p 'test_test_suite.py' -v
# 如果有 node
node --check web/test_console.js

cd ~/Desktop/car
bash algorithm/car_ws/bash/web.sh doctor
ls sim2real/data/web_modes/*.log
```

预期：Python 测试覆盖模拟专用字段、通信健康度输入校验、测试/实车隔离、跨站拒绝、失联离线与 4 轮数据解码；`node --check` 和 `doctor` 语法检查通过。若 ROS2 启动失败或浏览器依然离线，查看 `sim2real/data/web_modes/test_ros.log`、`test_web_bridge.log`、`test_inspection_bridge.log`。

## 四、限制与提醒

1. `test_telemetry_bridge.py` **仅订阅** `/car/chassis/telemetry`、`/car/chassis/wheels`、`/car/stm32/link_status`、`/car/mission/state`、`/car/vision/event`、`/car/stm32/command_feedback`、`/car/mechanism/state` 和 `/car/power`，向本机测试 API 推送；不提供任何控制指令。
2. `/odom` 则由现有 `ros2_visualization_bridge.py` 转发，TEST 地图显示必须是真正从 mock UART 解码出来的 `odom`。
3. 模拟 STM32 轨迹改为场地内可循环的有界椭圆，不用于证明比赛道路上能够安全运动；模拟轮速数据本来就是预设值，与机体运动不一定满足真实运动学约束。
4. 网页与车场坐标映射只是用于测试显示。想验证实际控制精度，必须另外做坐标、轮径及 IMU 标定。
5. 未实际连接 STM32/树莓派/OpenMV，不代表这些硬件被验收。
