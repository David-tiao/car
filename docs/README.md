# 工创智能搬运小车 · 项目技术文档总入口

> 文档快照：2026-11-22（Asia/Singapore）。对照仓库 `David-tiao/car`、`main@1a7a696b7609585b94618f1284fd5c3120300039`。这一 SHA 是**记录时的审计基线**，不是永久最新版本。全部条目以同一基线的源码、测试、Git 提交和模块文档为依据；没有现场证据的事项不标记“实机通过”。

## 建议阅读顺序

| 文件 | 内容 | 面向 |
| --- | --- | --- |
| [项目当前进度](PROJECT_STATUS.md) | 当前目标、各子系统状态、完成/未完成清单、主要风险 | 队长、项目管理、所有成员 |
| [整体架构与算法](ARCHITECTURE_AND_ALGORITHMS.md) | MCU、OpenMV、ROS2、地图规划、四模式网页、运动学/视觉/状态机算法 | 视觉、电控、ROS2、前端 |
| [通信协议与数据流](PROTOCOLS_AND_DATA_FLOW.md) | V4/V5、ULog43、CAR-LINK、ROS2、HTTP、数据分类与单位、安全边界 | 联调负责人 |
| [版本发展时间线](VERSION_HISTORY.md) | 阶段划分、可核查提交、变更原因、技术取舍、合并记录 | 复盘、交接、设计报告 |
| [错误复盘与经验](ISSUES_AND_LESSONS.md) | 已复现的故障、根因、修复、回归测试、残留风险 | 开发、测试、AI 辅助开发 |
| [路线图与验收标准](ROADMAP_AND_ACCEPTANCE.md) | 分阶段联调计划、离线/软件/硬件验收矩阵、操作命令、记录模板 | 全体联调人员 |

## 项目代码区一览

| 路径 | 职责 |
| --- | --- |
| [`STM32F4/`](../STM32F4/) | STM32F407 固件，麦克纳姆四轮、编码器、IMU、赛事状态机、圆环纠偏、夹爪/舵机/升降 |
| [`algorithm/openmv/openmv_all/`](../algorithm/openmv/openmv_all/) | OpenMV H7 Plus 四文件相机程序；当前配置 `UART_PROTOCOL='V4'` |
| [`algorithm/car_ws/`](../algorithm/car_ws/) | ROS2 Humble、地图、雷达、道路拓扑与 Dijkstra、任务预览、STM32 只读桥接 |
| [`sim2real/`](../sim2real/) | 统一网页、SIM/TEST/REAL/KEYBOARD 四模式，软件回环、机械 MOCK、离线验收入口 |
| [`Hardware_mechanical/`](../Hardware_mechanical/) | 机械结构图、树莓派 5 与设备资料 |
| [`item/`](../item/) | 比赛材料与早期项目记录；[早期错误笔记](../item/docs/error.md) 保留原貌 |
| [`QR/`](../QR/) | 二维码相关实验资产 |

## 阅读时务必区分的四种“通过”

1. **静态/主机测试通过**：语法、GCC HAL stub、模拟 MCU PTY、协议和状态机单元测试合格。
2. **软件闭环通过**：真实在电脑 ROS2 环境运行、消息经订阅端返回、网页状态更新；仍不是实体运动。
3. **实机子系统验收通过**：设备接线、烧录、轮速、IMU、OpenMV、机械动作有日志和重复实测记录。
4. **整车比赛闭环通过**：扫码→规划→避障→抓放与码垛→返回，全部物理动作和安全联锁可重复通过。

目前可以确认第 1 类已有成功的完整 CI 运行：[GitHub Actions #44878008112](https://github.com/David-tiao/car/actions/runs/44878008112)。第 2 类具有脚本和接口，但不凭主机 CI 宣称当前 Ubuntu 环境在线。第 3、4 类**没有足够实测证据**，详见状态和验收文档。

## 与已有模块文档的关系

- STM32 源码与历史状态：[架构](../STM32F4/docs/ARCHITECTURE.md)、[实机验收表](../STM32F4/docs/HARDWARE_VERIFICATION.md)、[工程状态（有历史分支信息）](../STM32F4/docs/PROJECT_STATE.md)。
- OpenMV：[视觉算法与运行流程](../algorithm/openmv/docs/OpenMV_视觉算法与运行流程.md)、[V4 联调](../algorithm/openmv/docs/OpenMV_V4_电控协议对齐.md)、[上板说明](../algorithm/openmv/docs/README.md)。
- ROS2：[道路规划](../algorithm/car_ws/src/car_rule_mission/README.md)、[离线执行](../algorithm/car_ws/src/car_rule_mission/OFFLINE_MISSION_EXECUTION.md)、[地图 V6 核对](../algorithm/car_ws/maps/competition_v1/v6_static_preview/README_静态场地地图核对版.md)。
- 网页：[四模式](../sim2real/FOUR_MODES.md)、[键盘](../sim2real/KEYBOARD_MAPPING_ROS2.md)、[TEST](../sim2real/TEST_SUITE.md)、[映射集历史故障模型](../sim2real/KEYBOARD_OFFLINE_TWIN_QA.md)。
- 总协议：[`通信协议.md`](../通信协议.md)（其中少数“当前版本/文件位置”是旧快照，应按本文和现行 `config.py` 复核）。

## 文档维护规则

- 版本标识采用 **日期 + Git SHA + 变更范围**；不要把前端资产缓存号或 `sim2real/VERSION.txt` 的 `13.0.0` 当成全仓统一发行版本。
- 代码、测试、机械和实机事实必须分开写；旧试验代码或历史说明不可自动当作正在使用的实现。
- 每次重要联调记录：提交 SHA、接线/固件、ROS_DOMAIN_ID、数据源（REAL/TEST/MOCK）、动作前提、预期与实际、日志、视频和结论。
- 表述 “已完成” 时指出是**源码具备、离线已测、软件实跑、实机已测**中的哪一种。
