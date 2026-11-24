<<<<<<< HEAD
# 项目汇总介绍

algorithm   --- SLAM,路径规划算法/openmv算法
item        --- 整个项目的介绍(包括方案改版与进度)
STM32F4     --- 下位机算法
Hardware_mechanical        --- 树莓派5/stm32f4 开发板介绍
sim2real    --- 仿真介绍
QR          --- 二维码测试


## 每个小项目应当做到的
algorithm           car_ws  /  openmv
item                下面有比赛规则以及校赛的规则,还有docs的说明文档(这里需要更新一下),这个说明文档应当总结我们当中遇到的问题以及解决方法
Hardware_mechanical 具体的介绍板子的信息/接口之类
QR                  二维码测试
sim2real            仿真平台,完成模拟运动,小车实际运动的运行轨迹,键盘操控
STM32F4             下位机算法



## 项目未完成的点

**项目整体还没有整理清楚       ----  整理openmv阶段与stm32阶段**
openmv这一块还没整理清楚     ---- 已解决
**设计报告还未完成**
stm32f4还没整理完成         ---- 具体的代码等他整理完给我   ---已完成

ros2整体的数据流有没有打通          --- 模拟与实际隔离开来          三大接口---实际接口/模拟接口/网页数据接口
    测试集:模拟stm32--ros2接口--网页数据接口
    运行集:实际--网页数据接口
    纯模拟集:网页接口自己模拟       ---已完成
    映射集:以后可以做一个键盘到车上面的映射,就是键盘接口--实际接口---反馈到网页上
所以网页这一部分还得去修改一下,就是分为这四个网页运行
    --- 已解决

目前该项目还未提供到我们的github上面同步修改        --- 我们需要一些脚本:github同步脚本,修改之后上传的脚本//脚本已经写好:sync_car.sh(上传的时候会取消掉原来的ros2编译的工作空间)                ---- 已解决

取消github上在这个文件里面嵌套的项目               --- 已解决
仿真实验室的设计                --- 还未进行实车的验证(话题数据传输)
**雷达雷达外参还未标定**         --- 与机械商量一下雷达的安装位置,以及如果需要移动的话应该怎么移动
**openmv外参还未标定**

**还没搞清楚这个底盘操作的运动算法**
**还没完全搞清楚这个摄像头的识别物料**/**色环的算法**

脚本的全部统一写成一个,制定脚本语法
树莓派5的连接VNC
机械结构命名
每个单独的文件夹的启动脚本
**openmv的屏幕的显示实时的画面**

询问当前这个stm32_openmv是否有用意义是什么          已解决
在我们修改ros2的算法下,要保持雷达继续运行           已解决
通信协议设计                已解决         
**小车的建模还未完成**       --- urdf建模小车,最好在sim端也可以看到
**通信协议冲突处理**
**stm32为树莓派写一个串口通信**
**通信协议写一个文档总结,stm32-openmv/stm32-pi5**
**树莓派还未启用摄像头/雷达启动**
**树莓派安装ros2(本机ros2)**

https://github.com/fishros/fishros_app  ---- 可以开发一个 ros2的手机操控版本

## Gazebo 三维建模与导航

新增 [car_gazebo](algorithm/car_ws/src/car_gazebo/) 独立三维通道，不依赖网页。使用现有赛场几何生成 URDF（机器人模型）和 SDF（Gazebo 场景），区分静态预览、纯仿真 SIM 和真实位姿只读镜像 REAL。SIM 中的虚拟雷达和里程计进入独立 ROS2 图，复用道路锁定与 Dijkstra，用户确认后由 Nav2 FollowPath 执行单段路线；REAL 镜像只读取真实定位，不控制电机。

统一启动入口为 [bash/gazebo.sh](algorithm/car_ws/bash/gazebo.sh)，提供环境检查、Gazebo + RViz2 联合启动、只读状态和独立 SIM 路径验收；运行日志保留在本地。现状：0.4.0 代码候选的离线 CI 已通过，ROS2 Humble/Fortress 依赖安装及 colcon 构建通过，但 [运行级 CI #45335849586](https://github.com/David-tiao/car/actions/runs/45335849586) 路径验收 FAIL：雷达锁路 CONFLICT，未进入 FollowPath。作为未验收开发基线纳入主分支，不代表仿真运动闭环已完成。完整机械 CAD、真实传感器外参及实车镜像仍待验收。阅读：[架构与原理](docs/ARCHITECTURE_AND_ALGORITHMS.md#gazebo-三维通道)、[运行命令和验收](docs/ROADMAP_AND_ACCEPTANCE.md#gazebo--urdf-验收独立于网页)、[当前进度](docs/PROJECT_STATUS.md#gazebo-建模与导航)、[版本记录](docs/VERSION_HISTORY.md)。

## 跨电脑部署与路径约定

car 是独立 Git 项目，可放在任意目录（包括 workplace 的项目入口、树莓派或其他队员机器），不要求用户名、Desktop 或 workplace 目录。ROS 2 Humble 使用系统 Python；x86_64 与 ARM64 各自构建，不复制 build/install/log。项目内资源默认从脚本位置发现，外部 SDK、ROS setup、雷达配置等由环境变量或显式参数覆盖。启动前拒绝混用 Conda/venv，Gazebo Fortress 与 Harmonic 不视为等价环境。参见 [跨机操作与验收](docs/ROADMAP_AND_ACCEPTANCE.md#跨电脑部署与路径验收) 与 [路径设计](docs/ARCHITECTURE_AND_ALGORITHMS.md#跨电脑路径设计)。本次不清理历史已跟踪构建产物，避免破坏已有驱动；后续单独审核并迁移。
