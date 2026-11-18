# 工创智能搬运小车——前六阶段技术总结与上下位机联调交接

**记录日期：2026 年 11 月 5 日**

**当前状态：前六阶段基础功能已联调完成，进入 STM32F4、编码器里程计和视觉通信开发阶段。**

> 重要说明：目前已经实现静态地图、雷达障碍物感知、Nav2 代价地图和道路级候选路径规划，但尚未实现真实小车自主导航。当前系统仍使用固定测试 TF，不应直接控制小车行驶。

---

## 一、项目总体方案

### 1. 硬件与软件环境

- 上位机：Ubuntu 22.04
- ROS 版本：ROS 2 Humble
- 可视化：RViz2
- 激光雷达：YDLIDAR X3
- 雷达接口：CP2102 USB 转串口
- 雷达当前串口：`/dev/ttyUSB0`
- 下位机：STM32F4
- 视觉方案：考虑 OpenMV H7 Plus，具体接口需结合 `vision.c`、`vision.h` 确认
- 地图方案：已知比赛场地，预先建立静态地图
- 避障方案：X3 实时检测障碍物，Nav2 Costmap 更新障碍区域
- 路径规划方案：道路拓扑规划，整段封路，自动绕行

### 2. 总体工作流程

```text
YDLIDAR X3
     |
     v
ROS 2 /scan
     |
     v
Nav2 ObstacleLayer
     |
     v
Global / Local Costmap
     |
     v
第六阶段：道路拓扑判断
     |
     v
封闭不可通行路段
     |
     v
Dijkstra 重新选择候选路线
     |
     v
后续接入 Nav2 Planner / Controller
     |
     v
ROS 2 /cmd_vel
     |
     v
STM32F4 电机控制
     |
     v
编码器反馈 /odom
     |
     v
机器人真实定位与导航
```

目前已经完成到“候选路线规划与显示”，下面的底盘控制和真实定位尚未完成。

---

## 二、前六阶段完成情况

### [x] 第一阶段：YDLIDAR X3 雷达驱动

已完成：

- [x] 安装 YDLIDAR ROS 2 驱动
- [x] 配置 X3 串口及参数
- [x] 正常发布 `/scan`
- [x] RViz2 显示激光数据
- [x] 完成 GMapping 基础演示
- [x] 排查 USB 串口丢失和串口占用问题

主要配置：

```text
~/Desktop/ydlidar_ws/x3.yaml
```

当前雷达参数包括：

- 串口：`/dev/ttyUSB0`
- 波特率：115200
- 雷达坐标系：`laser_frame`

注意：GMapping 演示使用过固定测试 TF，不代表真实移动建图已经完成。

### [x] 第二阶段：比赛静态地图

采用已知比赛场地建立静态地图，而非依赖比赛当天重新建图。

地图参数：

| 参数 | 当前值 |
|---|---|
| 比赛场地 | 约 2.4 m × 2.4 m |
| 灰色道路宽度 | 约 0.4 m |
| 地图分辨率 | 0.01 m/像素 |
| 地图大小 | 280 × 280 像素 |
| 地图原点 | 约 (-0.2, -0.2) |

已实现：

- [x] 场地导航地图
- [x] 黄色禁行区域掩膜
- [x] 蓝色启停区域 RViz 显示
- [x] 固定原料圆盘建模
- [x] 地图 YAML / PGM 文件
- [x] 比赛区域坐标定义

固定原料圆盘中心约为 `(1.20, 2.40)`，位于场地上边界，半径约 `0.15 m`。

主要目录：

```text
~/Desktop/ydlidar_ws/maps/competition_v1/
```

主要文件：

```text
competition_navigation_map.pgm
competition_navigation_map.yaml

competition_keepout_mask.pgm
competition_keepout_mask.yaml

competition_rules_preview.pgm
competition_rules_preview.yaml

zones.json
field_preview_color.svg
```

**设计原则：黄色禁行区域通过规则掩膜表达，不作为虚构的激光实体墙。**

### [x] 第三阶段：Nav2 Keepout Filter

采用 Nav2 Keepout Filter 处理比赛规则中的不可进入区域。

已完成：

- [x] 加载黄色禁行区掩膜
- [x] 发布 Keepout Mask
- [x] 发布 Costmap Filter Info
- [x] 验证黄色区域不可通行
- [x] 验证蓝色启停区不被错误禁行
- [x] 验证地图外区域的禁行处理

主要 ROS 话题：

```text
/keepout_filter_mask
/costmap_filter_info
```

主要目录：

```text
competition_v1/keepout_stage/
```

### [x] 第四阶段：Global / Local Costmap

采用 ROS 2 Humble 的 `nav2_costmap_2d` 组件。

已完成：

- [x] Global Costmap
- [x] Local Costmap
- [x] StaticLayer
- [x] Keepout Filter
- [x] InflationLayer
- [x] 与地图、禁行区联调
- [x] RViz2 代价地图显示

主要话题：

```text
/global_costmap/costmap
/local_costmap/costmap
```

主要目录：

```text
competition_v1/costmap_stage4/
```

当前使用固定测试位姿，`base_link` 约为 `(1.20, 1.20)`，不能用于真实车辆运动定位。

### [x] 第五阶段：X3 实时障碍物检测

在第四阶段基础上，为两张 Costmap 增加：

```text
nav2_costmap_2d::ObstacleLayer
```

处理流程：

```text
X3 激光雷达
    |
    v
/scan
    |
    v
ObstacleLayer
    |
    v
Global / Local Costmap
```

已完成：

- [x] X3 实时扫描
- [x] `/scan` 接入 Costmap
- [x] 动态障碍物显示
- [x] 静态禁行区与实时障碍物共同生效
- [x] 联合启动时自动检查雷达串口
- [x] 安全清理旧雷达驱动进程

采用方案：

**Plan C：X3 固定安装在小车上，行驶过程中持续扫描和更新 Costmap。**

目前已经验证静止状态下的算法链路。

尚未验证真实移动时的动态 TF、定位以及全部比赛障碍物检测性能。

主要目录：

```text
competition_v1/lidar_stage5/
```

### [x] 第六阶段：道路拓扑路径规划

考虑到比赛道路名义宽度约为 400 mm，小车宽度约 300 mm，不能简单依赖局部规划器在狭窄道路中随意挤过障碍物。

最终采用：

**道路级拓扑规划 + 整段封路 + Dijkstra 改道。**

#### 路网结构

共 9 个道路节点、12 条候选路段。

节点坐标：

```text
X轴：

A = 0.35
B = 1.20
C = 2.05

Y轴：

1 = 0.35
2 = 1.20
3 = 2.05
```

对应路网：

```text
A3 ------- B3 ------- C3
 |          |          |
 |          |          |
A2 ------- B2 ------- C2
 |          |          |
 |          |          |
A1 ------- B1 ------- C1
```

#### 规划算法

- 使用 Dijkstra 搜索候选路线。
- 读取第五阶段的 Global Costmap。
- 读取 Keepout Mask。
- 按道路中心线及两侧约 0.17 m 的示范安全带检查。
- 发现障碍物后封闭对应道路边。
- 连续 2 次检测到受阻后封路。
- 连续 3 次确认无障碍后重新开放。
- 在剩余可通行路段中重新选择路线。

注意：0.17 m 只是当前算法参数，尚未完成真实底盘旋转包络和定位误差校验。

#### 发布的话题

```text
/competition/road_graph
/competition/road_path
/competition/road_status
```

其中：

| 话题 | 作用 |
|---|---|
| `/competition/road_graph` | 显示道路网络 |
| `/competition/road_path` | 输出候选路线 |
| `/competition/road_status` | 输出道路封闭和规划状态 |

`road_status` 状态：

- `OK`：已计算出候选路线
- `NO_ROUTE`：没有可行候选路线
- `WAIT_FOR_COSTMAP`：正在等待有效地图数据

RViz2 中：

- 绿色：可通行道路
- 红色：动态或手工封闭道路
- 橙色：静态不安全道路
- 蓝色节点：道路拓扑节点
- 青色：当前候选规划路线

**目前是道路级候选路线，不是 Nav2 Controller 可以直接执行的完整运动轨迹。**

---

## 三、第五、六阶段联合启动

已经将第五、六阶段组合为一个统一启动程序。

### 联合启动脚本

```bash
~/Desktop/start_stage56.sh
```

启动命令：

```bash
bash ~/Desktop/start_stage56.sh
```

桌面已经创建“一键启动第五六阶段”快捷方式，可直接双击启动。

### 启动流程

1. 检查 YDLIDAR X3 串口。
2. 检测是否存在旧雷达进程。
3. 对确认属于当前用户的旧 X3 驱动执行 `kill -TERM`。
4. 必要时执行 `kill -KILL`。
5. 非雷达程序占用串口时拒绝误杀。
6. 启动第五阶段 X3 和 Costmap。
7. 等待 Global Costmap 发布。
8. 启动第六阶段道路拓扑程序。
9. 在 RViz2 中显示地图、雷达、道路和候选路线。
10. 退出时清理本次启动的进程。

目前用户已验证第五、六阶段能够联合运行。

**现象说明：刚启动后需要等待几秒钟才显示青色规划路线，目前属于可接受的初始化现象，无需优先优化。**

但后续实车必须单独测试障碍物出现到停车、封路、重规划的整体延迟。

---

## 四、现有 ROS 2 接口

| 话题 | 功能 | 状态 |
|---|---|---|
| `/scan` | X3 激光扫描 | 已完成 |
| `/map` | 静态地图 | 已完成 |
| `/keepout_filter_mask` | 禁行区域 | 已完成 |
| `/costmap_filter_info` | Filter 信息 | 已完成 |
| `/global_costmap/costmap` | 全局代价地图 | 已完成 |
| `/local_costmap/costmap` | 局部代价地图 | 已完成 |
| `/competition/road_graph` | 道路显示 | 已完成 |
| `/competition/road_path` | 候选路线 | 已完成 |
| `/competition/road_status` | 路网状态 | 已完成 |
| `/cmd_vel` | 底盘速度控制 | 未接入 |
| `/odom` | 真实轮式里程计 | 未完成 |

查看道路规划状态：

```bash
source /opt/ros/humble/setup.bash

ros2 topic echo /competition/road_status \
  --once --qos-durability transient_local
```

判断规则：

- `ready=true` 不代表必然有路。
- `status=OK` 且 `route` 非空，才表示本次道路级规划成功。
- `status=NO_ROUTE` 时，不允许直接继续执行路线任务。
- `road_path` 目前没有接入电机控制。

---

## 五、当前存在的技术限制

### [ ] 1. 缺少真实里程计

当前系统使用固定测试 TF：

```text
map
 └── odom
      └── base_link
           └── laser_frame
```

实车需要：

- STM32F4 反馈编码器信息。
- ROS 2 发布真实 `/odom`。
- 发布动态 `odom → base_link`。
- 通过定位模块提供 `map → odom`。
- 标定 `base_link → laser_frame`。

接入真实 TF 时，必须取消原有固定测试变换，避免重复发布。

### [ ] 2. 缺少完整 Nav2 导航执行

目前只有道路级候选路径。

后续还需要：

- Nav2 Planner Server
- Nav2 Controller Server
- 实际目标点处理
- 机器人位姿定位
- 路径跟踪
- 底盘速度控制
- 行驶中停车和重规划

### [ ] 3. 道路安全性尚需实车验证

- 道路宽度与机器人外形
- 转弯空间
- 机器人定位误差
- 激光动态障碍物清除
- 蓝色启停区到路网的连接
- 比赛小尺寸黑色圆柱识别
- 障碍物出现后的停车距离

### [ ] 4. 尚未联调视觉与任务决策

视觉模块的二维码、物料识别与搬运任务流程需要进一步实现和验证。

---

## 六、下一阶段：STM32F4 上下位机联调

### 总体目标

实现：

```text
ROS 2 上位机
      |
      | 发送速度、运动和任务指令
      v
STM32F4
      |
      | 驱动电机
      v
小车实际运动
      |
      | 编码器反馈
      v
STM32F4
      |
      | 回传运动状态
      v
ROS 2 /odom
```

### 任务 1：通信协议

- [ ] 确认 Ubuntu 和 STM32F4 的通信方式。
- [ ] 确认串口号、波特率和接线。
- [ ] 区分 X3 雷达串口和 STM32 串口。
- [ ] 定义命令帧格式。
- [ ] 定义反馈帧格式。
- [ ] 增加帧头、长度、序号和校验机制。
- [ ] 增加通信超时停车机制。

建议优先使用稳定的 `/dev/serial/by-id/` 设备路径，避免 USB 插拔造成串口号变化。

### 任务 2：底盘运动控制

- [ ] 明确底盘类型。
- [ ] 明确电机数量。
- [ ] 明确轮径、轮距和编码器参数。
- [ ] 完成电机速度闭环控制。
- [ ] 实现速度指令解析。
- [ ] 实现前进、后退和转向。
- [ ] 实现低速限幅。
- [ ] 实现急停和超时停车。

### 任务 3：编码器里程计

- [ ] STM32F4 周期性读取编码器。
- [ ] 计算电机转速或反馈编码器计数。
- [ ] 向 Ubuntu 发送时间戳和运动数据。
- [ ] ROS 2 计算机器人运动位姿。
- [ ] 发布 `/odom`。
- [ ] 发布真实 `odom → base_link`。
- [ ] 在 RViz2 中验证机器人运动方向与实际一致。

### 任务 4：Nav2 对接

- [ ] 接收 `/cmd_vel`。
- [ ] 将速度指令发送到 STM32F4。
- [ ] 接收 STM32F4 编码器反馈。
- [ ] 更新真实机器人位姿。
- [ ] 接入完整导航规划与控制。
- [ ] 遇到障碍物时能够安全停车和重新规划。

---

## 七、`vision.c` / `vision.h` 联调交接要求

**注意：当前尚未核对实际 `vision.c` 和 `vision.h` 源代码，不能假定其已有某些函数或协议。以下为需要对接的内容。**

### 需要确认

- [ ] `vision.c` / `vision.h` 运行于哪块控制板。
- [ ] 是否使用 OpenMV H7 Plus。
- [ ] 使用哪个 USART/UART 接口。
- [ ] 是否采用 DMA 接收。
- [ ] 当前视觉协议帧格式。
- [ ] 已实现哪些识别算法。
- [ ] 识别结果如何传给 STM32F4。
- [ ] STM32F4 如何将任务状态反馈给 Ubuntu。
- [ ] 视觉数据有效性及超时机制。

### 视觉与导航的职责分工

建议采用：

**视觉模块**

- 负责二维码、颜色、物料等视觉信息识别。
- 输出识别结果和有效性状态。

**ROS 2 上位机**

- 负责地图、定位、道路选择和任务规划。
- 根据视觉任务结果确定目标或操作阶段。

**STM32F4 下位机**

- 负责电机闭环控制。
- 负责编码器反馈。
- 负责执行运动与执行机构命令。
- 负责急停及通信失联保护。

避免多个模块同时独立发送互相冲突的电机控制命令。

---

## 八、后续开发顺序

按以下顺序继续，不建议跳过中间步骤。

- [x] 第一阶段：YDLIDAR X3 雷达驱动
- [x] 第二阶段：比赛静态地图
- [x] 第三阶段：Keepout 禁行区
- [x] 第四阶段：Global / Local Costmap
- [x] 第五阶段：实时激光障碍物
- [x] 第六阶段：道路拓扑候选路径规划
- [x] 第五、六阶段联合启动脚本
- [ ] 第七阶段：STM32F4 上下位机通信
- [ ] 第七阶段：底盘速度闭环与编码器反馈
- [ ] 第八阶段：真实里程计和 TF
- [ ] 第八阶段：地图定位及完整 Nav2 导航
- [ ] 第九阶段：视觉识别与搬运任务决策
- [ ] 第十阶段：实车场地综合测试

---

## 九、下一次联调需要提供的材料

完成 STM32F4 和视觉代码整理后，请准备：

- [ ] `vision.c`
- [ ] `vision.h`
- [ ] 相关 UART/USART、DMA 和中断代码
- [ ] 电机驱动代码
- [ ] 编码器读取代码
- [ ] STM32F4 主程序或关键初始化代码
- [ ] 底盘类型及机械参数
- [ ] 上下位机通信协议
- [ ] 实际接线和串口参数

下一次开发将优先阅读现有代码，再设计 ROS 2 通信桥接，不直接覆盖已经正常运行的前六阶段程序。

---

## 十、交接结论

**已经完成：**

1. 已知比赛地图的建立。
2. Nav2 禁行区处理。
3. Global / Local Costmap。
4. X3 实时障碍物接入。
5. 道路拓扑封路和 Dijkstra 候选选路。
6. 第五、六阶段联合一键启动。
7. 旧雷达驱动的安全串口清理。

**尚未完成：**

1. STM32F4 双向通信。
2. 电机速度闭环与编码器 ROS 2 反馈。
3. 真实机器人里程计与定位。
4. 完整 Nav2 导航执行。
5. `vision.c/.h` 与任务流程联调。
6. 小车实际运动和比赛综合验收。

**后续工作原则：保留当前已经调通的前六阶段，将开发重点转移到 STM32F4 通信、运动反馈、真实定位和视觉任务整合。**

当前固定 TF 方案仅供静止算法测试，不能直接用于小车实车自主运动。