# 工创智能搬运初赛静态地图 V6（核对版，不自动安装）

## 来源及依据

- 用户提供的《2027年第十届中国大学生工程实践与创新能力大赛：智能+工程创新赛道竞赛命题与运行》：第3页“运行场地”及图1；第4页表2。
- 用户提供的 `algorithm.zip`：`algorithm/ydlidar_ws/maps/competition_v1/zones.json`、`generate_static_map.py`、`topology_stage6/road_topology.py`。
- 用户提供的 `radar.zip`：`radar/README.md` 说明雷达 `/scan`、`map → odom → base_link → laser_frame` 等接口，不另提供可覆盖原规则的实体场地尺寸。

## 比赛规则明确规定

- 场地 2.4m × 2.4m。
- 中央十字灰色车道宽 0.4m。
- 四个淡黄色区域各 0.45m × 0.45m，机器人驶离起停区后不能进入。
- 两个蓝色启停区各 0.3m × 0.3m。
- 左侧暂存区、下侧粗加工区各 0.58m × 0.15m。
- 原料转盘顶面直径 0.3m，位置会在规定范围内变化。
- 黑色柱状障碍物直径 0.05m，高 0.1m，数量与位置随机；二维码板位置同样不固定。

## 与原工程统一的坐标系（部分绝对坐标为图示推算，而非竞赛规则逐点测量结果）

- `map` 地图左下场地角：(0, 0)，右上：(2.4, 2.4)，X 向右、Y 向上，单位米。
- 地图分辨率：0.01m/像素，280 × 280 像素，左下原点 [-0.2, -0.2, 0]，外围保留20cm未知区。
- 左下黄色块：[0.55,0.55,1.00,1.00]；右下：[1.40,0.55,1.85,1.00]。
- 左上黄色块：[0.55,1.40,1.00,1.85]；右上：[1.40,1.40,1.85,1.85]。
- 右上蓝色启停1：[2.10,2.10,2.40,2.40]；右下启停2：[2.10,0.00,2.40,0.30]。
- 左侧暂存区：[0,0.91,0.15,1.49]；底部粗加工区：[0.91,0,1.49,0.15]。
- 原料转盘按原工程示意中心 (1.20,2.40)，半径0.15m，转盘中心在图示北边界上。**这只是原工程建图设定；场地现场可能移动，不能当成精密测量结果。**

## 文件说明

- `competition_field_preview.png`：直观彩色赛场图。
- `competition_topology_preview.png`：原工程9节点12路段连线核对图；虚线不是已验证的安全轨迹。
- `competition_map_layers.png`：场地彩色示意、物理图、规则禁行图对比。
- `competition_navigation_map.yaml` + `.pgm`：ROS2/Nav2静态**物理占据图**：场地内空闲、原料转盘占据、场地外未知；不会把黄色禁行区当成真实物理墙。
- `competition_keepout_mask.yaml` + `.pgm`：**规则禁行掩膜**：黄色区、暂存/粗加工区、转盘占据；蓝色启停区可通过。
- `competition_rules_preview.yaml` + `.pgm`：用于RViz预览规则禁行图，和keepout掩膜几何完全相同。
- `zones.json`：几何数据、来源说明、9节点12路段。
- `competition_field_preview.svg`：可缩放彩色示意图。
- `generate_static_map_v6.py`：仅用Python标准库重新生成PGM/YAML、SVG和JSON；默认写到脚本所在目录。

## 和原V5地图的差异

- 保持所有区域的尺寸和坐标与原工程V5一致，9节点12路段不变。
- 修复一处边界问题：原料盘位于上边界时，原V5代码把地图场地**外侧**的半圆也标为 occupied。本版先判断场地边界，场地外一律 unknown；每张PGM与原版只有358个对应栅格不同。**无其他道路几何变化。**

## 使用与风险

- 当前提供的是**离线核对文件**，未改动用户Ubuntu中的任何文件，且没有执行实车定位、TF、Nav2、安全裕量验证。
- 先看图片和实地测量确认场地坐标，之后如需投入工程，应在备份条件下分别替换对应地图文件，并保持一键启动引用的路径一致。
- 使用 keepout mask 仍须正确加载 Nav2 Keepout Filter / Costmap Filter Info；仅仅加载物理`/map`不会阻止驶入黄色区域。
- 未预置障碍物和二维码板位置，比赛现场实时识别并锁路，不能提前硬编码。

### 纯离线重建（不会碰用户原competition_v1）

```bash
cd <本目录>
python3 generate_static_map_v6.py
```

### 检查文件与尺寸

```bash
python3 verify_static_map.py
```

### ROS 2 用法示例（实际工程启动流程需统一管理，避免重复map_server）

`competition_navigation_map.yaml` 作为 `nav2_map_server` 的地图输入；`competition_keepout_mask.yaml` 需独立 mask server 以及 Filter Info Server、Nav2 Keepout Filter 参数配合。不能仅依靠YAML就自动启用禁行。
