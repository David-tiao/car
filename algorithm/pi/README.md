# 树莓派 5（Pi）集成工作区

> 整理日期：2026-11-23。面向 2027 工创「智能搬运」初赛；这是**复用评审与分阶段实机验收入口**，不是声称整车已经跑通。

## 目录职责

本目录集中维护 **Pi5 专属**的运行前检查、相机实验、集成约束和验收记录。原有 ROS2、STM32、OpenMV、网页的**权威源码不搬家、不复制一份重新维护**：

- ROS2 / 雷达 / 地图 / 规划：[`algorithm/car_ws/`](../car_ws/)
- OpenMV H7 Plus 正式相机程序：[`algorithm/openmv/openmv_all/`](../openmv/openmv_all/)
- STM32 实时底盘/机构：[`STM32F4/`](../../STM32F4/)
- 仿真与观察网页：[`sim2real/`](../../sim2real/)
- Pi 原始任务：[`Hardware_mechanical/pi5/pi_task.md`](../../Hardware_mechanical/pi5/pi_task.md)

## 先看四份材料

1. [TASK_REUSE_AUDIT.md](TASK_REUSE_AUDIT.md) —— 对原始六项任务逐项对照仓库源码，说明「已有 / 能复用 / 缺什么 / 如何验收」。
2. [RULES_2027_CHECKLIST.md](RULES_2027_CHECKLIST.md) —— 依据 2027 公开赛项命题及评分规则整理的硬性约束与动作顺序；现场补充规则仍需校验。
3. [INTEGRATION_PLAN.md](INTEGRATION_PLAN.md) —— Pi5 与 OpenMV、STM32、ROS2、雷达、C2 管理系统的分层职责、实机阶段门槛。
4. [pi_healthcheck.sh](pi_healthcheck.sh) —— **只读**环境检查；默认不安装包、不启动驱动、不发运动指令。

## 已有的树莓派 CSI 摄像头实验

- [pi5_qr_material_vision.py](pi5_qr_material_vision.py)：Picamera2 + OpenCV QR 读取与红/绿/蓝 HSV 轮廓预览；Tk 图形窗口。
- [pi5_color_distance.py](pi5_color_distance.py)：已知距离标定下的**单目外观宽度估距实验**，仅红/绿/蓝，需实体标定。

这两个脚本来自此前的 Pi5 实验代码，作为**独立验证**收纳，不是可替换正式 OpenMV 六色识别 / V4 串口 / STM32 抓取执行链的已验收生产版本。特别注意：标定单目距离并不是获得任意形状、角度、遮挡条件下的真实三维测距能力。

### 只读检查

```bash
cd ~/Desktop/car
bash algorithm/pi/pi_healthcheck.sh
```

摄像头 GUI 实验需在 Pi5 **实体设备**、受支持的 Raspberry Pi OS / Picamera2 和带桌面的环境中，确认依赖、相机接口、权限与实际安装后再运行。不要在比赛或底盘可动时直接启用测试脚本。

## 绝对边界

- 现有 `car_stm32_bridge`/ULog43 **只读遥测**；不能把 `/cmd_vel` 或网页控制意图直接接上 STM32 电机。
- `/sim`、`/test`、`/keyboard` 的成功不代表 `/real` 实车成功；所有现场验收需保存版本、测量、日志、失败情况。
- 规则要求自主运行，不可在正式比赛中使用 C2、VNC、SSH、网页键盘或电脑对机器人进行遥控。
- 启动前先处理出厂镜像的默认登录/热点密码；不要把密码、证书、配对令牌或正式设备身份写入 Git。
- Pi5 相机、OpenMV、激光雷达、STM32 串口硬件可能互相竞争资源，先约定端口与设备归属。

**推荐推进顺序：** 规则尺寸/供电/急停 → Pi5 SSH/网络 → C2 只读管理 → 摄像头独立验证 → ROS2 Humble 环境 → `/scan`/TF/锁路 → STM32 真实只读遥测 → 安全控制协议 → 单件抓放闭环。
