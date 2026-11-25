# Pi5 C++ 物料识别、二维码顺序和视觉伺服结果

> 本目录位于 `algorithm/pi/material/`，依赖 `algorithm/pi/QR/` 的 C++ 任务码解析器（PR #13）。**这是可运行/可测试的视觉观测模块，不是已验收的实车抓取闭环**。当前没有 Pi5 实物标定数据，禁止假定毫米运动指令已经可用。

## 1. 工作链路

```text
pi/QR：Pi5 CSI 相机 → OpenCV QR → 合法任务码（连续3帧）
                               ↓ --output /tmp/pi_qr_verified.txt
pi/material：读取并再次校验任务码 + 检查文件新鲜度
   → batch1 颜色顺序（组1）：RED / BLACK / CYAN
   → batch2 颜色顺序（组3）：BLACK / RED / CYAN
   → 分别在摄像头画面寻找当前指定颜色的圆形物料候选
   → 唯一候选、连续帧/位置抖动确认
   → 当前像素中心与实测夹爪参考点的误差
   → （可选：经真实标定的图像雅可比）计算纠偏量/速度建议
   → 人工 N 记录本步“已观察”并转到下一物料
   → 最终 6 步只读 JSON 观测记录
```

**颜色编码**沿用 `algorithm/openmv/openmv_all/config.py`：`1 红、2 黄、3 蓝、4 绿、5 黑、6 浅蓝`；OpenMV 原有阈值为 LAB，不能直接用到 Pi OpenCV HSV。项目使用 `hsv_thresholds.example.csv` 作为起始值；物料识别需用比赛光源、材质、工作距离、相机白平衡重新测色。

**六次任务顺序**例子 `156+123+516+231`：

| 步 | 批次 | 目标颜色 | 对应加工环号 | 最终码垛底座环号 |
| --- | --- | --- | --- | --- |
| 1 | 1 | 红(1) | 1 | 1 |
| 2 | 1 | 黑(5) | 2 | 2 |
| 3 | 1 | 浅蓝(6) | 3 | 3 |
| 4 | 2 | 黑(5) | 2 | 2 |
| 5 | 2 | 红(1) | 3 | 1 |
| 6 | 2 | 浅蓝(6) | 1 | 3 |

注意：第二批粗加工环号按**第四组**取；第二批暂存区的**同色叠放底座**按第一批颜色与环号映射，不可拿第四组覆盖。

## 2. C++ 编译与离线单测

Pi5 Raspberry Pi OS ARM64（安装依赖先核实系统对应包版本）：

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev rpicam-apps

# 先构建 QR（产出 pi_qr 扫码程序）
cmake -S algorithm/pi/QR -B build/pi_qr -DCMAKE_BUILD_TYPE=Release
cmake --build build/pi_qr -j2
ctest --test-dir build/pi_qr --output-on-failure

# 再构建材料识别程序
cmake -S algorithm/pi/material -B build/pi_material -DCMAKE_BUILD_TYPE=Release
cmake --build build/pi_material -j2
ctest --test-dir build/pi_material --output-on-failure
```

`test_material_core` 不依赖 OpenCV；在未安装 OpenCV 的 Linux 主机也能单独测试任务顺序、连续帧、2×2 雅可比纠偏、限幅、奇异矩阵拒绝：

```bash
cmake -S algorithm/pi/material -B build/pi_material_core -DPI_MATERIAL_WITH_OPENCV=OFF
cmake --build build/pi_material_core -j2
ctest --test-dir build/pi_material_core --output-on-failure
```

`test_material_detector` 使用合成图像验证红色、错误颜色、双红目标歧义、ROI 等；真实现场必须重新验收。

## 3. 从二维码接入，按顺序识别

先运行 QR，显示二维码并写入合法任务码。完成扫码后退出 QR，避免单 CSI 摄像头被两个进程争用：

```bash
./build/pi_qr/pi_qr --source pi --output /tmp/pi_qr_verified.txt
```

再运行物料识别（任务码文件默认要求启动时修改时间在 600 秒以内）：

```bash
./build/pi_material/pi_material --source pi \
  --qr-file /tmp/pi_qr_verified.txt \
  --hsv-file algorithm/pi/material/hsv_thresholds.example.csv \
  --roi 80,90,470,320 \
  --result-file /tmp/pi_material_result.json
```

上方 `--roi` **仅为命令格式示例**，不表示真实物料区坐标；必须按 Pi 摄像头画面设置，特别是识别黑色物料时，以免把黑色车体、底盘、场地背景误当物块。若转盘投影已实测，可加 `--disk cx,cy,r` 限定材料在半径 `0.39r..0.96r` 的轨道上（与 OpenMV 源代码盘面门控相呼应），**本版本不自动检测转盘位置**。

操作键：`N` 仅当当前颜色目标连续 ≥3 帧稳定时，**记录本步视觉观测**并进入下一个目标；`S` 把本步标为跳过；`R` 重置当前跟踪；`Q`/`Esc` 退出。没有收到抓取成功/失败回执，按 N **不能称为抓取成功**。画面能看到目标颜色、加工/码垛环号、当前状态、像素参考点、框与偏差箭头。

**二维码码变更策略**：材料程序启动时读取并冻结一次合法扫码结果，不会在运行中偷偷改变六步顺序。若比赛需要重新扫码，退出并启动新会话。

## 4. 视觉伺服：物料中心如何转换成车体移动建议

使用当前物料中心 `(u,v)`，相机像素坐标 **右为正、下为正**；`(u*,v*)` 是经过**实际夹爪与摄像头标定**的目标位置。

```text
e = [ u-u*, v-v* ]^T                       // px
J = [ du/dx  du/dy ; dv/dx  dv/dy ]       // px/mm
Δq_full = -J^{-1} e                        // mm，期望平面微移
Δq_step = clip_norm(Δq_full, max_step)    // mm，单步建议
v_advice = clip_norm(K·Δq_full, max_v)    // mm/s，比例伺服速度建议
```

- `J` 必须在**相同摄像头装配方式、物料高度/距离和目标坐标系**下用真实实验测量。它描述的是 **机器人正向移动 1 mm 所引起的静止物料在图像中的像素移动**；不是普通 `mm/px` 矩阵，不可凭想象填写。
- 分别在安全、受控的台架环境中施加已知的机器人平面 X/Y 微位移，测量同一静止目标的像素位移，估计四个雅可比元素。建议多个位置重复标定、回归拟合并测试回程差；变更焦距、工作高度、转盘姿态或摄像头固定方式要重标定。**不要由该程序驱动实车进行标定**。
- `--target-u/--target-v` 必须成对提供，`--jacobian` 必须非奇异才允许输出毫米建议。默认参考点仅是旧 OpenMV 的 `(160,190)` 按画面大小缩放得到的**诊断十字线**，不属于 Pi 摄像头标定。
- 建议运动采用向量模长限幅：默认单步 ≤5mm，速度 ≤25mm/s；进入像素容差半径内建议速度为 0。**没有下位机闭环、停止反馈、坐标系授权，任何建议都不得自动发布到 /cmd_vel 或电机**。
- 此处用的是**局部平面 image-based visual servo（IBVS 的二维近似）**，不是完整 6 自由度特征交互矩阵，也不涵盖机器人转向、物料移动、转盘角速度或深度不确定性。对动态转盘还需要轨迹预测、停车确认及抓取姿态约束。

标定后的命令形式（**数字仅为格式举例，不是实测值**）：

```bash
./build/pi_material/pi_material --source pi \
  --target-u 320 --target-v 380 \
  --jacobian '-2.10,0.04,0.12,-1.85' \
  --pixel-tolerance 8 --gain 0.7 \
  --max-step-mm 5 --max-speed-mm-s 25
```

在未标定时，输出 `error_px`、`pixel_centered`；`servo_calibrated=false`，三个以 mm/mm/s 为单位的字段都是 JSON `null`，不允许报告“已完成毫米纠偏”。

## 5. 物料尺寸测距（仅有标定时）

如果实际材料顶面直径 `D_mm` 已量出、同一焦段的相机焦距 `f_px` 已标定，使用针孔近似：

```text
Z_approx_mm = f_px * D_mm / observed_diameter_px
```

可选参数 `--focal-px 650 --known-diameter-mm 30`。直径、焦距、相机与目标倾角、畸变和顶面是否圆形都会引入误差；**这只是估测镜头到目标的粗略距离，不是可靠抓取高度或激光雷达测距**。未标定则 `approx_monocular_range_mm=null`。

## 6. 无 Pi 摄像头的完整六步离线验证

准备按二维码颜色顺序的六张测试图片（每张应只有一个符合该颜色和几何条件的圆形物料），按行写入文本：

```text
/tmp/red1.png
/tmp/black1.png
/tmp/cyan1.png
/tmp/black2.png
/tmp/red2.png
/tmp/cyan2.png
```

然后：

```bash
./build/pi_material/pi_material \
  --task-code '156+123+516+231' \
  --image-list /tmp/material_frames.txt \
  --repeat-each 3 --confirm-frames 3 \
  --offline-auto-next --headless \
  --result-file /tmp/pi_material_result.json
cat /tmp/pi_material_result.json
```

**只允许离线图片模式**用 `--offline-auto-next`，并明确这些是静态图片的人工重复帧；它们并不代表真实连续采集或机械动作完成。真实摄像头只能由人工按 N 记录下一步。

每步结果包含 `batch`、`color_id`、`process_ring`、`storage_ring`、物料像素中心、物料轮廓尺寸、像素误差、估算纠偏量/限幅步长/建议速度（仅在标定后）、近似测距（仅在给出物理标定后），并写入 **原子更新的** `/tmp/pi_material_result.json`。 `all_six_observed=true` 也只表示六步均获得了视觉观测，`physical_actions_executed=false` 始终成立。

## 7. 仍需实机验收

- [ ] Pi5 CSI、摄像头长时间连续采集、设备断流异常、帧间时序。
- [ ] 在实际比赛光照和旋转转盘场景中标定 HSV，记录六色误检率/漏检率、混淆矩阵、目标尺寸/ROI。
- [ ] 黑色背景隔离、目标遮挡、反光、多个同色物料出现时的安全拒识。
- [ ] 标定 Pi 摄像头夹爪目标像素和 Jacobian 2×2，记录误差、范围和重标定条件。
- [ ] 若使用单目测距，标定目标真实直径与相机 `f_px`，实测检验误差；必要时引入测距传感器。
- [ ] 动态转盘跟踪、时间对齐、到取料位的等待与停稳确认。
- [ ] 与 STM32 定义唯一权威视觉源/任务会话，以及“只读观测→运动授权→实际抓取回执”闭环；当前**没有**可写真实底盘闭环，不能自动驱动电机/夹爪。
- [ ] 与 QR 代码/屏显示互斥摄像头使用，实测二维码内容→材料顺序→全 6 步报告的端到端链路。

本目录不修改 `algorithm/openmv/`、`STM32F4/`、`algorithm/car_ws/` 和 C2 代码。

## 8. YOLO 神经网络推理（C++ ONNX）

本目录现在支持 `--backend hsv`（默认）和 `--backend yolo` 两种可互换识别后端，二者共享**同一套六步 QR 目标序列、稳定跟踪、局部视觉伺服和 JSON 报告**。

YOLO 模型必须是已按 `red / yellow / blue / green / black / cyan` 训练的**六类** YOLOv8/YOLO11 Detect ONNX，且 `nms=False`、batch=1、输出张量的通道数为 10。COCO-80 / YOLOv5 / 内嵌 NMS 导出格式不能直接用。训练和导出细节见 [YOLO 推理与数据集说明](yolo/README.md)，数据集类别模板见 [material6.example.yaml](yolo/material6.example.yaml)。

```bash
# 编译时需要 OpenCV dnn 模块：libopencv-dev
cmake -S algorithm/pi/material -B build/pi_material -DCMAKE_BUILD_TYPE=Release
cmake --build build/pi_material -j2
ctest --test-dir build/pi_material --output-on-failure

# 先用 algorithm/pi/QR/pi_qr 扫码写出 /tmp/pi_qr_verified.txt
# 退出 QR 摄像头进程，然后在 Pi5 上执行：
./build/pi_material/pi_material \
  --backend yolo --model /ABS/PATH/TO/best.onnx \
  --yolo-imgsz 640 --confidence 0.50 --nms-iou 0.45 \
  --source pi --qr-file /tmp/pi_qr_verified.txt \
  --result-file /tmp/pi_material_result.json
```

YOLO 使用 Letterbox 原图坐标逆映射、argmax 类别过滤和 NMS；只检测**当前任务指定颜色**，同色多候选必须拒识。报告增加 `detector_backend`、`detector_confidence` 和 `detector_processing_ms`（仅检测器运行时间，不是实际图像/控制指令全链路延迟）。

**当前没有六类实际训练权重、实物标定和实机推理记录**。YOLO 检测框中心只是视觉目标候选；若未提供经过实测的夹爪目标点与雅可比，仍不会产生毫米级动作建议，也不会发送任何真实电机命令。

