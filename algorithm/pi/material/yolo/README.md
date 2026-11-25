# Pi5 / material：YOLO ONNX 神经网络推理 + 视觉伺服

> 实现路径：`algorithm/pi/material/{include/pi_material/yolo_detector.hpp,src/yolo_detector.cpp}`，**推理及后处理均为 C++17**。训练/导出是开发 PC 上的一次性步骤。此目录不提供伪造的训练数据，也不提交大体积权重文件；没有六类有效权重，YOLO 运行模式会明确报错。

## 1. 输入和模型的**严格合同**

实现针对 Ultralytics **YOLOv8/YOLO11 Detect** 模型，导出为**固定尺寸、batch 1、六类、float32、未集成 NMS** 的 ONNX 目标检测网络；C++ 使用 `cv::dnn::readNetFromONNX` + `DNN_BACKEND_OPENCV / DNN_TARGET_CPU`，不依赖 Python/PyTorch 的运行时。

| 参数 | 要求 |
| --- | --- |
| 图像输入 | `[1,3,640,640]` (NCHW)，RGB，float32，归一化 0..1；`--yolo-imgsz 640` 必须与导出尺寸一致 |
| 图像预处理 | 保持宽高比 Letterbox 补灰 `BGR=(114,114,114)`；ROI 截取后 resize+pad；输出检测框逆变换到完整摄像头画面像素坐标 |
| 输出张量 | `[1,10,N]`、`[1,N,10]` 或 `[N,10]`；10=4 个 `cx,cy,w,h`（letterbox 后的输入像素）+ 6 个颜色类别分数 |
| 模型类别索引 | `0:red,1:yellow,2:blue,3:green,4:black,5:cyan`，对应比赛物料 ID 1..6 |
| 后处理 | 六类 argmax、置信度阈值、同类 NMS、ROI / 可选实测盘面径向约束；若目标颜色有两个或更多保留框：拒识 |
| 非支持模型 | COCO-80 预训练、YOLOv5 含 objectness 输出、内嵌 NMS `[1,300,6]`、YOLO 分类/分割/姿态模型、YOLO26 end-to-end 等；**不做猜测式解析** |

这些 ONNX 约定来自 [Ultralytics ONNX 文档](https://docs.ultralytics.com/integrations/onnx)、[模型导出文档](https://docs.ultralytics.com/modes/export)，以及 [OpenCV DNN YOLO 文档](https://docs.opencv.org/4.11.0/da/d9d/tutorial_dnn_yolo.html)。不同 Ultralytics 版本与导出参数可能改变张量结构，必须在部署前检查模型的输入/输出形状，不能只凭扩展名推断格式。

## 2. 训练数据准备（在开发电脑完成）

编制真实 Pi5 相机拍摄的训练数据。建议在**固定取料姿态**下，从多种白平衡、比赛光照、反光、遮挡、转盘速度、材料高度/大小和背景采集实物物料照片；划分训练、验证和保留的测试集，避免连续视频帧跨集合泄露。

- 标注对象必须是用于视觉伺服的**物料顶面/期望夹持参考区域**，而不是整片转盘或车体。YOLO 框中心是几何近似，不保证等于真实夹持点；有系统偏差应使用关键点或标定偏移。
- 每个标签 `class_id x_center y_center width height`，四个几何量是相对于**原图尺寸**的 0..1 归一化数字，class_id 严格对应上述六个颜色类别。
- 目录结构：`images/train`、`images/val`、`labels/train`、`labels/val`，同名图片和文本标签成对；每张图可以包含多个标签。
- 复制 `material6.example.yaml` 并修改其中 `path:` 为真实数据集的**绝对路径**；模板路径并不是已经存在的数据。
- 黑色物块容易与深色背景混淆，浅蓝与蓝容易混淆；必须特别收集负样本、交叠物块与临近颜色。

在有合适 GPU / 存储的开发电脑运行 Ultralytics 的训练与导出 CLI（仅训练需要 Python；Pi5 的部署推理仍是 C++）：

```bash
# 在训练 PC 的虚拟环境中执行，不是在 Pi5 上训练
python -m pip install ultralytics onnx
yolo detect train model=yolo11n.pt \
  data=algorithm/pi/material/yolo/material6.example.yaml \
  imgsz=640 epochs=120 batch=16

# 用实际训练产生的 best.pt 路径替换，下列路径只是示例。
yolo export model=runs/detect/train/weights/best.pt \
  format=onnx imgsz=640 batch=1 dynamic=False nms=False \
  opset=12 simplify=False
```

模型文件输出路径和可用导出参数因版本而异。**先确认导出张量的 `[1,10,N]` 形状**。建议用验证集计算六类混淆矩阵、每类 precision/recall、mAP，并单独测试端到端物料中心定位误差，而不只看 mAP。

模型加载失败（OpenCV DNN 不支持某个新算子）时应停止并报告；可尝试在**开发环境**用与实装匹配的较新 OpenCV C++ 版本重新构建，或重新导出低版本 ONNX opset。不要盲目换用 YOLOv5/COCO 结果，因为输出含义不同。

## 3. 构建和运行（树莓派 5）

在 `car/` 仓库根目录：

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev rpicam-apps

cmake -S algorithm/pi/material -B build/pi_material -DCMAKE_BUILD_TYPE=Release
cmake --build build/pi_material -j2
ctest --test-dir build/pi_material --output-on-failure
```

先运行 [QR 模块](../../QR/README.md) 扫码、显示并写出确认的任务码：

```bash
./build/pi_qr/pi_qr --source pi --output /tmp/pi_qr_verified.txt
```

然后退出 QR（避免同一个 CSI 摄像头被两个程序争抢），启动 YOLO 推理：

```bash
./build/pi_material/pi_material \
  --backend yolo \
  --model /ABS/PATH/TO/best.onnx \
  --yolo-imgsz 640 --confidence 0.50 --nms-iou 0.45 \
  --source pi --qr-file /tmp/pi_qr_verified.txt \
  --result-file /tmp/pi_material_result.json
```

若已真实测量夹持参考点像素和**局部图像雅可比** `J(px/mm)`：

```bash
./build/pi_material/pi_material --backend yolo \
  --model /ABS/PATH/TO/best.onnx --source pi \
  --target-u <你的实测值> --target-v <你的实测值> \
  --jacobian <j00,j01,j10,j11> \
  --max-step-mm 5 --max-speed-mm-s 25
```

上面的尖括号是**说明占位符，不要原样执行**。雅可比定义和受控安全标定步骤参见 [material 根目录说明](../README.md)。没有 `--target-u/--target-v/--jacobian` 时，只有像素坐标和像素误差，JSON 中毫米数为 `null`。

程序使用同一套任务顺序、三帧确认、目标丢失打断和 `N` 记录观测 / `S` 跳过 / `R` 重置 / `Q` 退出流程。**N 不等于抓取成功；没有任何串口、GPIO、电机、ROS2 /cmd_vel 写操作**。

## 4. 深度、目标中心与动态取料约束

- YOLO 检测框中心 `(u,v)` 是**图像估计**，只有经过相机/夹爪参考点标定才可用于可靠抓取；bbox 尺寸估测的单目深度 `Z=fD/d` 仅在物料顶面尺寸和焦距经过实测时才可启用，且偏角、遮挡、模型框误差会影响结果。
- `--roi x,y,w,h` 和 `--disk cx,cy,r` 可以限制目标属于取料圆盘的区域。转盘本体检测、盘面实际中心跟踪、预测其旋转位置及停稳确认尚**不属于本 PR**。
- 视觉伺服：`e=[u-u*,v-v*]^T`、`Δq=-J^{-1}e`，向量限幅、像素死区。这是二维局部 IBVS **建议值**，不是一个能自行驱动真实小车或夹爪的闭环控制器；**必须有目标新鲜度/真实机体坐标系、安全授权、停止回执和抓取回执**才能接入真实底盘。
- Pi5 CPU 运行 YOLO ONNX 的实际吞吐率/延迟未知。启用缩小模型尺寸如 320 前需**重新导出同尺寸模型**并对比准确率，不能只改单个运行参数。尽量避免 CSI 帧排队和过期观测。

## 5. 可复现的软件层面测试

`ctest` 中 `material_yolo_output_and_letterbox` 在**无训练模型**情况下构造严格匹配真实 ONNX raw shape 的合成张量，验证：

1. `[1,10,N]` 和 `[1,N,10]` 两种布局；
2. 原图/ROI Letterbox 的补边、缩放和坐标逆映射；
3. 六类映射与“当前颜色”的 argmax 筛选；
4. 重叠同类框 NMS、同色多目标安全拒识；
5. 盘面径向门控；
6. 非六类的 COCO-80 与内置 NMS 输出格式明确报错。

这只是**网络输出的后处理测试**，**不意味着已经训练好权重、实际推理通过、相机实机对焦通过、视觉伺服实车纠偏通过**。

实际验收须单独记录模型 SHA256、训练来源、OpenCV 编译版本、Pi5 图像采集帧率、模型推理耗时分位数、误检/漏检、目标位置偏差分布、不同光照下表现和停机保护行为。
