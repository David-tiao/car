# demo 阶段任务：OpenMV ↔ STM32F4（不接 ROS 2）

本次严格保留原始项目顶层文件夹 `demo/` 与 `item/`。原始演示文件没有改名、删改；`demo/src_openmv` 的三个原文件已备份在 `原版备份/`。本阶段交付不含任何 `ros2/` 目录和 ROS 2 节点。

## 一、数据流

```text
OpenMV H7 Plus: 摄像头取帧 -> QR / 白盘+物料 / 数字环 / 黑线 / 蓝框识别
     | UART3 P4(TX), P5(RX)；115200、8N1；TTL 3.3V；共地
     v
STM32F4: UART接收中断 -> FIFO -> Vision_FeedByte -> 验证 @SEQ,LEN,BODY*XOR
     -> 存入 VisionContext.target / tracks / disk / task_code
     -> STM32应用调用 Vision_GetTarget() / Vision_GetMetricTarget()

STM32F4: 主状态机 -> Vision_SendBody("MODE,MATERIAL" 或 "COLOR,1")
     -> STM32 UART TX -> OpenMV UART RX -> PacketSerial.poll() -> 模式切换/颜色筛选
```

OpenMV 开机默认在 `QR` 模式。**识别二维码并不会自动进入 MATERIAL**：必须由 STM32 的主控制流程在合适时机发送 `MODE,MATERIAL`。不能把“目标已检测到”误当成“底盘已经对齐或夹爪已执行”。

## 二、接线

OpenMV H7 Plus：`P4 TX` → STM32 UART RX；`P5 RX` ← STM32 UART TX；`GND` ↔ `GND`。两侧使用 3.3V TTL，115200 8N1。示例假定 MCU 使用 `USART3`，这不是你的 CubeMX 实际映射保证；检查引脚后修改 `stm32/hal_uart_integration_example.c`。

注意 UART3 P4/P5 与 I2C2 共享引脚；如果你的 LCD 占用它们，需要调整接线方案。

## 三、数据帧

`@SEQ,LEN,BODY*CC\n`，其中 `LEN` 是 BODY 的 ASCII 字节数，`CC` 是 `SEQ,LEN,BODY` 全部字符的 XOR 校验（2位大写十六进制）。每帧从 `@` 起，到换行结束。

STM32 -> OpenMV：

- `MODE,QR` / `MODE,MATERIAL` / `MODE,RING` / `MODE,EDGE` / `MODE,BLUE`：切换模式
- `COLOR,0..6`：0为当前候选最近，1红2黄3蓝4绿5黑6浅蓝
- `RINGID,0..3`：0所有色环数字，1/2/3指定
- `DONE,1..4`：**在对应任务组真正完成之后**才发送
- `RESET`：重扫任务码并清空圆盘锁定历史
- `PING`：链路测试，返回 PONG

OpenMV -> STM32：

- `QR,156+123+516+231`：任务码
- `DISK,1,cx,cy,r`：当前帧认证白盘，0表示没有当前有效白盘
- `MAT,id,color,u,v,r,X,Y,Z,D,state,q,valid`：最多三项实际被看到的物料跟踪结果
- `TARGET,id,color,u,v,r,X,Y,Z,D,state,q,valid`：选中的目标物料
- `NONE,TARGET`：目标失效
- `RING,digit,u,v,X,Y,Z,D,q,valid`：数字圆环中心与深度。STM32 通过 `Vision_GetRing()` / `Vision_GetMetricRing()` 按数字 1..3 获取
- `EDGE,...` / `BLUE,...` / `STATUS,...` / `ACK,...` / `ERR,...` / `PONG`：其他识别状态及命令反馈
- `STOP,EDGE`：边线接近警报，在 STM32 内设置急停状态并执行 stop_motor 回调

`u,v,r` 为 320×240 图像像素；`X,Y,Z,D` 为 mm。光学坐标：X向右、Y向下、Z沿光轴向前；D为相机光心到目标顶部中心的直线距离。`state`：U未确认、M移动、S静止；`valid=1` 才允许把 mm 视为可用。`q` 是算法质量分，不是概率。

## 四、关键约束

1. 首次锁白盘要求三个不同颜色物料的几何提示、真实亮度盘沿和白色盘芯验证，并累计五次严格扫描。未验证白盘不发物料 TARGET。
2. 锁盘后即使只剩 2 块或 1 块，也可在继续验证同一个盘沿与盘芯的情况下跟踪；不虚构被拿走的物料。重新切模式/RESET 则失去原盘信任，需要重新初始锁定。
3. `TARGET` 的最近判定：只有 `valid=1` 是真实相机距离最近。否则退化为**离画面中心最近**，不能用于物理最近角点抓取。
4. 目前 `src_openmv/config.py` 的 `FX_PX=0, FY_PX=0, TOP_FACE_DIAMETER_MM` 各项=0：所以未经标定的输出为 `X=Y=Z=D=0`、`valid=0`，不是已经实现可靠毫米测距。
5. 单目近似测距公式：`Z=fx*物料顶部真实直径/顶部像素直径`、`X=(u-cx)*Z/fx`、`Y=(v-cy)*Z/fy`、`D=sqrt(X²+Y²+Z²)`。需要相机内参与目标真实尺寸、畸变、安装角度校正。单目估算不是硬件测距传感器。
6. 从相机坐标映射到车体/夹爪坐标还需要外参 `R,t`，未实现；底盘闭环、机械臂抓取尚未实现；STM32 `Robot_EmergencyStop` 必须接真实安全接口。
7. 每次换识别模式都让相机帧格式切换（QR/EDGE灰度、MATERIAL/BLUE/RING RGB565），**不是同时识别所有任务**。

## 五、在 CubeMX 下如何接入

复制 `stm32/vision.c`、`vision.h` 和 `app_vision.h` 到 STM32F4 现有项目中；参考 `stm32/hal_uart_integration_example.c` 把它的内容*合并*到现有 `main.c` / `usart.c` 的相应位置。不要另建一个名为 demo 或 vision 的 STM32 顶层项目，也不能直接粘贴示例生成重复 `HAL_UART_RxCpltCallback`。

先初始化 CubeMX UART，再 `App_VisionInit()`；主循环重复 `App_VisionPoll()`。

```c
// 任务步骤由 MCU 的上层状态机决定，不要每帧切模式
App_VisionSetMode("MATERIAL");
App_VisionSetColor(0);

const VisionTarget *pixel_target = App_VisionGetTarget();
if (pixel_target) {
    // pixel_target->pixel_x / pixel_y 可用
}
const VisionTarget *metric_target = App_VisionGetMetricTarget();
if (metric_target && metric_target->state == 'S') {
    // metric_target->x_mm / y_mm / z_mm / distance_mm 可用
    // 仅表示相机测量，不表示已经满足抓取条件
}
```

要读取数字圆环，使用 `App_VisionGetRing(2)` 或仅已标定毫米值的 `App_VisionGetMetricRing(2)`。要读取二维码，使用 `App_VisionGetTask(buffer)`；是否切模式由 MCU 决定，不依赖 Linux 或 ROS2。

## 六、测试

开发电脑上（非真实板）：

```sh
python3 demo/tests/test_openmv_protocol.py
cc -std=c99 -Wall -Wextra -Werror -pedantic demo/stm32/vision.c demo/tests/test_vision.c -o /tmp/vision_test && /tmp/vision_test
```

测试仅覆盖封包/校验/解析/状态保鲜/毫米有效标志和逻辑片段；**不能替代 OpenMV H7 Plus 与 STM32F4 的真实串口及测距验收**。
