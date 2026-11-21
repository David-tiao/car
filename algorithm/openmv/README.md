# OpenMV H7 Plus 与 architecture-v2 电控联调

相机基于 David-tiao/car 的 `algorithm/openmv/openmv_all` 四文件版，提交 `efeb3e91`。串口收发规则取自 Wangdachang888/GongChuangSai 的 `refactor/architecture-v2` 分支，提交 `1da165cc`。这个版本沿用你们的二维码、圆盘、物块和数字环识别器。[相机原始目录](https://github.com/David-tiao/car/tree/efeb3e91115bacfa2699fd6c68ccfe2e71cb2eca/algorithm/openmv/openmv_all) · [STM32 收发实现](https://github.com/Wangdachang888/GongChuangSai/blob/1da165ccc48b3a7d4f15743a5b2a6660b77226dc/Hardware/vision.c) · [比赛任务调用](https://github.com/Wangdachang888/GongChuangSai/blob/1da165ccc48b3a7d4f15743a5b2a6660b77226dc/App/app_race.c)

## 装到相机

从 `openmv_all/` 取出 `main.py`、`tools.py`、`serial.py`、`config.py`，四个文件放在 OpenMV 启动盘根目录。默认 `UART_PROTOCOL = 'V4'`。要在自己的 Git 仓库采用这次修改，将四个文件放回 `algorithm/openmv/openmv_all/`。只在使用原有 V5 电控时才选择 `'V5'`。

OpenMV P4（TX）接 STM32 PD9（RX），P5（RX）接 PD8（TX），两板共地。双方使用 UART3、115200、8N1。上电后相机发 `READY,IDLE`。下表给出消息正文。实际上线时，收发层自动封装为 `@SEQ,LEN,BODY*XX\n`。LEN 为正文字节数，XX 为从 SEQ 到 BODY 末尾所有 ASCII 字节的异或。例子 `@1,9,ALIGN,2,7*40\n` 最后一个字节为 LF。

## 串口助手和电控该看到什么

| 流程 | STM32 发 | 相机回 | 怎样判定完成 |
|---|---|---|---|
| 扫码 | 裸 `R` 后跟有校验的 `JOB,1,23` | `ACK,JOB,1`，确认读码后重复发 `QR,156+123+516+231,23` | STM32 回 `ACK,QR,23`，相机发 `DONE,QR`、`IDLE`。会话号 0 兼容 `JOB,1` 和不带会话号的 QR。 |
| 数字环 | `ALIGN,2,7` | `ACK,ALIGN,2,7`，识别到对应圆环后发 `POS,RING,2,dx,dy,cx,cy,7,0` | 电控停稳发 `STEP_DONE,0,1`，原地复测则用末位 0。相机按下一步号给新坐标。达到条件后，电控发 `ARRIVED,RING,2,dx,dy,7,count`，相机回 `IDLE`。 |
| 原料区 | `JOB,40` | `ACK,JOB,40`，确认盘几何后发 `AREA_OK,cx,cy,r`、`IDLE` | 三个量都用像素；电控据此计算第一次纠偏。 |
| 物块对准 | 区域确认后发 `JOB,41`，也可用 `JOB,41,tol_x,tol_y` | `ACK,JOB,41`，目标稳定且偏离时发 `POS,MATERIAL,0,dx,dy,cx,cy`，进入容差时发 `ARRIVED,MATERIAL,0,dx,dy`、`IDLE` | 字段 0 是电控解析器要求的目标值。坐标来自抓取位的实际物块。对准完成与夹爪抓取成功是两个事件。 |
| 中止与复位 | `CANCEL` 或 `RESET` | `IDLE` | 停止本轮观测；复位还清除任务码。 |
| 状态查询 | `STATUS` | 空闲时回 `READY,IDLE`；忙时回 `STATUS,当前视觉任务` | 电控使用 READY 建立就绪状态，忙时 STATUS 供观察。 |

圆环偏差以画面中的 (160,120) 为目标。物块沿用你们的 `PICK_TARGET_X/Y=(160,190)`，默认抓取容差为 12 像素，需要按实车夹爪位置核对。首次找盘仍需三个不同颜色物块和盘面几何。包内测试包含首次仅见两块时不确认、出现第三块时确认。

## 和电控同学一起核对的两处

1. `JOB,40` 的 `AREA_OK` 和 `IDLE` 是两帧。相机连续写出两帧，串口仍可能分两次交给电控主循环。主机测试复现了只收到第一帧时，电控已取得有效区域，但 `Vision_StartJob(41)` 会返回 0。建议电控在区域已确认后等待 `VISION_IDLE`，再从主循环下发 41；无需靠固定延时猜测。该分支原料区功能默认 `RACE_MATERIAL_ALIGN_ENABLE=0`，相机姿态、夹爪目标及纠偏矩阵另有实车标定配置。[任务切换](https://github.com/Wangdachang888/GongChuangSai/blob/1da165ccc48b3a7d4f15743a5b2a6660b77226dc/App/app_race.c#L293-L313) · [电控配置](https://github.com/Wangdachang888/GongChuangSai/blob/1da165ccc48b3a7d4f15743a5b2a6660b77226dc/Config/race_config.h#L37-L114)
2. 电控底层允许发送 11～16、21～23、31、32、42 等通用 JOB 编号，但本次核对的比赛调用链对接的是扫码、40、41 和单独的 ALIGN。这个四文件版在 V4 模式下对其余 JOB 回 `ERR,UNSUPPORTED_JOB`。尤其是 42，当前比赛代码没有可供核对的自动抓取请求与成功/失败回执闭环。原视觉算法和旧 V5 的取放流程保留在 V5 模式。

## 在电脑复现测试

需要 Python 3、C99 编译器，并在本机已有所列电控版本的检出目录。在解压目录运行：

```bash
MCU_TREE=/实际路径/GongChuangSai python3 host_test/test_peer.py
```

脚本用板级函数替身编译对方原版 `Hardware/vision.c`，收发字节真实经过两端编解码器。11 项测试覆盖扫码会话、重复及坏帧、复位、环号与复测步号、原料区切换、物料对准和未接入命令。其中一项使用合成转盘场景贯穿本方识别器。测试环境未连接实体 OpenMV、STM32、底盘或夹爪。
