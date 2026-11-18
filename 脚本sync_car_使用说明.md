# sync_car.sh 使用说明

`sync_car.sh` 是工创小车项目 `car` 的 Git 管理脚本，用于同步 GitHub、记录修改说明、查看与回溯版本，以及手动接收队友的 STM32F4 更新。

## 1. 启动方法

脚本位置：`/home/tao/Desktop/car/sync_car.sh`

```bash
cd /home/tao/Desktop/car
chmod +x sync_car.sh        # 首次使用执行一次
./sync_car.sh --help       # 查看帮助
```

## 2. 主要功能与命令

| 功能 | 命令 | 说明 |
| --- | --- | --- |
| 同步 GitHub | `./sync_car.sh` | 提交本地修改、拉取并合并远程 `main`、推送；有修改时手动填写提交标题及说明 |
| 指定提交说明 | `./sync_car.sh --message "标题" --details "详细说明"` | 不使用交互输入提交说明 |
| 查看历史 | `./sync_car.sh --history 20` | 查看最近 20 次提交及修改说明 |
| 查看版本详情 | `./sync_car.sh --show <提交ID>` | 查看版本信息、提交说明及文件变动统计 |
| 回溯版本 | `./sync_car.sh --rollback <提交ID>` | 经确认后恢复旧版文件，创建新提交并推送，不强制改写历史 |
| 查看备份 | `./sync_car.sh --backups` | 查看回溯及 STM32 合并前的本地备份标签 |
| 检查队友更新 | `./sync_car.sh --check-stm32` | 获取队友分支更新、列出尚未导入的提交，不修改本地 STM32F4 |
| 查看队友差异 | `./sync_car.sh --stm32-diff` | 查看队友相对上次导入版本的代码差异；首次导入前显示文件统计 |
| 手动合并 STM32 | `./sync_car.sh --update-stm32` | 经确认后将队友代码集成到 `car/STM32F4`，只在本地提交，不自动上传 |
| 清理编译目录 | `./sync_car.sh --delete-build` | 确认后删除本地编译目录，再同步 GitHub |
| 取消嵌套 Git | `./sync_car.sh --include-nested` | 备份嵌套 `.git` 元数据，把子项目源码交给 `car` 主仓库管理 |

## 3. 推荐使用流程

1. 修改代码后执行 `./sync_car.sh`，填写本次修改内容。
2. 想查看旧版本时，先用 `--history 20`，再用 `--show <提交ID>`。
3. 需要恢复旧版时，用 `--rollback <提交ID>` 并按提示确认。
4. 需要接收队友 STM32 代码时，依次运行 `--check-stm32`、`--stm32-diff`、`--update-stm32`；完成编译与实机检查后，再运行 `./sync_car.sh` 上传。

## 4. 注意事项

- 默认项目路径为**脚本所在目录**；主仓库为 `David-tiao/car` 的 `main` 分支。
- 队友仓库为 `Wangdachang888/GongChuangSai`，同步分支是 `refactor/architecture-v2`。
- 默认只忽略 ROS2 `build/`、`install/`、`log/` 等编译产物，**不会删除本地编译文件**；使用 `--delete-build` 才会在确认后删除。
- **首次**运行 `--update-stm32` 会用队友仓库完整快照替换 `STM32F4` 目录中原先被 Git 跟踪的文件，务必先检查差异；后续更新采用受控三方合并，冲突时停止。
- `--rollback` 作用于**整个 `car` 仓库**，执行前要求工作区干净，并会推送恢复提交；`--update-stm32` 同样要求工作区干净，但不会自动推送。
- 本脚本不会自动验证 Keil 编译、OpenMV 通信或小车实机运行，合并队友代码后应自行测试。
