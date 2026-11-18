# sync_car.sh 使用说明

## 1. 脚本介绍

`sync_car.sh` 用于工创小车 `car` 项目的 GitHub 代码管理。

主要功能：
- 一键提交并同步代码到 GitHub。
- 自动重试网络异常，检查推送是否成功。
- 查看历史版本并安全回退。
- 检查和合并队友的 STM32F4 代码。
- 防止重要文件误删除。

## 2. 首次使用

脚本放在 `~/Desktop/car/` 目录下。

打开 Ubuntu 终端：

```bash
cd ~/Desktop/car
chmod +x sync_car.sh
bash -n sync_car.sh
```

## 3. 常用命令

| 命令 | 功能 |
|---|---|
| `./sync_car.sh` | 一键同步代码到 GitHub |
| `./sync_car.sh --preview` | 预览本地代码修改 |
| `./sync_car.sh --doctor` | 检查仓库状态 |
| `./sync_car.sh --history 20` | 查看最近 20 次提交 |
| `./sync_car.sh --show <版本ID>` | 查看指定版本 |
| `./sync_car.sh --rollback <版本ID>` | 恢复历史版本 |
| `./sync_car.sh --check-remote` | 检查是否同步成功 |
| `./sync_car.sh --retry-push` | 推送失败后重新推送 |
| `./sync_car.sh --backups` | 查看备份版本 |

**平时上传代码只需要执行：**

```bash
./sync_car.sh
```

按照提示填写提交标题和修改说明，说明写完后单独输入 `END` 结束。

## 4. STM32F4 队友代码同步

| 命令 | 功能 |
|---|---|
| `./sync_car.sh --check-stm32` | 检查队友有没有更新 |
| `./sync_car.sh --stm32-diff` | 查看队友修改了哪些代码 |
| `./sync_car.sh --update-stm32` | 将队友代码合并到本地 |

首次导入需要输入 `ADOPT STM32`，会用队友整个仓库快照替换本地 `STM32F4` 中已跟踪的文件。

后续增量合并需要输入 `MERGE STM32`。

**注意：STM32 合并完成后不会自动上传 GitHub。** 建议先完成编译和实机测试，再执行 `./sync_car.sh`。

## 5. 推送失败怎么办？

如果出现 `HTTP 408` 或其他网络错误，先检查：

```bash
./sync_car.sh --check-remote
```

确认未成功同步后，重新推送：

```bash
./sync_car.sh --retry-push
```

如果仍然失败，可以尝试：

```bash
CAR_GIT_HTTP_VERSION=HTTP/1.1 ./sync_car.sh --retry-push
```

脚本默认自动重试最多 3 次，不会强制覆盖远程代码。

## 6. 其他功能

处理嵌套 Git 仓库：

```bash
./sync_car.sh --include-nested
```

确认后删除本地编译目录：

```bash
./sync_car.sh --delete-build
```

## 7. 注意事项

1. 重要文件被删除时，脚本会要求人工确认。
2. Git 合并出现冲突时，脚本会停止，不会自动覆盖代码。
3. 历史版本回退会创建新提交，保留原有 Git 历史。
4. STM32F4 代码更新后，应先测试，再推送到 GitHub。

---

**推荐工作流程：**

日常开发：`修改代码 → ./sync_car.sh → GitHub`

队友代码更新：`--check-stm32 → --stm32-diff → --update-stm32 → 编译测试 → ./sync_car.sh`