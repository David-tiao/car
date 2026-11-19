# car_branch.sh 分支管理使用说明

## 1. 功能

用于工创小车 `car` 项目的 Git 分支管理，与原有的 `sync_car.sh` 配合使用。

主要支持：创建分支、切换分支、合并到 main、合并 main、推送功能分支。

## 2. 使用方法

进入项目目录：

```bash
cd ~/Desktop/car
chmod +x car_branch.sh
./car_branch.sh
```

打开菜单后：

| 输入 | 功能 |
|---|---|
| 回车或 `0` | 使用 main（默认） |
| `1` | 创建新分支 |
| `2` | 切换已有分支 |
| `3` | 将功能分支合并到 main |
| `4` | 将 main 合并到当前分支 |
| `5` | 推送当前功能分支 |
| `6` | 查看分支 |
| `7` | 查看状态 |

## 3. 常用快捷命令

```bash
# 返回 main
./car_branch.sh --main

# 创建 OpenMV 功能分支
./car_branch.sh --create feature/openmv

# 切换到已有分支
./car_branch.sh --switch feature/openmv

# 将功能分支合并到 main
./car_branch.sh --merge-to-main feature/openmv

# 将 main 合并到当前分支
./car_branch.sh --merge-main

# 推送当前功能分支
./car_branch.sh --push
```

## 4. 推荐使用方式

**日常一个人开发：**

直接在 `main` 修改代码，并运行：

```bash
./sync_car.sh
```

**需要单独开发新功能：**

```bash
./car_branch.sh --create feature/openmv
```

修改并测试代码，先在功能分支创建 Git 提交，然后：

```bash
./car_branch.sh --merge-to-main feature/openmv
./sync_car.sh
```

如果需要单独将功能分支上传 GitHub，切换到该分支并执行：

```bash
./car_branch.sh --push
```

## 5. 注意事项

- 新建分支基于本地 main，需要最新代码时先同步 main。
- 切换和合并前，应先提交或妥善保存未提交文件。
- 合并出现冲突会停止，不会直接修改主工作区。
- 分支合并只在本地完成，不会自动上传。
- `sync_car.sh` 仍然只管理 main。
- 不需要分支时，完全可以跳过 `car_branch.sh`。