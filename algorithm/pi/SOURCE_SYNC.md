# PC 自动同步与测试源码部署

与 sync_car.sh（GitHub 提交）分离。PC 的 systemd 用户定时器默认每 60 秒检查一次，Pi 离线跳过；仅拉取指定 GitHub 分支的已提交 algorithm/pi/tests。Pi 无需互联网。新版本在独立目录中校验后发布，current 是固定入口。自动部署指测试源码文件发布，不编译 C++/ROS、不安装依赖、不运行或重启程序、不发电机命令。

首次先确认 PC→Pi SSH 免密且主机密钥可信（已有密钥可直接用；没有则自行 ssh-keygen，ssh-copy-id 后检查 BatchMode），并确认 PC 的 GitHub 凭据在无交互环境可用。不要将私钥、密码、个人配置提交到 Git。安装器只创建 PC 用户级 systemd 单元与 ~/.local/share/car-pi-auto，默认不启用 linger，不 sudo。

从已存在的 Car 仓库拉取一次启动入口（不切换当前工作区）：

```bash
cd ~/workspace/05_project_github/car
git fetch origin feat/pi-auto-deploy
git show FETCH_HEAD:scripts/pi_auto.sh | bash -s -- setup --repo "$PWD" --host pi@10.42.0.1 --branch feat/pi-auto-deploy
```

示例地址按实际 Pi 修改。setup 会校验凭据、读取指定分支的两个管理脚本、首次同步后启用定时器；内部仍会为自动管理执行 git fetch。管理器固定为安装时版本，不自动执行未来 GitHub 上的管理脚本。重复 setup 拒绝覆盖已有配置。测试分支后续更新应直接提交到本分支；未经审核不自动跟踪 main。回滚前先停止定时器，恢复自动同步后会再次追踪最新分支。

```bash
A="$HOME/.local/share/car-pi-auto/pi_auto.sh"
bash "$A" doctor
bash "$A" status
bash "$A" logs
bash "$A" run
bash "$A" auto-stop
bash "$A" rollback <release-id>
bash "$A" auto-start
```

Pi 固定入口：
`~/workspace/12_artifacts/car_source_sync/current/algorithm/pi/tests/`
从实体屏幕终端先退出旧相机程序，再使用该路径运行 undistort_demo.py 或 csi_camera_demo.sh。doctor 检查 Python OpenCV/NumPy，缺失则报错，不能据源码发布成功判定运行环境就绪。

发布：归档和文件 SHA256、Python AST/Bash 语法检查、锁、原子 current 软链接更新、旧快照保留。同步失败不切换 current；远端已损坏版本需人工诊断，禁止自动覆盖。Git 非快进更新相对于上次发布版本拒绝。相同版本重新校验但不传文件。定时器依赖 PC 在线登录及凭据有效；Pi 无网络下载。当前范围不包含 C++/ROS 自动构建或依赖包更新；生产二进制仍沿用 PC 构建、离线部署、人工启动验收。未提交的本地修改不传输。

软件验证：bash -n 两脚本；隔离模拟 PC/Pi SSH/SCP/systemctl、真实本地 Git 与文件校验的生命周期测试，覆盖首次发布、重复免传、更新保留旧版、回滚、断网跳过、语法错误、损坏传输、恢复重试。真实 systemd、SSH/GitHub 凭据、Pi 权限及硬件未验证。

```bash
bash -n sync_pi.sh && bash -n scripts/pi_auto.sh
python3 scripts/test_pi_auto.py -v
```
