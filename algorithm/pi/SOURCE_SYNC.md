# PC 到 Pi 的独立源码同步

本工具与 sync_car.sh（GitHub 同步）职责不同。只在 PC 上执行，使用已有 SSH/SCP，不要求 Pi 访问 GitHub。默认选择本地 Git HEAD 的 algorithm/pi/tests，也可用 --ref 选择已获取的远端分支；仅允许 algorithm/pi 下的明确路径。未提交的工作树修改不会传输。

```bash
bash sync_pi.sh doctor --host pi@10.42.0.1
bash sync_pi.sh plan --host pi@10.42.0.1 --ref origin/test/pi-csi-camera-demo
bash sync_pi.sh sync --host pi@10.42.0.1 --ref origin/test/pi-csi-camera-demo
```

在仓库外调用时提供 --repo /path/to/car；先在 PC 使用 git fetch 获取所需引用。doctor 只读检查 SSH、架构及远端工具，plan 只列出选定提交的文件，sync 才写入 Pi。源文件保留 Git 路径，发布到 ~/workspace/12_artifacts/car_source_sync/<commit>-<archivehash>/，输出 [PATH]。同一已验证快照不再传输；新版本另建目录，不覆盖旧版本。使用归档与逐文件 SHA256 校验、独立暂存目录、发布锁。失败的暂存目录保留供诊断，不自动删除。既有 ROS、视觉运行时、依赖包不变。不安装、不编译、不启动硬件、不修改 Pi 系统。暂不支持其他模块、未提交文件和反向同步。

测试证据：Bash 语法及隔离模拟 SSH/SCP（非真实 Pi）完成 plan/doctor 无写入、首次发布、重复免传输、损坏传输拒绝、新版本保留旧版本、符号链接源拒绝。真实 SSH、Pi 权限及运行测试仍需验收。
