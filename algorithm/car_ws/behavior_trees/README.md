# Nav2 行为树文件说明

- `navigate_to_pose_w_replanning_and_recovery.xml`：旧 radar 工作空间保存的 Nav2 默认行为树。包含清理 costmap、Spin、Wait、BackUp 等恢复动作，作为对照资料。
- `competition_nav_to_pose.xml`：比赛导航的保守版本，保留路径规划与跟踪，失败后只等待并有限重试，不主动旋转或倒车。

当前两份 XML 都只是工程文件。Nav2 是否使用比赛版，取决于实际启动参数中 `bt_navigator.default_nav_to_pose_bt_xml` 的配置。修改前先备份现有 Nav2 参数文件，并在静止/低速测试环境验证。

比赛规则中的场地边界、黄色禁行区和其他禁行区域不能只靠行为树保证；需要由地图、Keepout Filter/Costmap 和控制器共同约束。
