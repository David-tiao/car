<<<<<<< HEAD
# 项目汇总介绍

algorithm   --- SLAM,路径规划算法/openmv算法
item        --- 整个项目的介绍(包括方案改版与进度)
STM32F4     --- 下位机算法
pi          --- 树莓派5介绍
sim2real    --- 仿真介绍


## 项目未完成的点

项目整体还没有整理清楚       ----  整理openmv阶段与stm32阶段
openmv这一块还没整理清楚     ---- 已解决
设计报告还未完成
stm32f4还没整理完成         ---- 具体的代码等他整理完给我


目前该项目还未提供到我们的github上面同步修改        --- 我们需要一些脚本:github同步脚本,修改之后上传的脚本//脚本已经写好:sync_car.sh(上传的时候会取消掉原来的ros2编译的工作空间)                ---- 已解决

用法:
bash sync_car.sh --delete-build
先列出匹配的本地编译目录，需要输入 YES 后才真正删除，再进行同步。删除后可能需要重新编译。
bash sync_car.sh 
同步提交
bash sync_car.sh --history 20
查看往前的版本并且还附带了版本说明
bash sync_car.sh --show a83b2c1
查看具体的版本说明
bash sync_car.sh --rollback a83b2c1
回溯版本

更多功能:./sync_car.sh --help

取消github上在这个文件里面嵌套的项目               --- 已解决
仿真实验室的设计                --- 还未进行实车的验证(话题数据传输)
雷达雷达外参还未标定
openmv外参还未标定
