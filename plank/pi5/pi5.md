# 本文档主要介绍树莓派5的一些基础信息,提供远程连接服务的用户名与密码
连接方式:ssh,vnc

## 之前的操作
但是为了图形可视化,我支持使用vnc
之前使用realvnc,连接成功,但是这个软件由于是需要费用的,后续就没有用了
也使用过ssh,但是由于没有图形化界面,最后也没用了

## 树莓派的基本情况
树莓派的数据手册:**树莓派5主板  https://www.yahboom.com/study/raspberry5          提取码：erwp**
上述是亚伯智能提供的教程,你可以点开来输入提取码进入里面看
我的树莓派5里面已经有树莓派的操作系统,但是我后续想换成ubunut22.04的  -- 这个是后话,我现在的任务是在linux上面稳定连接树莓派5的图形化界面
树莓派出厂系统默认开启SSH、VNC以及热点，我们可以使用电脑连接树莓派的热点，然后通过SSH或者Real_VNC Viewer连接树莓派系统。
系统信息
用户名：pi
用户密码：yahboom
热点名称：Pi_Hot
热点密码：12345678
jupyter lab的密码：yahboom

## 树莓派的基本配置
PCle 连接器
8.树莓派PCIe连接器
PCIe接口


树莓派5在板子的右侧有一个FPC连接器。

注：M.2 HAT尚未发布。

默认情况下，PCIe 连接器未启用。要启用它，我们要/boot/firmware/config.txt中添加以下内容并重新启动：


# Enable the PCIe External connector.
dtparam=pciex1
pciex1还有一个更容易记住的别名，我们可以将dtparam=nvme添加到/boot/firmware/config.txt文件中。

注：当前不支持枚举交换机后面的PCIe设备。

PCIe Gen 3.0
连接认证的速度是2.0代速度(5 GT/秒)，如果在/boot/firmware/config.txt文件中添加以下内容，则可以强制连接到3.0代速度(10 GT/秒)。


# Enable the PCIe external connector
dtparam=pciex1

# Force Gen 3.0 speeds
dtparam=pciex1_gen=3
树莓派5没有经过3.0代速度认证，在这种速度下连接PCIe设备可能不稳定。

MIPI CSI/DSI连接器  -- 将显示屏与摄像头统一起来的
9.MIPI CSI/DSI连接器


之前在树莓派模型上的CSI和DSI端口已经合并为两个双重用途的CSI/DSI (MIPI)端口。为了适应板子，现在使用更密集的连接器引脚，以前只能在树莓派Zero和CM4IO板上找到。但现在您可以将两个显示器、两个摄像头或一个摄像头和一个显示器连接到这两个端口。

连接相机
相机等 Raspberry Pi 设备无需配置。如果将相机插入任一接口，我们就可以获得 CSI 连接，然后相机将可供系统识别和使用。

连接显示器
如果您将树莓派7 英寸触摸显示屏与 Raspberry Pi 5 一起使用，则不会自动配置。您需要将以下两行之一添加到您的/boot/firmware/config.txt文件中。如果是将显示器连接到CAM/DISP 1连接器时，您应该添加：


dtoverlay=vc4-kms-dsi-7inch
如果将其连接到CAM/DISP 0连接器，您可以添加以下行：


dtoverlay=vc4-kms-dsi-7inch,dsi0
可用的显示电缆


共有三种扁平柔性电缆 (FFC) 可用于将 DSI 显示器连接到 Raspberry Pi 5：

200 毫米显示电缆标准至迷你
300 毫米显示电缆标准至迷你
500 毫米显示电缆标准至迷你
使用非 Raspberry Pi 设备
如果您使用的是非 Raspberry Pi MIPI 设备（无论是相机还是显示器），它将不会自动为您的 Raspberry Pi 5 配置。相反，您需要在/boot/firmware/config.txt文件中添加一个dtoverlay设置，以正确配置正确的摄像头或显示端口。

这些dtoverlay设置应该由您的设备制造商提供。例如，添加dtoverlay=ov9281将在CSI/DSI1上配置一个基于Omnivision ov9281的相机，而添加dtoverlay=ov9281,cam0将将相同的相机添加到CSI/DSI0。

远程访问