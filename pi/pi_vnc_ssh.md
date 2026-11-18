# 树莓派 5：网线 SSH 与 Pi_Hot 热点恢复速查

## 1. 设备信息

系统：Raspberry Pi OS（亚博智能预装）

| 项目 | 配置 |
|---|---|
| 主机名 | raspberrypi |
| 用户名 | pi |
| 原系统密码 | yahboom |
| 热点名称 | Pi_Hot |
| 原热点密码 | 12345678 |
| Linux 有线网卡 | enp2s0 |
| 树莓派有线网卡 | eth0 |
| 树莓派无线网卡 | wlan0 |
| 电脑有线 IP | 10.42.0.1/24 |
| 树莓派历史 IP | 10.42.0.17 |

注意：10.42.0.17 是之前 DHCP 日志确认分配过的地址，不能保证现在仍在线。

## 2. Linux 电脑开启有线网络共享

在 Linux 电脑终端执行：

```bash
sudo nmcli connection modify "Wired connection 1" \
  ipv4.method shared ipv6.method disabled

sudo nmcli connection down "Wired connection 1"
sudo nmcli connection up "Wired connection 1"

ip -4 addr show enp2s0
```

如果已经正确配置为 `10.42.0.1/24`，无需重复执行修改命令。

## 3. SSH 登录树莓派

```bash
ssh -o ConnectTimeout=5 pi@10.42.0.17
```

如果地址不通，在 Linux 电脑检查 DHCP 日志：

```bash
sudo journalctl -u NetworkManager --since today \
  --no-pager | grep -Ei 'DHCPACK|DHCPREQUEST'
```

确认实际地址后重新执行 SSH。

## 4. 恢复无线网卡

以下操作必须在树莓派 SSH 终端执行：

```bash
nmcli radio all
rfkill list
nmcli device status

sudo rfkill unblock wifi
sudo nmcli radio wifi on

nmcli device status
```

如果 `wlan0` 仍然是 `unavailable`，先检查设备原因，不要删除原热点：

```bash
nmcli -f GENERAL.STATE,GENERAL.REASON,GENERAL.MANAGED \
  device show wlan0

rfkill list
```

## 5. 恢复原来的 Pi_Hot 热点

先检查原配置：

```bash
nmcli connection show "Pi_Hot"
```

当 `wlan0` 已恢复可用时：

```bash
sudo nmcli connection up id "Pi_Hot" ifname wlan0
```

验证：

```bash
nmcli device status
ip -4 addr show wlan0
```

如果已经正常广播 `Pi_Hot`，再设置自动连接：

```bash
sudo nmcli connection modify "Pi_Hot" \
  connection.autoconnect yes
```

## 6. 原配置不存在时才重新创建

如果 `Pi_Hot` 已被删除，而且 `wlan0` 可用：

```bash
sudo nmcli device wifi hotspot \
  ifname wlan0 \
  con-name Pi_Hot \
  ssid Pi_Hot \
  password '12345678'
```

这会创建新的热点配置，不保证恢复原厂全部定制参数。原配置存在时不要执行。

## 7. Linux 电脑通过 VNC 连接

安装 TigerVNC：

```bash
sudo apt update
sudo apt install -y tigervnc-viewer
vncviewer
```

连接树莓派实际的热点 IP，按 VNC 服务端要求使用对应账号密码。

如果 VNC 未启用，在树莓派 SSH 终端运行：

```bash
sudo raspi-config
```

进入 `Interface Options → VNC`，选择启用。

## 8. 完成检查

- [ ] 网线 SSH 正常
- [ ] wlan0 无线网卡可用
- [ ] Pi_Hot 热点重新出现
- [ ] 电脑能够连接 Pi_Hot
- [ ] 通过热点可以 SSH 登录
- [ ] TigerVNC 可以显示树莓派桌面
- [ ] 验证正常后修改默认密码

**注意：热点尚未在本次排查中确认恢复成功。以上恢复命令需要根据实际设备状态逐步执行，不应直接整段运行。**