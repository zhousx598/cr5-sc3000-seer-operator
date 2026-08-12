# 硬件、网络和安全

## 1. 硬件拓扑

当前验证方案采用两条网络：

| 设备 | 推荐连接 | 示例地址 | 主要端口 |
|---|---|---:|---|
| CR5 | Wi-Fi 或独立有线网段 | `192.168.1.6` / `192.168.100.6` | 29999、30003、30004 |
| SC3000 | 工业交换机/有线 | `192.168.192.11` | 502、主动连接 PC 的 2121/30000:30009 |
| SEER AGV | 工业交换机/有线 | `192.168.192.5` | 19204、19205、19206、19207 |
| Ubuntu PC | 对应静态地址 | Wi-Fi `192.168.1.20/24`；有线 `192.168.192.104/24` | FTP Server |

如果 CR5、相机和 AGV 都走有线，应使用独立网卡/VLAN，或由网络管理员为同一网卡
配置多个无网关的静态子网。不要给多个设备网段配置互相竞争的默认网关。

## 2. 配置 Ubuntu 有线地址

先确认接口和连接名称：

```bash
ip -br link
nmcli connection show
```

以下以接口 `enp4s0`、连接名 `Robot-Network` 为例：

```bash
sudo nmcli connection modify "Robot-Network" \
  ipv4.method manual \
  ipv4.addresses 192.168.192.104/24 \
  ipv4.gateway "" \
  ipv4.dns "" \
  ipv6.method disabled
sudo nmcli connection up "Robot-Network"
```

验证：

```bash
ip -br addr show enp4s0
ip route get 192.168.192.5
ping -c 4 192.168.192.5
ping -c 4 192.168.192.11
```

路由结果必须包含实际有线接口，而不是 `Meta`、VPN 或 TUN 接口。综合启动器默认
检查 `enp4s0`；接口名不同时：

```bash
export SEER_AGV_INTERFACE=enx001122334455
```

## 3. 端口检查

CR5：

```bash
for port in 29999 30003 30004; do
  nc -zv -w 2 "${IP_address}" "${port}"
done
```

AGV：

```bash
for port in 19204 19205 19206 19207; do
  nc -zv -w 2 192.168.192.5 "${port}"
done
```

SC3000 的 502 是 PC 主动连接相机；2121 和被动端口则由 PC 监听，供相机上传：

```bash
ss -lnt | grep -E ':(2121|3000[0-9])\b'
```

若启用了 UFW：

```bash
sudo ufw allow 2121/tcp
sudo ufw allow 30000:30009/tcp
```

## 4. 关键安全原则

- 实体急停必须可立即触及，软件急停不能替代实体急停。
- 第一次运动只允许低速、短距离、单方向，并清空完整运动包络。
- 末端总负载必须包括相机、支架、夹爪、线缆和被抓物体。
- `ClearError` 只清除可清报警，不会消除报警根因。
- `Continue` 可能恢复控制器中已有的旧队列，必须先在示教器核对任务来源。
- 本体碰撞检测和 SafeSkin 是两套独立保护；日常均应开启。
- SafeSkin 报警 `-3` 不是通信故障。只有现场确认是误触发时才允许临时关闭。
- AGV 的 `blocked`/`slowed` 是安全状态，不允许通过提高速度或修改驱动绕过。
- 地图切换和重定位会改变导航参考，必须停车、现场核对并二次确认。

## 5. 多客户端限制

不要同时让 ROS 驱动、DobotStudio、`nc` 或另一份程序占用同一 CR5 控制端口。
同样，AGV 只能由一个 `seer_agv_node` 持有 TCP 连接。GUI 已遵守这一规则：

- CR5 驱动运行时，GUI 不再额外探测 29999/30003/30004；
- GUI 不构造 `SeerClient`，只调用 `/seer_agv/*`；
- 启动器发现已有 `/seer_agv_node` 时直接复用。

## 6. 公共仓库中的敏感信息

不要提交以下内容：

- sudo/系统登录密码、Wi-Fi 密码、控制器管理密码；
- 含生产目标、人员或厂区信息的原始图片和地图；
- 不能公开的设备序列号、MAC、许可证或厂家 SDK；
- 未脱敏的现场日志。

SC3000 示例 FTP 用户名和密码是当前任务配置，不应直接用于生产网络。部署时应在
相机和程序两端同步更换，并通过网络隔离限制访问范围。

