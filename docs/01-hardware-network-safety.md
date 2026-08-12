# 硬件、网络和安全

## 1. 硬件拓扑

当前验证方案将机器人设备统一接入 `192.168.192.0/24` 有线网络，Wi-Fi 只用于
互联网访问，不再连接 CR5 的机器人 Wi-Fi：

| 设备 | 推荐连接 | 示例地址 | 主要端口 |
|---|---|---:|---|
| CR5 | 工业交换机/有线 | `192.168.192.201` | 29999、30003、30005 |
| SC3000 | 工业交换机/有线 | `192.168.192.11` | 502、主动连接 PC 的 2121/30000:30009 |
| SEER AGV | 工业交换机/有线 | `192.168.192.5` | 19204、19205、19206、19207 |
| Ubuntu PC | `enp88s0` | `192.168.192.104/24` | FTP Server |

机器人有线连接不设置默认网关，避免与 Wi-Fi 的互联网默认路由竞争。VPN、Mihomo
或其他 TUN/透明代理也不得接管 `192.168.192.0/24`。

## 2. 配置 Ubuntu 有线地址

先确认接口和连接名称：

```bash
ip -br link
nmcli connection show
```

以下以接口 `enp88s0`、连接名 `Robot-Network` 为例：

```bash
sudo nmcli connection modify "Robot-Network" \
  connection.interface-name enp88s0 \
  ipv4.method manual \
  ipv4.addresses 192.168.192.104/24 \
  ipv4.gateway "" \
  ipv4.dns "" \
  ipv6.method disabled
sudo nmcli connection up "Robot-Network"
```

验证：

```bash
ip -br addr show enp88s0
ip route get 192.168.192.201
ip route get 192.168.192.5
ip route get 192.168.192.11
ping -c 4 192.168.192.201
ping -c 4 192.168.192.5
ping -c 4 192.168.192.11
```

三个 `ip route get` 结果都必须包含 `dev enp88s0 src 192.168.192.104`，不能经过
Wi-Fi、VPN、Mihomo、`Meta` 或 TUN 接口。综合启动器默认检查 `enp88s0`；只有实际
硬件接口名不同才覆盖：

```bash
export SEER_AGV_INTERFACE=enx001122334455
```

## 3. 端口检查

CR5：

```bash
for port in 29999 30003 30005; do
  nc -zv -w 2 192.168.192.201 "${port}"
done
```

当前 CR5 的 30005 发送 1440 字节实时反馈帧。30004 只保留给其他控制器兼容使用；
它在这台 CR5 上可以完成 TCP 握手，但不会发送反馈字节。

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

- CR5 驱动运行时，GUI 不再额外探测 29999/30003/30005；
- GUI 不构造 `SeerClient`，只调用 `/seer_agv/*`；
- 启动器发现已有 `/seer_agv_node` 时直接复用。

## 6. 公共仓库中的敏感信息

SC3000 示例 FTP 用户名和密码是当前任务配置，不应直接用于生产网络。部署时应在
相机和程序两端同步更换，并通过网络隔离限制访问范围。
