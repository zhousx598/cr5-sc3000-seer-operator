# SEER AGV 地图、定位、导航与点动

## 1. 架构与端口

`seer_agv_node` 是唯一 TCP 所有者：

| 端口 | 类别 | 已使用 API |
|---:|---|---|
| 19204 | 状态 | 1000、1004～1007、1012、1020～1022、1050、1060、1101、1300、1301、1500 |
| 19205 | 控制 | 2000、2002、2003、2004、2010、2022 |
| 19206 | 导航 | 3003、3051 |
| 19207 | 配置 | 4011 |

GUI 不创建第二份 TCP 连接，只订阅状态、地图和站点，并调用 ROS 服务。

## 2. 启动

只读模式：

```bash
ros2 launch seer_agv_driver seer_agv.launch.py \
  host:=192.168.192.5 enable_cmd_vel:=false
```

确需 GUI 低速点动时：

```bash
ros2 launch seer_agv_driver seer_agv.launch.py \
  host:=192.168.192.5 enable_cmd_vel:=true
```

综合启动器会检查：有线接口物理载波、到 AGV 的路由、19204 端口，以及是否已经
存在 `/seer_agv_node`。它不会仅凭可能被 VPN 代理的 TCP 探测判定在线。

## 3. 实时地图和车体位置

驱动启动时通过 API 1300 查询当前/已存地图，API 4011 下载当前地图，并发布：

- `/seer_agv/map_markers`：RViz MarkerArray；
- `/seer_agv/map_data`：GUI 使用的精简 JSON，transient-local；
- `/seer_agv/stations`：站点 JSON，transient-local；
- `/seer_agv/pose`、`/seer_agv/odom`；
- TF `seer_map -> seer_base_link`；
- `/seer_agv/footprint_markers`：实时车体轮廓。

GUI 原生 QPainter 地图支持自动适配、滚轮缩放、右键拖动、左键选择重定位坐标，
并实时绘制车体轮廓和车头方向。

手动刷新：

```bash
ros2 service call /seer_agv/download_map std_srvs/srv/Trigger "{}"
```

## 4. 地图加载与重定位

加载控制器中已有地图使用 API 2022。GUI 要求状态新鲜、车辆静止、无导航任务，
并显示明确的人工确认对话框。切换后定位自动失效。

重定位使用 API 2002，输入 X、Y、车头角（GUI 中为度，驱动中为弧度）和搜索半径。
可直接在地图左键选取 X/Y。取消重定位使用 2004。

当前 Robokit 3.4.4.6 的定位状态：

| `reloc_status` | 含义 |
|---:|---|
| 1 | 定位有效，可进入其他安全门检查 |
| 2 | 正在重定位 |
| 3 | 重定位完成，等待操作员确认 |

驱动绝不会在启动或重定位结束后自动调用 2003。状态 3 时，操作员必须现场核对地图
中的车体位置和车头方向，再点击“确认定位正确”。请求包含 GUI 当时显示的位置，
驱动会再次比较最新位置并确认车辆静止，才发送 2003。

## 5. 运动安全门

点动或导航要求：

- 地图已加载且 `reloc_status=1`；
- 快速和低频状态均未过期；
- 无急停、充电、控制权抢占和报警；
- `blocked=false`、`slowed=false`；
- 导航与点动互斥。

点动限制：

- `|vx| <= 0.1 m/s`；
- 差速底盘 `vy=0`；
- `|w| <= 0.2 rad/s`；
- GUI 默认前后 `0.03 m/s`，旋转 `0.10 rad/s`；
- 按住按钮时每 100 ms 发布，松开立即发布零速度；
- 驱动看门狗超时后发送已确认的 API 2000 停止；
- 单个 API 2010 时长默认不超过 300 ms。

## 6. “前进无反应”的判断方法

已修复 GUI 之前使用全局鼠标状态导致按住前进被误判为松开的缺陷。现在绑定具体
`QPushButton.isDown()`，并在状态栏同时显示驱动已发送速度和 AGV 实测速度。

| 界面结果 | 判断 |
|---|---|
| 指令 `vx=+0.030`，实测 `vx>0` | 软件与底盘均正常 |
| 指令 `vx=+0.030`，实测约 0，`slowed=true` | 前向安全区拦截；清除障碍/检查传感器 |
| 指令一直为 0 | 点动未启用、按钮事件或安全门拒绝 |
| 指令非零、无 slowed、实测为 0 | 检查控制权、底盘方向配置和控制器日志 |

不允许为解决前进问题而删除 `slowed` 安全门。

## 7. 站点导航

GUI 从 API 1301 获取站点，通过 API 3051 发起导航，最大速度限制为 0.08 m/s。
取消导航使用 3003。开始导航前驱动先将点动目标置零并确认 2000 停止。

## 8. RViz

```bash
rviz2 -d install/seer_agv_driver/share/seer_agv_driver/config/seer_agv.rviz
```

固定坐标系为 `seer_map`。命名与 CR5 的 `base_link` 隔离，避免 TF 冲突。

