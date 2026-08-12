# CR5、SC3000 与 SEER AGV 统一 ROS 2 工作空间

> 本文件是 2026-08-11 早期集成快照。可复用的 GitHub 入口、最新功能和完整部署
> 说明已统一整理到 [README.md](README.md)，请优先阅读新文档。

更新时间：2026-08-11

## 目录职责

- `src/DOBOT_6Axis_ROS2_V3`：CR5 驱动、消息、MoveIt 和模型。
- `src/dobot_point_manager`：机械臂点位运动。
- `src/dobot_operator_gui`：CR5、DH-AG95、SC3000、AprilTag 与 AGV 综合上位机。
- `src/seer_agv_driver`：SEER AMB-300 的唯一 TCP 通信节点。
- `src/seer_agv_msgs`：AGV 自定义 ROS 2 服务接口。
- `points`：机械臂点位、同步位姿与图像数据。
- `backups`：修改前备份；有 `COLCON_IGNORE`，不会参与构建。

## 模块隔离规则

1. 只有 `/seer_agv_node` 可以连接 AGV 的 19204、19205、19206、19207 端口。
2. 综合上位机只通过 ROS 2 控制 AGV，不创建 `SeerClient`。
3. AGV 使用 `/seer_agv/*` 话题和服务；CR5 继续使用 `/dobot_bringup_v3/*`。
4. AGV TF 为 `seer_map -> seer_base_link`，避免与机械臂 `base_link` 冲突。
5. AGV 点动话题为 `/seer_agv/cmd_vel`，不使用全局 `/cmd_vel`。
6. 机械臂服务和 AGV 服务分别加锁；普通界面操作仍按安全顺序串行执行。

## 构建

不要在 Conda 环境的 Python 3.13 中构建 ROS 2 Humble。使用系统 Python 3.10：

```bash
cd /home/zsx/dobot_ws
unset PYTHONHOME
unset PYTHONPATH
source /opt/ros/humble/setup.bash
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
colcon build --symlink-install --packages-select \
  seer_agv_msgs seer_agv_driver dobot_operator_gui
source install/setup.bash
```

## 启动

桌面双击“CR5·相机·AGV 综合上位机”，或执行：

```bash
/home/zsx/dobot_ws/start_dobot_operator_gui.sh
```

启动脚本行为：

- 若 `192.168.192.5:19204` 可达且没有 `/seer_agv_node`，启动 AGV 驱动；
- 若已有 `/seer_agv_node`，直接复用，不启动第二套 TCP 连接；
- AGV 不可达时仍允许使用机械臂、夹爪和相机页面；
- 上位机退出时发布零速度，并向自己启动的 AGV 驱动请求 `2000` 停止。

单独只读启动 AGV 驱动：

```bash
source /opt/ros/humble/setup.bash
source /home/zsx/dobot_ws/install/setup.bash
ros2 launch seer_agv_driver seer_agv.launch.py enable_cmd_vel:=false
```

## AGV 安全门

只有以下状态全部满足时才允许运动：

- 定位状态为 `reloc_status=1`；
- 地图已经载入；
- 急停、驱动器急停和软件急停均未激活；
- AGV 未处于充电状态；
- 控制权未被其他客户端抢占；
- 未被阻挡或减速；
- 无 fatal、error 或 warning；
- 快速和低频状态均未过期。

导航运行时，点动安全门自动关闭。`reloc_status=3` 在当前 Robokit
`3.4.4.6` 上表示“重定位完成但尚未由操作员确认”，驱动不会自动调用 2003，
必须在 AGV 侧人工确认后变为状态 1。

## 2026-08-11 验证结果

- 有线地址：PC `192.168.192.104/24`，AGV `192.168.192.5`。
- Ping：0% 丢包，约 0.25～0.48 ms。
- 19204、19206、19207 可连接。
- 机器人模型：宽 0.700 m，头部 0.520 m，尾部 0.480 m。
- 地图下载成功，共发布 33 个 RViz Marker。
- 站点：LM1、LM2、LM3、LM4。
- 当前实机状态：地图已载入、无报警、无急停、未充电、控制权未抢占，
  但 `reloc_status=3`，因此运动被正确禁止。
- 协议与安全单元测试：17 项通过。
- 统一工作空间全部相关测试：76 项通过。
- 上位机离屏集成测试通过，能显示 AGV 状态与4个站点。
- 导航拒绝测试通过：状态3时请求 LM2 返回
  `localization awaiting operator confirmation`，未下发 3051。
- AGV 驱动 Ctrl-C 后干净退出，无重复 `rclpy.shutdown()` 错误。

## 尚未执行

本轮没有发送非零 `2010` 点动，也没有执行 `3051` 实机导航。完成定位人工确认、
清空现场并确认实体急停可用后，才应在上位机中进行低速实机运动验证。
