# ROS 2 与设备 API 参考

## 1. CR5：GUI 实际调用的服务

统一前缀：`/dobot_bringup_v3/srv/`。

| 服务 | 类型 | 用途 |
|---|---|---|
| RobotMode | `dobot_msgs_v3/srv/RobotMode` | 模式 |
| GetAngle | `GetAngle` | 六轴角度，单位度 |
| GetPose | `GetPose` | User/Tool 下 XYZ mm、RPY deg |
| GetErrorID | `GetErrorID` | 嵌套报警数组 |
| ClearError | `ClearError` | 清除已排除原因的报警 |
| EnableRobot / DisableRobot | 同名类型 | 使能/下使能 |
| SpeedFactor | `SpeedFactor` | 全局速度百分比 |
| JointMovJ | `JointMovJ` | 关节目标和动态参数 |
| StartDrag / StopDrag | 同名类型 | 拖动模式 |
| SetCollisionLevel | 同名类型 | 本体碰撞等级 0～5 |
| SetSafeSkin | 同名类型 | SafeSkin：0 关，1 开 |
| Continues | `Continues` | 明确确认后恢复暂停 |
| EmergencyStop | 同名类型 | 软件急停请求 |
| ModbusCreate / ModbusClose | 同名类型 | 夹爪 RTU 通道 |
| SetHoldRegs / GetHoldRegs | 同名类型 | 夹爪寄存器 |

所有返回中 `res=0` 只表示控制器接受/完成该 API，不表示物理动作已经完成。运动和
夹爪动作都必须继续读取状态确认。

## 2. CR5 反馈话题

设备方驱动发布的常用话题包括：

- `/joint_states_robot`；
- `/dobot_msgs_v3/msg/ToolVectorActual`。

控制与取点优先使用 `GetAngle` 和 `GetPose` 服务，因为本项目可校验返回码并得到
同一次操作上下文；可用话题做独立交叉检查。

## 3. AGV 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/seer_agv/status` | `std_msgs/String` | 状态、安全门、位姿、速度、指令 JSON |
| `/seer_agv/pose` | `geometry_msgs/PoseStamped` | 地图定位位姿 |
| `/seer_agv/odom` | `nav_msgs/Odometry` | 地图定位形式的里程信息 |
| `/seer_agv/cmd_vel` | `geometry_msgs/Twist` | 安全门控点动输入 |
| `/seer_agv/stations` | `std_msgs/String` | 站点 JSON，持久化 |
| `/seer_agv/map_data` | `std_msgs/String` | GUI 地图 JSON，持久化 |
| `/seer_agv/map_markers` | `visualization_msgs/MarkerArray` | RViz 地图 |
| `/seer_agv/footprint_markers` | `MarkerArray` | 实时车体轮廓 |

`status.command.sent_vx/sent_w` 是驱动实际尝试下发的速度；`status.speed.vx/w` 是
AGV 反馈。二者用于区分 GUI 问题与底盘安全拦截。

## 4. AGV 服务

| 服务 | 类型 | 底层 API |
|---|---|---:|
| `/seer_agv/stop` | `std_srvs/Trigger` | 2000 |
| `/seer_agv/download_map` | `std_srvs/Trigger` | 1300 + 4011 |
| `/seer_agv/load_map` | `seer_agv_msgs/LoadMap` | 2022 |
| `/seer_agv/relocalize` | `seer_agv_msgs/Relocalize` | 2002 |
| `/seer_agv/cancel_relocalization` | `std_srvs/Trigger` | 2004 |
| `/seer_agv/confirm_localization` | `ConfirmLocalization` | 2003 |
| `/seer_agv/navigate_to_station` | `NavigateToStation` | 3051 |
| `/seer_agv/cancel_nav` | `std_srvs/Trigger` | 3003 |

危险状态改变服务含 `operator_confirmed` 和/或 GUI 位姿快照，不能用来绕过现场确认。

## 5. SC3000 协议

| 方向 | 协议 | 端口 | 用途 |
|---|---|---:|---|
| PC → SC3000 | Modbus TCP | 502 | 触发与状态读取 |
| SC3000 → PC | FTP | 2121 | 上传图片 |
| SC3000 → PC | FTP passive | 30000～30009 | 数据连接 |

Modbus 使用功能码 0x06 写单个 holding register、0x03 读 holding register。

## 6. 数据文件

点位文件保存原始 ROS 风格响应文本，解析器只接受 6 个有限数值。采集 JSON 包含：

- 相机 IP、触发/收图时间和状态寄存器；
- 拍照前后关节角、Tool0 位姿、RobotMode 和最大漂移；
- 图像文件名；
- AprilTag 相机坐标检测；
- 可用时的 CR5 基座坐标及手眼标定文件来源。

队列 JSON 的 schema 见[夹爪与任务队列](05-dh-ag95-and-queue.md)。

