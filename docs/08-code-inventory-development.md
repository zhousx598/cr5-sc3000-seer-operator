# 代码目录、修改记录与扩展方法

## 1. 代码所有权边界

`src/DOBOT_6Axis_ROS2_V3` 来源于 Dobot 官方仓库，本项目在其上做了连接恢复、
报警解析、Modbus 和 SafeSkin 修复。其余四个 ROS 包及综合启动器是本项目新增。
发布到 GitHub 前应分别核对上游许可证和本项目文件许可证，不能用一个许可证覆盖
不兼容的厂家代码或手册。

源码文件本身是唯一权威实现；本文提供索引，不在 Markdown 中复制几千行源码，
避免文档副本与实际程序分叉。

## 2. Dobot 官方包中的修改

| 文件 | 修改内容 |
|---|---|
| `dobot_bringup_v3/dobot_api.py` | socket 生命周期、`sendall`、断线传播、Modbus 参数校验、`SetSafeSkin` |
| `dobot_bringup_v3/dobot_bringup.py` | 原子连接/重连、服务未连接保护、网络异常返回、`GetErrorID` 解析、SafeSkin 服务 |
| `dobot_bringup_v3/feedback.py` | `feed_v=None` 防护、定时重连、完整 1440 字节帧重组、断线关闭 |
| `dobot_msgs_v3/srv/SetSafeSkin.srv` | 新增 `status -> res` 服务定义 |
| `dobot_bringup_v3/test/test_connection_recovery.py` | 连接和反馈恢复测试 |
| `dobot_bringup_v3/test/test_modbus_commands.py` | Modbus 序列化/校验测试 |
| `dobot_bringup_v3/test/test_safeskin_commands.py` | SafeSkin API 和 ROS 回调测试 |

重要设计：网络错误后不自动重发 `JointMovJ`。发送失败时无法确定控制器是否已经
接收，自动重发可能造成重复运动。

## 3. `dobot_point_manager`

| 文件 | 职责 |
|---|---|
| `move_between_points.py` | 点位文件解析、起点校验、低速 `JointMovJ`、轮询到位、模式/报警诊断 |
| `test/test_point_files.py` | 文件名、格式、角度环绕和运动状态测试 |
| `setup.py` | 注册 `move_between_points` 命令 |

扩展新运动类型时仍应保留：默认不运动、显式 `--execute`、模式检查、超时、最终
位置确认和不自动清警。

## 4. `dobot_operator_gui`

| 文件 | 职责 |
|---|---|
| `main.py` | 创建 rclpy 多线程执行器和 Qt 应用 |
| `main_window.py` | 五个综合页面、人工确认、后台任务、预览、关闭恢复逻辑 |
| `ros_client.py` | CR5/AGV 类型化 ROS 门面、服务串行化、运动/夹爪状态机 |
| `core.py` | 点位解析、原子保存、采集组格式、TCP 检查 |
| `camera_capture.py` | 内嵌 FTP Server、Modbus 触发、完整图片检测 |
| `apriltag_localization.py` | AprilTag 检测、IPPE + LM、叠加显示 |
| `handeye_transform.py` | Tool0/相机/标签刚体变换及校验 |
| `task_queue.py` | 版本化 JSON、严格参数校验、串行执行 |
| `agv_map_widget.py` | 原生 Qt 地图、缩放/平移/点选和车体姿态 |

工具：

| 文件 | 职责 |
|---|---|
| `tools/calibrate_sc3000.py` | 棋盘内参与逐图诊断 |
| `tools/compare_sc3000_models.py` | 六种畸变约束和留一法比较 |
| `tools/estimate_apriltag_pose.py` | 图片/实时单帧 AprilTag 位姿 |
| `tools/calibrate_handeye_first20.py` | 眼在手上标定和多算法一致性验证 |

测试目录覆盖点位、拖动保护回滚、Modbus 请求、队列、相机、AprilTag、手眼变换、
Qt 环境和 AGV ROS 桥。`ros_client.py` 还会缓存 30005 驱动发布的关节角和末端位姿；
当 Dashboard 的 `GetAngle`/`GetPose` 被控制器拒绝时，仅在反馈未超过两秒的前提下
用于界面状态兜底，过期数据仍按失败处理。

## 5. `seer_agv_driver` 和 `seer_agv_msgs`

| 文件 | 职责 |
|---|---|
| `seer_client.py` | SEER 二进制 TCP 帧、API 号、响应校验、端口锁和持久控制连接 |
| `seer_agv_node.py` | 状态轮询、安全门、速度看门狗、地图/TF/服务 |
| `seer_keyboard_teleop.py` | 独立低速键盘测试工具 |
| `config/seer_agv.yaml` | 状态周期、超时、速度和坐标系限制 |
| `launch/seer_agv.launch.py` | 驱动启动参数 |
| `seer_agv_msgs/srv/*.srv` | 导航、地图、重定位和定位确认接口 |

`seer_agv_panel.py` 是早期独立面板源码，不安装为入口，避免它创建第二套连接。

## 6. 综合启动器

`start_dobot_operator_gui.sh`：

- 清除 Snap/Conda 可能污染 PyQt/ROS 的环境变量；
- source Humble 和工作空间；
- 检查 AGV 有线载波、路由、端口和现有 ROS 节点；
- 必要时启动唯一 AGV 驱动；
- 启动 GUI；
- 退出时发布零速度并请求 AGV 2000 停止。

可配置变量：`DOBOT_WS`、`SEER_AGV_HOST`、`SEER_AGV_INTERFACE`、`SC3000_IP`。

## 7. 主要外部算法/库调用

| 库 | 调用 | 用途 |
|---|---|---|
| ROS 2 `rclpy` | Node、service/client、publisher/subscriber、executor | 设备解耦 |
| PyQt5 | QWidget、QPainter、QThread/信号、QTimer | GUI 和非阻塞刷新 |
| OpenCV | `findChessboardCornersSB`、`calibrateCameraExtended` | 内参标定 |
| OpenCV ArUco | AprilTag dictionaries、corner refinement | 标签检测 |
| OpenCV PnP | IPPE square、LM refine、projectPoints | 2.5D 位姿 |
| OpenCV hand-eye | `calibrateHandEye` 五种方法 | 眼在手上外参 |
| NumPy | 矩阵、刚体变换、统计 | 几何计算 |
| pyftpdlib | FTPServer、被动端口 | SC3000 收图 |
| Python socket | Dobot/SEER/Modbus TCP | 协议传输 |

## 8. 开发里程碑

1. 安装 ROS 2 Humble 依赖，完成 CR5 RViz/MoveIt 验证；
2. 打通 CR5 V3 三端口并识别 RobotMode/报警/关节反馈问题；
3. 修复驱动断线、反馈帧、GetErrorID 和 Modbus；
4. 用轮询替代阻塞 `Sync`，建立安全点位管理包；
5. 接入末端 RS485 DH-AG95；
6. 建立 PyQt 综合上位机、拖动保护和任务队列；
7. 以 FTP + Modbus TCP 接入 SC3000 预览和同步采集；
8. 完成棋盘内参、AprilTag PnP、前 20 组手眼标定和基座坐标输出；
9. 将 SEER AGV TCP 驱动迁入统一工作空间，加入状态安全门、地图与导航；
10. 增加地图加载/重定位、实时车体显示并修复前进按压逻辑。

## 9. 继续扩展的推荐接口

- 相机 ROS 化：发布 `sensor_msgs/Image`、`CameraInfo` 和 `PoseStamped`；
- 抓取：单独标定 `tag_T_grasp`，先只预览目标，再增加规划与执行确认；
- AGV：将地图 JSON 转成标准 `nav_msgs/OccupancyGrid`；
- 配置：把设备地址、FTP 凭据和标定文件统一移到 YAML/ROS 参数；
- CI：无硬件运行解析、协议帧、算法和 Qt 离屏测试，硬件测试保持人工触发。
