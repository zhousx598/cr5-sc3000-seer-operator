# 源码、接口与测试文件清单

本文给出当前工作空间中本项目直接新增、修改和调用的代码入口。完整实现以链接到的
源文件为准；不在 Markdown 中复制源代码，以避免形成无法同步维护的第二份实现。

## 1. 根入口

| 文件 | 作用 |
|---|---|
| [`start_dobot_operator_gui.sh`](../start_dobot_operator_gui.sh) | 环境净化、AGV 唯一驱动检查、GUI 启停和退出停车 |
| [`estimate_apriltag_pose.sh`](../estimate_apriltag_pose.sh) | 从终端触发单帧并估计 AprilTag 相机位姿 |
| [`.gitignore`](../.gitignore) | 隔离构建产物、现场数据和标定数据 |

## 2. Dobot 上游驱动中的本项目修改

上游包目录为 [`DOBOT_6Axis_ROS2_V3`](../src/DOBOT_6Axis_ROS2_V3/)。其余 CR/Nova
URDF、RViz、Gazebo 和 MoveIt 文件保持设备方包结构；本项目直接修复或新增的是：

| 文件 | 作用 |
|---|---|
| [`dobot_api.py`](../src/DOBOT_6Axis_ROS2_V3/dobot_bringup_v3/dobot_bringup_v3/dobot_api.py) | Dashboard/运动 socket、Modbus 和 SafeSkin 命令 |
| [`dobot_bringup.py`](../src/DOBOT_6Axis_ROS2_V3/dobot_bringup_v3/dobot_bringup_v3/dobot_bringup.py) | ROS 服务、连接恢复、报警解析和异常边界 |
| [`feedback.py`](../src/DOBOT_6Axis_ROS2_V3/dobot_bringup_v3/dobot_bringup_v3/feedback.py) | 30004 完整帧、重连和状态发布 |
| [`SetSafeSkin.srv`](../src/DOBOT_6Axis_ROS2_V3/dobot_msgs_v3/srv/SetSafeSkin.srv) | 电子皮肤开关服务定义 |
| [`test_connection_recovery.py`](../src/DOBOT_6Axis_ROS2_V3/dobot_bringup_v3/test/test_connection_recovery.py) | socket/节点恢复回归 |
| [`test_modbus_commands.py`](../src/DOBOT_6Axis_ROS2_V3/dobot_bringup_v3/test/test_modbus_commands.py) | Modbus 帧与参数校验 |
| [`test_safeskin_commands.py`](../src/DOBOT_6Axis_ROS2_V3/dobot_bringup_v3/test/test_safeskin_commands.py) | SafeSkin 命令和 ROS 回调 |

所有 Dobot ROS 服务定义位于
[`dobot_msgs_v3/srv`](../src/DOBOT_6Axis_ROS2_V3/dobot_msgs_v3/srv/)，实际使用的
服务和示例见 [API 参考](07-api-reference.md)。

## 3. 安全点位管理包

包目录：[`dobot_point_manager`](../src/dobot_point_manager/)。

| 文件 | 作用 |
|---|---|
| [`move_between_points.py`](../src/dobot_point_manager/dobot_point_manager/move_between_points.py) | 点位解析、起点验证、运动下发、角度/模式轮询和失败诊断 |
| [`test_point_files.py`](../src/dobot_point_manager/test/test_point_files.py) | 点位格式、角度差和运动状态机测试 |
| [`setup.py`](../src/dobot_point_manager/setup.py) | 注册 `move_between_points` CLI |
| [`package.xml`](../src/dobot_point_manager/package.xml) | ROS 依赖和包元数据 |

## 4. 综合上位机包

包目录：[`dobot_operator_gui`](../src/dobot_operator_gui/)。

### 运行代码

| 文件 | 作用 |
|---|---|
| [`main.py`](../src/dobot_operator_gui/dobot_operator_gui/main.py) | ROS/Qt 进程入口 |
| [`main_window.py`](../src/dobot_operator_gui/dobot_operator_gui/main_window.py) | 页面、按钮、异步任务和安全回滚 |
| [`ros_client.py`](../src/dobot_operator_gui/dobot_operator_gui/ros_client.py) | CR5、夹爪和 AGV ROS 门面 |
| [`core.py`](../src/dobot_operator_gui/dobot_operator_gui/core.py) | 连通检查、点位/采集组原子写入 |
| [`camera_capture.py`](../src/dobot_operator_gui/dobot_operator_gui/camera_capture.py) | SC3000 FTP + Modbus 触发和预览帧 |
| [`apriltag_localization.py`](../src/dobot_operator_gui/dobot_operator_gui/apriltag_localization.py) | 标签检测、PnP 和图像叠加 |
| [`handeye_transform.py`](../src/dobot_operator_gui/dobot_operator_gui/handeye_transform.py) | `tool0_T_camera` 与基座坐标换算 |
| [`task_queue.py`](../src/dobot_operator_gui/dobot_operator_gui/task_queue.py) | 队列 schema、校验和串行执行 |
| [`agv_map_widget.py`](../src/dobot_operator_gui/dobot_operator_gui/agv_map_widget.py) | 地图绘制、交互和车体位置 |

### 标定和诊断工具

| 文件 | 作用 |
|---|---|
| [`calibrate_sc3000.py`](../src/dobot_operator_gui/tools/calibrate_sc3000.py) | 7×5 棋盘角点和相机内参 |
| [`compare_sc3000_models.py`](../src/dobot_operator_gui/tools/compare_sc3000_models.py) | 畸变模型与留一法对比 |
| [`estimate_apriltag_pose.py`](../src/dobot_operator_gui/tools/estimate_apriltag_pose.py) | 单图/实时 AprilTag PnP |
| [`calibrate_handeye_first20.py`](../src/dobot_operator_gui/tools/calibrate_handeye_first20.py) | 眼在手上多算法标定和一致性统计 |

### 测试

`test/` 中的九个用例文件分别覆盖 AGV ROS 桥、AprilTag、相机协议、采点核心、拖动
保护、夹爪请求、手眼变换、Qt 插件环境和任务队列：

[`test_agv_ros_bridge.py`](../src/dobot_operator_gui/test/test_agv_ros_bridge.py)、
[`test_apriltag_localization.py`](../src/dobot_operator_gui/test/test_apriltag_localization.py)、
[`test_camera_capture.py`](../src/dobot_operator_gui/test/test_camera_capture.py)、
[`test_core.py`](../src/dobot_operator_gui/test/test_core.py)、
[`test_drag_safety.py`](../src/dobot_operator_gui/test/test_drag_safety.py)、
[`test_gripper_requests.py`](../src/dobot_operator_gui/test/test_gripper_requests.py)、
[`test_handeye_transform.py`](../src/dobot_operator_gui/test/test_handeye_transform.py)、
[`test_qt_plugin_path.py`](../src/dobot_operator_gui/test/test_qt_plugin_path.py)、
[`test_task_queue.py`](../src/dobot_operator_gui/test/test_task_queue.py)。

## 5. SEER AGV 驱动与接口

包目录：[`seer_agv_driver`](../src/seer_agv_driver/) 和
[`seer_agv_msgs`](../src/seer_agv_msgs/)。

| 文件 | 作用 |
|---|---|
| [`seer_client.py`](../src/seer_agv_driver/seer_agv_driver/seer_client.py) | SEER TCP 帧、API 调用和连接锁 |
| [`seer_agv_node.py`](../src/seer_agv_driver/seer_agv_driver/seer_agv_node.py) | 状态、地图、TF、导航、重定位和速度安全门 |
| [`seer_keyboard_teleop.py`](../src/seer_agv_driver/seer_agv_driver/seer_keyboard_teleop.py) | 独立人工低速测试 |
| [`seer_agv_panel.py`](../src/seer_agv_driver/seer_agv_driver/seer_agv_panel.py) | 早期面板，仅保留源码，不作为安装入口 |
| [`seer_agv.launch.py`](../src/seer_agv_driver/launch/seer_agv.launch.py) | ROS launch |
| [`seer_agv.yaml`](../src/seer_agv_driver/config/seer_agv.yaml) | 默认安全、周期、速度和 frame 参数 |
| [`seer_agv.rviz`](../src/seer_agv_driver/config/seer_agv.rviz) | RViz 配置 |
| [`test_seer_client.py`](../src/seer_agv_driver/test/test_seer_client.py) | TCP 客户端、状态与控制测试 |
| [`test_map_snapshot.py`](../src/seer_agv_driver/test/test_map_snapshot.py) | 地图快照和坐标变换测试 |

自定义服务：

- [`ConfirmLocalization.srv`](../src/seer_agv_msgs/srv/ConfirmLocalization.srv)；
- [`LoadMap.srv`](../src/seer_agv_msgs/srv/LoadMap.srv)；
- [`NavigateToStation.srv`](../src/seer_agv_msgs/srv/NavigateToStation.srv)；
- [`Relocalize.srv`](../src/seer_agv_msgs/srv/Relocalize.srv)。

## 6. 运行期文件约定

以下目录由程序在部署机生成，不是可复用源码：

- `points/`：点位、图像、内参和手眼结果；
- `queues/`：任务队列；
- `apriltag_validation/`：外参验证数据；
- `build/`、`install/`、`log/`：colcon/ROS 产物。

这些目录已由 `.gitignore` 排除。新设备必须重新生成标定和现场数据。
