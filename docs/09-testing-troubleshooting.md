# 测试、验收与故障排查

## 1. 离线测试

```bash
export DOBOT_WS="${HOME}/dobot_ws"
cd "${DOBOT_WS}"
source /opt/ros/humble/setup.bash

python3 -m py_compile \
  src/dobot_operator_gui/dobot_operator_gui/*.py \
  src/seer_agv_driver/seer_agv_driver/*.py

colcon build --symlink-install --packages-up-to \
  seer_agv_driver dobot_operator_gui dobot_point_manager
source install/setup.bash

/usr/bin/python3 -m pytest -q \
  src/DOBOT_6Axis_ROS2_V3/dobot_bringup_v3/test/test_connection_recovery.py \
  src/DOBOT_6Axis_ROS2_V3/dobot_bringup_v3/test/test_modbus_commands.py \
  src/DOBOT_6Axis_ROS2_V3/dobot_bringup_v3/test/test_safeskin_commands.py \
  src/dobot_point_manager/test \
  src/dobot_operator_gui/test \
  src/seer_agv_driver/test
```

这里明确列出 Dobot 驱动的功能用例，不直接传入其整个 `test/` 目录。上游
`test_flake8.py` 和 `test_pep257.py` 使用工作目录全局扫描，若从工作空间根目录运行，
会把 `build/` 生成代码和同仓库其他厂家包一起扫入，因此不适合作为本项目功能
回归结果。需要做风格检查时，应先排除 `build/`、`install/`、`log/`，再按具体包
单独运行。

测试数量会随新增用例变化，应以 `pytest` 实际结果为准。
本次文档整理后的基线为 **106 passed**。

## 2. 真机分阶段验收

每阶段通过后才进入下一阶段：

1. 物理接线、电压、负载、实体急停；
2. IP、路由、ping 和端口；
3. 只读模式/角度/位姿/报警；
4. 下使能或静止状态下的相机与地图显示；
5. 夹爪低力、空载、短行程；
6. CR5 低速单点短运动；
7. 拖动取点并恢复保护；
8. AGV 定位核对后低速单方向点动；
9. 站点导航；
10. 最后才运行组合队列或视觉抓取。

## 3. ROS 环境问题

### `install/setup.bash` 不存在

工作空间未构建或构建失败：

```bash
cd "${DOBOT_WS}"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### `Package 'dobot_point_manager' not found`

```bash
cd "${DOBOT_WS}"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dobot_point_manager
source install/setup.bash
ros2 pkg prefix dobot_point_manager
```

切换目录不能解决未构建/未 source 的包。

### 缺少 `joint_state_publisher_gui` / `controller_manager`

返回[安装文档](02-install-build-run.md)安装对应 Humble 包，然后重新 source。

## 4. CR5 驱动问题

### `ConnectionResetError: [Errno 104]`

- 检查是否同时运行 DobotStudio、`nc` 或第二份驱动；
- 检查 Wi-Fi 丢包/有线载波；
- 停止整个 launch 后重启，不能只继续调用已经死亡的服务；
- 查看 bringup 重连日志。

### `feedback` 的 `feed_v` 属性不存在

这是旧驱动在 30004 初次连接失败后仍进入定时回调造成的二次异常。本项目已修复
为 `None` 防护和定时重连。若仍出现，说明运行的是旧 `install/`，请重新构建并
重启 launch。

### `/joint_states_robot` 全为 0

先比较：

```bash
ros2 service call /dobot_bringup_v3/srv/GetAngle \
  dobot_msgs_v3/srv/GetAngle "{}"
ros2 topic echo /joint_states_robot --once
```

若 GetAngle 非零但话题全零，检查 30004 反馈节点、完整帧 magic 和是否运行旧版本。

### `Sync` 超时且服务全部卡住

不要加大客户端超时。停止并重启 bringup，检查模式和报警，然后使用本项目轮询式
点位程序。阻塞的服务端 `recv()` 不会被客户端超时取消。

### 模式 9 / 10 / 11

- 9：查询报警，排除根因，再人工清警；
- 10：确认没有遗留运动队列后才允许 Continue；
- 11：排除真实碰撞/接触并确认安全功能状态。

## 5. SC3000 问题

### 触发成功但收不到图片

依次检查：

1. PC 有线地址是否为相机 FTP 目标 IP；
2. 相机是否处于循环运行态；
3. 502 可达；
4. 2121 正在监听且没有第二个 FTP Server；
5. 30000～30009 已放行；
6. 相机 FTP 用户、密码和目标目录权限；
7. `/dev/shm` 或保存目录是否可写。

### `AMENT_TRACE_SETUP_FILES: 未绑定的变量`

不要在 source ROS 环境前启用 `set -u`。本项目快捷脚本先 source，再启用 nounset。

### AprilTag 深度比例错误

核对输入的是黑框真实边长。将 58.5 mm 错写成 5.85 mm 会造成约 10 倍尺度错误。
同时核对图像分辨率必须与内参文件一致。

### 手眼外参看起来不符合机械尺寸

先确认矩阵方向是 `tool0_T_camera`，平移是在 Tool0 坐标轴中表达，不等于用尺子
沿世界 Z 轴测量的距离。再用固定标记、多姿态数据检查 `base_T_tag` 一致性；支架
移动、Tool/User 配置变化、旋转变化不足都会使结果失效。

## 6. AGV 问题

### 地图为空或车体位置不更新

```bash
ros2 node list | grep seer_agv_node
ros2 topic echo /seer_agv/status --once
ros2 topic info -v /seer_agv/map_data
ros2 service call /seer_agv/download_map std_srvs/srv/Trigger "{}"
```

若网线在 GUI 启动时断开，启动器不会启动 AGV 驱动；接好后关闭并重新打开 GUI。

### `localization awaiting operator confirmation`

当前 Robokit 的 `reloc_status=3`。在 GUI 地图中核对 X/Y/车头方向，确认真实现场
位置一致后，使用受保护的 API 2003 按钮。不要自动确认。

### 前进无反应，后退和旋转正常

观察 GUI 的“指令 vx”、实际 `vx` 和 `slowed`。若正向指令已发送而 `slowed=true`，
前向安全传感器正在正确拦截。清除障碍、清洁/检查传感器和安全区配置，不得改代码
绕过。若指令始终为零，确认正在使用重建后的 GUI，且按住按钮期间安全门保持开启。

### VPN/TUN 导致端口假阳性

```bash
ip -br link show enp4s0
ip route get 192.168.192.5
```

必须显示有线载波和正确 `dev enp4s0`。启动器已经同时检查载波、路由和端口。

## 7. 上位机打不开

```bash
tail -n 200 ~/.ros/dobot_operator_gui_launcher.log
ps -ef | grep -E 'dobot_operator_gui|seer_agv_node'
```

常见原因是 Snap/Conda 注入 Qt 插件路径、工作空间未 source 或重复 GUI。综合启动器
会清除 `PYTHONHOME`、`PYTHONPATH` 和 Qt/Snap 污染变量，再使用系统 Python。

## 8. 提交问题时应附带

- Ubuntu、ROS、Robokit/CR5 固件版本；
- 使用的 commit；
- 网络拓扑和已脱敏的 `ip -br addr` / `ip route get`；
- 完整启动命令；
- `ros2 node list`、相关 service/topic 类型；
- 从首次异常开始的完整日志，而不是只有最后一行；
- 是否连接真机、是否使能、是否真的发生运动；
- 不包含密码、生产地图和隐私图像的最小复现数据。
