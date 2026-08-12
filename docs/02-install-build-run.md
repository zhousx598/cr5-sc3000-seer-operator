# 安装、构建和启动

## 1. 平台要求

- Ubuntu 22.04 x86_64；
- ROS 2 Humble Desktop；
- 系统 Python 3.10；
- 不需要 Conda。若安装了 Conda，构建和运行 ROS 时必须退出环境。

```bash
conda deactivate 2>/dev/null || true
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
python3 --version
```

## 2. 系统依赖

```bash
sudo apt update
sudo apt install -y \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python3-numpy \
  python3-opencv \
  python3-pyqt5 \
  python3-pytest \
  netcat-openbsd \
  network-manager \
  ros-humble-joint-state-publisher-gui \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-moveit \
  ros-humble-rviz2

/usr/bin/python3 -m pip install --user pyftpdlib
```

验证 OpenCV 包含 ArUco/AprilTag：

```bash
/usr/bin/python3 -c 'import cv2; print(cv2.__version__, hasattr(cv2, "aruco"))'
```

## 3. 准备源码

建议统一放在：

```bash
export DOBOT_WS="${HOME}/dobot_ws"
mkdir -p "${DOBOT_WS}/src" "${DOBOT_WS}/points/calibration_results" \
  "${DOBOT_WS}/queues"
cd "${DOBOT_WS}/src"
```

Dobot 官方 V3 ROS 2 驱动来源：

```bash
git clone https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V3.git
```

本项目还要求 `dobot_operator_gui`、`dobot_point_manager`、`seer_agv_driver` 和
`seer_agv_msgs` 位于同一个 `src/`。如果发布为单一 GitHub 仓库，应保留当前目录
结构；如果将 Dobot 官方包作为子模块，必须应用本项目记录的驱动修复。

## 4. rosdep 与构建

```bash
cd "${DOBOT_WS}"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

只构建综合上位机相关包：

```bash
colcon build --symlink-install --packages-up-to \
  seer_agv_driver dobot_operator_gui dobot_point_manager
```

`install/setup.bash` 不存在表示工作空间尚未成功构建。不要通过创建空文件绕过。

## 5. 环境变量

```bash
export DOBOT_WS="${HOME}/dobot_ws"
export DOBOT_TYPE=cr5
export IP_address=192.168.192.201
export DOBOT_FEEDBACK_PORT=30005
export SC3000_IP=192.168.192.11
export SC3000_FTP_USER=sc3000
export SC3000_FTP_PASSWORD='<相机任务中配置的FTP口令>'
export SEER_AGV_HOST=192.168.192.5
# 通常无需设置；启动器会从路由自动识别。需要锁定时再取消下一行注释：
# export SEER_AGV_INTERFACE=enp4s0
```

`DOBOT_WS` 未设置时，工具默认使用 `$HOME/dobot_ws`。
SC3000 程序保留设备出厂示例账号的兼容默认值；生产部署应通过上述环境变量覆盖，
并确保口令不写入 Git。

## 6. 启动顺序

终端 A，启动 CR5 驱动并保持运行：

```bash
source /opt/ros/humble/setup.bash
source "${DOBOT_WS}/install/setup.bash"
export DOBOT_TYPE=cr5
export IP_address=192.168.192.201
export DOBOT_FEEDBACK_PORT=30005
ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py
```

终端 B，启动综合上位机：

```bash
"${DOBOT_WS}/start_dobot_operator_gui.sh"
```

也可以直接运行 GUI，不自动管理 AGV 驱动：

```bash
source /opt/ros/humble/setup.bash
source "${DOBOT_WS}/install/setup.bash"
ros2 run dobot_operator_gui dobot_operator_gui
```

只启动 AGV 驱动：

```bash
ros2 launch seer_agv_driver seer_agv.launch.py \
  host:=192.168.192.5 enable_cmd_vel:=false
```

## 7. 桌面入口

创建 `~/Desktop/Dobot_CR5上位机.desktop`，其中 `Exec` 和 `Path` 改为实际工作区：

```ini
[Desktop Entry]
Type=Application
Version=1.0
Name=CR5·相机·AGV 综合上位机
Comment=CR5, DH-AG95, SC3000 and SEER AGV ROS 2 control panel
Exec=/home/USER/dobot_ws/start_dobot_operator_gui.sh
TryExec=/home/USER/dobot_ws/start_dobot_operator_gui.sh
Path=/home/USER/dobot_ws
Icon=applications-engineering
Terminal=false
Categories=Utility;Engineering;
StartupNotify=true
```

```bash
chmod +x ~/Desktop/Dobot_CR5上位机.desktop
chmod +x "${DOBOT_WS}/start_dobot_operator_gui.sh"
```

启动日志：

```text
~/.ros/dobot_operator_gui_launcher.log
~/.ros/seer_agv_integrated_driver.log
~/.ros/log/<launch-time>/
```

## 8. RViz/MoveIt 基础检查

```bash
ros2 launch dobot_rviz dobot_rviz.launch.py gui:=true
ros2 launch cr5_moveit demo.launch.py
```

这两条命令验证模型与 MoveIt 环境，不代表已经连接真机。缺少
`joint_state_publisher_gui` 或 `controller_manager` 时返回本页第 2 节安装依赖。
