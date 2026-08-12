# dobot_point_manager

按点名读取 `~/dobot_ws/points/<点名>_joint.txt`，通过 Dobot V3 ROS 2
服务从一个已确认的关节点移动到另一个关节点。

程序不会自动执行以下操作：

- 清除报警；
- 使能机械臂；
- 修改碰撞检测；
- 退出拖动模式；
- 将机械臂移动到指定的起点。

执行运动前，程序要求：

1. `RobotMode` 必须等于 5；
2. `GetAngle` 必须与 `--from` 点逐关节相差不超过指定容差；
3. 所有 ROS 服务调用必须返回 `res=0`；
4. `JointMovJ` 下发后，程序轮询 `GetAngle` 和 `RobotMode`；
5. 机械臂进入目标容差且回到 Mode 5 后，才判定运动完成。

程序不调用 Dobot 的阻塞式 `Sync` ROS 服务。该服务会占用当前 V3
bringup 的单线程回调；控制器队列暂停时，它可能连带堵住所有其他服务。

## 构建

```bash
cd ~/dobot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dobot_point_manager
source ~/dobot_ws/install/setup.bash
```

## 预览 P1 到 P2

默认不运动：

```bash
ros2 run dobot_point_manager move_between_points --from P1 --to P2
```

## 实际执行

```bash
ros2 run dobot_point_manager move_between_points \
  --from P1 --to P2 --execute
```

默认参数：

```text
SpeedFactor=5%
SpeedJ=10%
AccJ=10%
逐关节容差=0.5°
等待开始超时=10 秒
运动完成总超时=300 秒
```

反向运动：

```bash
ros2 run dobot_point_manager move_between_points \
  --from P2 --to P1 --execute
```

查看全部选项：

```bash
ros2 run dobot_point_manager move_between_points --help
```

## “命令已接受，但机械臂没有开始运动”

这表示 `JointMovJ` 返回了成功，但实际关节角没有变化。常见原因是控制器
运动队列暂停/堵塞，或者运动下发后出现报警。程序会在 10 秒内停止等待并
报告最后的 `RobotMode`、关节角和目标误差，不会自动清警或恢复队列。

如果此前已经调用过 `Sync` 并超时，先停止并重新启动 bringup；客户端超时
不会解除驱动端正在等待的阻塞回调。

检查报警并确认工作区安全后，才可以显式恢复队列：

```bash
ros2 service call /dobot_bringup_v3/srv/Continue \
  dobot_msgs_v3/srv/Continues "{}"
```

注意：`Continue` 可能立即执行控制器中此前遗留的运动命令，因此不要在人员、
工具或障碍物位于机械臂工作区时调用。
