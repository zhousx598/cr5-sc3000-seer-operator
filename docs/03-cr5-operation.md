# CR5 驱动、拖动取点与点位运动

## 1. 驱动连接

CR5 V3 使用三个 TCP 端口：

| 端口 | 用途 |
|---:|---|
| 29999 | Dashboard：模式、使能、报警、配置、Modbus |
| 30003 | 运动指令 |
| 30004 | 实时反馈 |

连接成功日志应同时出现 29999、30003、30004。随后只读验证：

```bash
ros2 service call /dobot_bringup_v3/srv/RobotMode \
  dobot_msgs_v3/srv/RobotMode "{}"

ros2 service call /dobot_bringup_v3/srv/GetAngle \
  dobot_msgs_v3/srv/GetAngle "{}"

ros2 service call /dobot_bringup_v3/srv/GetPose \
  dobot_msgs_v3/srv/GetPose "{user: 0, tool: 0}"
```

`requester: making request` 只表示 ROS 客户端已经发出请求；只有出现 `response:`
才代表服务端给出响应。

## 2. RobotMode

| 值 | 本项目显示 | 操作含义 |
|---:|---|---|
| 4 | 未使能 | 可以在核对负载和安全后使能 |
| 5 | 已使能且空闲 | 点位运动和进入拖动的起始状态 |
| 6 | 拖动模式 | 可以手动引导和取点 |
| 7 | 运行中 | 运动正在执行 |
| 9 | 报警 | 查明并排除报警原因 |
| 10 | 暂停 | 只有核对旧队列后才能 `Continue` |
| 11 | 碰撞 | 排除接触/碰撞后处理 |

报警查询：

```bash
ros2 service call /dobot_bringup_v3/srv/GetErrorID \
  dobot_msgs_v3/srv/GetErrorID "{}"
```

清警：

```bash
ros2 service call /dobot_bringup_v3/srv/ClearError \
  dobot_msgs_v3/srv/ClearError "{}"
```

`ClearError res=0` 表示指令受理，不保证模式立刻退出 9。若原因仍存在，模式会保持
或重新进入报警。

## 3. 使能与末端负载

上位机的“末端总负载”必须包含 VX500/相机、支架、DH-AG95、线缆和工件总重量。
示例命令只演示接口，不代表任何设备的正确负载：

```bash
ros2 service call /dobot_bringup_v3/srv/EnableRobot \
  dobot_msgs_v3/srv/EnableRobot "{load: 1.0}"

ros2 service call /dobot_bringup_v3/srv/SpeedFactor \
  dobot_msgs_v3/srv/SpeedFactor "{ratio: 5}"
```

重心、惯量、Tool 坐标系应在设备支持的配置入口中按真实机械结构设置。仅填写重量
不能替代完整负载辨识。

## 4. 拖动取点

推荐使用 GUI，因为它会按顺序检查并恢复保护：

1. 确认模式 5、无报警、现场有人监护；
2. 默认保留 SafeSkin，只在已确认误触发时勾选临时关闭；
3. 如需降低本体碰撞等级，记录退出后的恢复等级；
4. 点击“进入拖动模式”，等待模式 6；
5. 扶稳机械臂移动，避开关节夹点；
6. 静止后保存点位或同步拍照保存整组；
7. 点击“退出拖动并恢复两项保护”；
8. 在示教器再次确认保护已恢复。

对应服务：

```bash
ros2 service call /dobot_bringup_v3/srv/SetCollisionLevel \
  dobot_msgs_v3/srv/SetCollisionLevel "{level: 0}"
ros2 service call /dobot_bringup_v3/srv/SetSafeSkin \
  dobot_msgs_v3/srv/SetSafeSkin "{status: 0}"
ros2 service call /dobot_bringup_v3/srv/StartDrag \
  dobot_msgs_v3/srv/StartDrag "{}"
```

退出后必须恢复：

```bash
ros2 service call /dobot_bringup_v3/srv/StopDrag \
  dobot_msgs_v3/srv/StopDrag "{}"
ros2 service call /dobot_bringup_v3/srv/SetSafeSkin \
  dobot_msgs_v3/srv/SetSafeSkin "{status: 1}"
ros2 service call /dobot_bringup_v3/srv/SetCollisionLevel \
  dobot_msgs_v3/srv/SetCollisionLevel "{level: 3}"
```

上面的等级 3 只是示例，必须使用现场原设置。上位机在进入失败、退出和关闭窗口时
都会尝试回滚，但通信中断时仍需在示教器确认。

## 5. 点位格式

旧格式保持兼容：

```text
points/P1_joint.txt
points/P1_pose.txt
```

同步图像组格式：

```text
points/P1/
├── P1_joint.txt
├── P1_pose.txt
├── P1_image.jpg
└── P1_capture.json
```

点名只允许字母、数字、点、下划线和连字符。写入采用临时文件原子替换。拍照组会
在曝光前后读取关节角，默认最大漂移超过 `0.05°` 时拒绝保存。

## 6. 安全点位运动 CLI

先预览：

```bash
ros2 run dobot_point_manager move_between_points \
  --from P2 --to P1
```

确认起点、目标点、完整路径和急停后才执行：

```bash
ros2 run dobot_point_manager move_between_points \
  --from P2 --to P1 \
  --speed-factor 5 --speed-j 10 --acc-j 10 \
  --tolerance-deg 0.5 --execute
```

程序要求：模式必须为 5、当前位置必须在起点容差内、所有服务返回 `res=0`。它在
`JointMovJ` 后轮询 `GetAngle` 和 `RobotMode`，连续到位并恢复模式 5 才算完成。

## 7. 为什么不调用 Sync

官方 V3 Python bringup 使用单线程执行器，`Sync()` 回调又在 30003 socket 上阻塞
等待队列完成。一旦运动队列暂停、报警或未运行，`Sync` 可永久占住回调，导致同一
节点的其他服务全部无响应。客户端的 120 秒超时不会取消服务端的 `recv()`。

本项目的 CLI 和 GUI 都不调用 `Sync`，而是主动轮询状态；若命令已接受但 10 秒内
位置没有变化，会停止继续发送指令，并提示检查报警和队列状态。

## 8. 本项目对官方 V3 驱动的修复

- Dashboard/运动连接只在全部成功后原子切换，断线后定时重连；
- 服务调用在未连接时明确返回 `res=-1`，不再因属性不存在崩溃；
- 网络异常后不自动重发运动命令，避免重复运动；
- 30004 反馈按完整 1440 字节帧重组，断线后可恢复；
- 修复 `GetErrorID` 未填充 `error_id` 的问题；
- 为 Modbus 寄存器、数量和值类型增加校验；
- 新增 `SetSafeSkin` Dashboard 和 ROS 2 服务；
- 增加连接恢复、Modbus 和 SafeSkin 自动测试。

详细代码位置见[代码目录与修改记录](08-code-inventory-development.md)。

