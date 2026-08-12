# DH-AG95 夹爪与任务队列

## 1. 物理与通信配置

DH-AG95 使用 CR5 末端 RS485 A/B：

- Modbus RTU；
- 从站 ID 1；
- 115200 bit/s，8N1；
- 控制器内部 Modbus 地址 `127.0.0.1:60000`；
- `ModbusCreate(..., is_rtu=1)`。

VX500 和 DH-AG95 不能同时并接占用同一组末端 RS485 A/B。上电前核对电压、极性、
A/B 和设备说明书。

## 2. GUI 操作顺序

1. 机械臂驱动在线；
2. 打开“DH-AG95夹爪”页；
3. 创建通道；
4. 初始化并等待 `0x0200=1`；
5. 先设置较低夹持力，例如 20%；
6. 完全打开（协议位置 1000）；
7. 在无障碍条件下低力闭合或移动到中间位置；
8. 读取状态；
9. 不再使用时关闭通道。

## 3. 寄存器

| 地址 | 方向 | 含义 | 本项目范围 |
|---:|---|---|---:|
| `0x0100` | 写 | 初始化 | 1 |
| `0x0101` | 写 | 夹持力 | 20～100% |
| `0x0103` | 写 | 目标位置 | 0 闭合，1000 打开 |
| `0x0200` | 读 | 初始化状态 | 1 为完成 |
| `0x0201` | 读 | 夹持状态 | 依协议解释 |
| `0x0202` | 读 | 实际位置 | 0～1000 |

本项目把夹持状态 3 视为“物体脱落”并报错；状态 1 且到位、或状态 2 且已夹到
物体，才允许队列继续。

闭合比例是便于操作员理解的直线行程，不是角度：

```text
协议位置 = 1000 × (100 - 闭合百分比) / 100
```

因此 0% 表示打开，100% 表示闭合。

## 4. ROS 2 原始调用示例

创建通道：

```bash
ros2 service call /dobot_bringup_v3/srv/ModbusCreate \
  dobot_msgs_v3/srv/ModbusCreate \
  "{ip: '127.0.0.1', port: 60000, slave_id: 1, is_rtu: 1}"
```

记录返回的 `index`，以下假设为 0。初始化：

```bash
ros2 service call /dobot_bringup_v3/srv/SetHoldRegs \
  dobot_msgs_v3/srv/SetHoldRegs \
  "{index: 0, addr: 256, count: 1, val_tab: '{1}', val_type: 'U16'}"
```

设置力 20% 和完全打开：

```bash
ros2 service call /dobot_bringup_v3/srv/SetHoldRegs \
  dobot_msgs_v3/srv/SetHoldRegs \
  "{index: 0, addr: 257, count: 1, val_tab: '{20}', val_type: 'U16'}"

ros2 service call /dobot_bringup_v3/srv/SetHoldRegs \
  dobot_msgs_v3/srv/SetHoldRegs \
  "{index: 0, addr: 259, count: 1, val_tab: '{1000}', val_type: 'U16'}"
```

读取三项反馈：

```bash
ros2 service call /dobot_bringup_v3/srv/GetHoldRegs \
  dobot_msgs_v3/srv/GetHoldRegs \
  "{index: 0, addr: 512, count: 3, val_type: 'U16'}"
```

## 5. 任务队列

支持：

- 导航到控制器官方站点及其保存朝向；
- 导航到本地用户航点的完整 `X/Y/Yaw`，由 Robokit 实时规划路径；
- 移动到已保存点位；
- 测量固定 AprilTag 的多帧 6D 到站纠偏；
- 以同一次纠偏量移动到一个或多个已示教 Tool0 点位；
- 夹爪闭合比例；
- 夹爪协议位置；
- 设置夹持力；
- 初始化夹爪；
- 等待。

示例 JSON：

```json
{
  "schema_version": 1,
  "commands": [
    {
      "kind": "move_point",
      "params": {
        "point": "P1",
        "speed_factor": 5,
        "speed_j": 10,
        "acc_j": 10,
        "tolerance_deg": 0.5
      }
    },
    {"kind": "gripper_close_percent", "params": {"percent": 35}},
    {"kind": "wait", "params": {"seconds": 1.0}},
    {
      "kind": "move_point",
      "params": {
        "point": "P2",
        "speed_factor": 5,
        "speed_j": 10,
        "acc_j": 10,
        "tolerance_deg": 0.5
      }
    }
  ]
}
```

队列加载时会一次性严格校验所有字段和点位。执行严格串行，当前步骤成功完成后才
进入下一步；失败后后续步骤标记为未执行。“当前步骤后停止”不会切断正在运行的
机械臂动作，紧急情况必须使用红色急停或实体急停。

用户航点队列项在添加时保存完整坐标快照，不只保存名称。之后编辑同名用户航点不会
悄悄改变已经保存的队列；需要采用新坐标时应删除旧队列项并重新添加。每次 AGV
导航都会作废之前的 AprilTag 纠偏结果，新的机械臂纠偏动作前必须重新测量。

## 6. AprilTag 到站纠偏队列

### 6.1 设计

纠偏由两种独立指令组成：

- `measure_apriltag_correction`：连续拍摄指定标签，建立本次队列的纠偏变换；
- `move_point_corrected`：把已示教点位的 User0/Tool0 位姿转换成当前目标后运动。

一条测量指令可以供后续多个纠偏点位共用，所以相机只需在观察点看见标签。接近、
按压和撤回过程中标签可以离开视野。队列每次启动都会清空上一次纠偏，不能沿用历史
结果。任何后续 AGV 导航都会立即作废之前的视觉测量，必须在新的到站位置重新执行
“测量AprilTag纠偏”。

设理想到站参考标签为 `B_ref_T_tag`，实际到站标签为 `B_now_T_tag`，示教目标为
`B_ref_T_target`：

```text
correction = B_now_T_tag × inverse(B_ref_T_tag)
B_now_T_target = correction × B_ref_T_target
```

这里全部是 4×4 刚体变换，能同时补偿 XYZ 和旋转；程序不会把 XYZ 差值直接加到
六个关节角上。

### 6.2 示教流程

1. 让 AGV 停在认为正确的理想到站位置；
2. 将按钮旁的 AprilTag 与按钮刚性固定；
3. 让机械臂处于能完整看到标签、且 AGV 有偏差时仍不会碰撞的宽视野观察点；
4. 在取点页启用精确标签族和 ID，执行“同步拍照并保存整组”，例如保存
   `BUTTON1_REF`；
5. 在同一理想到站条件下示教并保存：`BUTTON1_VIEW`、`BUTTON1_APPROACH`、
   `BUTTON1_PRESS`、`BUTTON1_RETRACT`；
6. `BUTTON1_REF` 必须有图片、Tool0 位姿和 AprilTag 检测；其他目标至少要有
   Tool0 位姿。

参考采集和执行必须使用同一个标签黑框边长、相机内参、手眼外参及 User0/Tool0
定义。相机支架、按钮、标签、Tool 或 User 发生变化后必须重新示教。

### 6.3 在 GUI 中建立队列

建议顺序：

1. 先把机械臂收回经现场验证的安全运输姿态；
2. `AGV导航到航点`，选择按钮任务对应的 LM 航点、低速和到站超时；
3. `移动到点位 BUTTON1_VIEW`，使用普通低速关节运动到宽视野观察点；
4. `测量AprilTag纠偏`，目标下拉框选择 `BUTTON1_REF`；
5. `移动到纠偏点位 BUTTON1_APPROACH`，选择关节路径 `MovJ`；
6. `移动到纠偏点位 BUTTON1_PRESS`，选择低速直线路径 `MovL`；
7. `等待`；
8. `移动到纠偏点位 BUTTON1_RETRACT`，选择低速直线路径 `MovL`。

AGV 导航服务返回只代表任务被 Robokit 接受。队列会继续等待更新后的导航状态，只有
状态 4（完成）才进入机械臂步骤；状态 5/6、报警、急停、定位丢失、状态超时或航点
不一致都会终止队列。点击停止队列会取消正在进行的导航。AGV 开始移动前，程序无法
从点位名称证明机械臂已经收拢，安全运输姿态仍必须由现场示教和验收保证。

只把“测量AprilTag纠偏”放入队列可以预览计算结果，不会产生运动。完成测量后，
运行日志会显示 `ΔXYZ`、`ΔRPY`、总平移和总旋转。

默认安全参数：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| 连续采样 | 3 帧 | 排除单帧偶然误差 |
| 重投影上限 | 1.5 px | 拒绝角点/PnP质量差的图像 |
| 纠偏平移上限 | 50 mm | 超出AGV预期停靠范围时拒绝 |
| 纠偏旋转上限 | 5° | 拒绝大角度错误停靠 |
| 多帧平移离散 | 2 mm | 拒绝不稳定测量 |
| 多帧旋转离散 | 1° | 拒绝不稳定姿态 |

这些值是初始低速验证值，不是所有现场通用参数。正式按压前应根据 AGV 到站统计、
按钮尺寸、工具几何和安全间隙进一步收紧。

### 6.4 JSON 示例

```json
{
  "schema_version": 1,
  "commands": [
    {
      "kind": "agv_navigate_station",
      "params": {
        "station_id": "LM_BUTTON1",
        "max_speed_mps": 0.08,
        "timeout_s": 300.0
      }
    },
    {
      "kind": "measure_apriltag_correction",
      "params": {
        "reference_capture": "BUTTON1_REF",
        "family": "tag36h11",
        "tag_id": 0,
        "tag_size_mm": 58.5,
        "camera_host": "192.168.192.11",
        "camera_timeout_s": 12.0,
        "samples": 3,
        "max_reprojection_rms_px": 1.5,
        "max_translation_mm": 50.0,
        "max_rotation_deg": 5.0,
        "max_sample_translation_spread_mm": 2.0,
        "max_sample_rotation_spread_deg": 1.0
      }
    },
    {
      "kind": "move_point_corrected",
      "params": {
        "point": "BUTTON1_APPROACH",
        "motion_type": "joint",
        "speed_factor": 5,
        "speed": 5,
        "acc": 5,
        "position_tolerance_mm": 1.0,
        "orientation_tolerance_deg": 1.0
      }
    },
    {
      "kind": "move_point_corrected",
      "params": {
        "point": "BUTTON1_PRESS",
        "motion_type": "linear",
        "speed_factor": 3,
        "speed": 3,
        "acc": 3,
        "position_tolerance_mm": 0.8,
        "orientation_tolerance_deg": 1.0
      }
    }
  ]
}
```

### 6.5 失败即停止条件

- 参考采集不存在、标签尺寸/族/ID不一致；
- AGV航点不在当前列表、导航未启动、失败、取消、超时或到站不一致；
- 相机内参或手眼外参不可用；
- 机械臂不是模式5，或拍照期间关节漂移超过 `0.05°`；
- 实时图像没有唯一目标标签；
- 重投影误差、多帧离散、总平移或总旋转超过上限；
- 某个具体目标因旋转杠杆效应产生的实际位移超过平移上限；
- 纠偏点位缺少 Tool0 位姿；
- `MovJ/MovL` 未开始、模式异常、报警或到位超时。

任一条件触发后，当前指令失败，后续按压和夹爪动作全部标记为未执行。视觉纠偏不
关闭碰撞检测或电子皮肤，也不能替代机械工作空间、逆解、奇异点和碰撞规划验证。
