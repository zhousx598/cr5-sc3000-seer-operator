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

- 移动到已保存点位；
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

