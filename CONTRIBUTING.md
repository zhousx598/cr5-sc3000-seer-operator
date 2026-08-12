# 贡献指南

本项目会连接并驱动真实机械臂、夹爪和 AGV。代码审查的首要标准是可预测、可停止、
失败后不产生额外运动，其次才是便利性。

首次参与开发请先阅读根目录的[工程接手引导](%E5%BC%95%E5%AF%BC.md)。

## 开发环境

```bash
export DOBOT_WS="${HOME}/dobot_ws"
cd "${DOBOT_WS}"
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  seer_agv_driver dobot_operator_gui dobot_point_manager
source install/setup.bash
```

使用 Ubuntu 22.04、ROS 2 Humble 和系统 `/usr/bin/python3`。不要把 Conda、Snap Qt
插件、`build/`、`install/`、`log/`、点位、现场图像、地图、口令或真实标定外参提交
到仓库。

## 修改原则

- GUI 只通过 ROS 2 访问 CR5 和 AGV；不要新增第二个 SEER TCP 所有者；
- 运动、点动、定位确认、清警和关闭安全保护必须由操作员明确触发；
- 不自动重发结果不确定的运动命令；
- 不用客户端超时掩盖阻塞式 `Sync()`；
- 退出、异常和按钮释放必须发送停止或恢复临时更改的安全配置；
- 网络断开返回明确失败，不把旧状态显示为在线；
- 新配置优先使用 ROS 参数、YAML 或环境变量，不写死用户名和绝对主目录；
- 标定结果必须携带单位、坐标系方向、算法和数据集信息。

## 测试

提交前运行 [测试文档](docs/09-testing-troubleshooting.md) 第 1 节的构建和功能回归。
协议测试使用假 socket/响应，不连接真机；真机测试必须人工分阶段执行，且不得放入
默认 CI。

新增功能至少覆盖：

1. 正常返回；
2. 超时、断线或畸形响应；
3. 安全门拒绝；
4. 关闭或取消后的状态恢复；
5. 不向其他设备命名空间发布同名控制指令。

## 提交说明

变更说明应包含：问题、根因、修改文件、测试结果、是否连接真机、真机是否发生运动、
回滚方法和剩余风险。硬件协议来源应写明手册名称/版本和 API 或寄存器号，不能只写
“已测试可用”。

公开发布前还必须完成[许可证与脱敏检查](docs/11-github-release-reuse.md)。
