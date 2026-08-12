# GitHub 发布、许可证与复用交接

## 1. 当前发布状态

工作空间已经按 GitHub 项目的方式补齐入口、专题文档、源码索引、测试命令和忽略
规则，但**不应在许可证确认前直接公开上传**。当前目录的特殊点是：

- `dobot_ws` 根目录目前不是 Git 仓库；
- `src/DOBOT_6Axis_ROS2_V3/.git` 是设备方源码自带的嵌套仓库；
- 自研包的 `setup.py/package.xml` 已声明 Apache-2.0 或 MIT，但工作空间尚未包含完整
  许可证正文；
- Dobot、SEER、SC3000 和 DH-AG95 的厂家源码、协议和手册可能有各自授权条件。

必须由项目所有者确认著作权和再分发权限，再选择根许可证。不要用一个新许可证覆盖
来源不明或许可证不兼容的厂家内容。

## 2. 推荐仓库边界

公开仓库建议包含：

- 根 `README.md`、`CONTRIBUTING.md`、`.gitignore` 和 `docs/`；
- 四个自研 ROS 包；
- 两个根启动脚本；
- 经授权的 Dobot 驱动快照或带修复的上游 fork/submodule；
- 不含口令的示例 YAML、JSON 和接口定义。

不要公开：

- `build/`、`install/`、`log/` 和缓存；
- `points/`、`queues/`、标定采集、现场照片、地图和站点数据；
- FTP/设备真实口令、个人目录、MAC 地址、生产 IP 拓扑；
- 未获再分发授权的 PDF、Windows SDK、固件和交接包。

## 3. 处理 Dobot 嵌套仓库

两种方式只能选一种：

### 方式 A：维护上游 fork，父仓库使用 submodule

这是保留提交历史和上游关系的首选方式。先把本项目对
`dobot_bringup_v3/dobot_msgs_v3` 的修复提交到有权限的 Dobot fork，再由父仓库锁定
该 commit。接手者使用：

```bash
git clone --recurse-submodules <PROJECT_REPOSITORY_URL>
```

### 方式 B：经授权后 vendoring 源码快照

若要求单一仓库包含全部源码，可在**新的发布暂存目录**中复制工作空间，并在复制时
排除内层 `.git`、运行数据和构建产物。不要直接删除现有内层 `.git`；它包含上游
来源信息和本地修改线索。快照中应附上来源 URL、上游 commit、修改清单和许可证。

## 4. 初始化父仓库

安装 Git、确定上述边界和许可证后，在发布目录操作：

```bash
git init
git status --short
git add README.md CONTRIBUTING.md .gitignore docs src \
  start_dobot_operator_gui.sh estimate_apriltag_pose.sh
git status --short
```

在第一次 commit 前重点确认 `git status` 没有以下内容：图片采集、标定外参、密码、
地图、日志、`build/install`，以及意外的嵌套仓库。随后再配置远端和分支保护。本文不
替使用者创建远端仓库或决定许可证。

另外应把各 `setup.py/package.xml` 中的示例维护者邮箱替换为仓库真实维护者；检查
`camera_capture.py` 中为现有设备兼容保留的 FTP 出厂示例口令，生产或公开部署应
通过 `SC3000_FTP_PASSWORD` 覆盖并在完成设备侧改密后移除该回退值。

## 5. 版本和发布说明

建议使用语义化版本：

- PATCH：错误处理、文档、兼容性修复，不增加动作能力；
- MINOR：新增设备接口、GUI 页面或默认关闭的动作能力；
- MAJOR：ROS 接口、队列格式、坐标系定义或安全语义不兼容变化。

每个 Release 应记录：

- Ubuntu/ROS/Python、CR5 固件和 Robokit 版本；
- 上游 Dobot commit；
- 支持的硬件与已验证网络模式；
- 标定文件格式版本、队列格式版本；
- 自动测试结果和真机验收范围；
- 已知风险、迁移步骤和回滚方法。

## 6. 新设备复用流程

接手者不应直接复制现场标定值。建议顺序是：

1. 按[安装文档](02-install-build-run.md)构建并运行离线测试；
2. 按[网络文档](01-hardware-network-safety.md)逐台连接设备；
3. 只读验证 CR5、相机和 AGV；
4. 设置真实工具负载、重心、Tool/User 坐标和夹爪参数；
5. 重新采集棋盘格并生成相机内参；
6. 重新进行眼在手上标定和独立外参一致性验证；
7. 导入现场 AGV 地图并人工完成重定位确认；
8. 按[验收顺序](09-testing-troubleshooting.md)从空载低速开始；
9. 最后才建立生产点位、夹爪动作和组合队列。

## 7. 建议 CI 边界

CI 可以执行 colcon 构建、Python 编译、假 socket 协议测试、图像算法样例测试和 Qt
离屏测试。CI 不应访问设备 IP，不应使能机械臂，不应确认 AGV 定位，也不应发送
任何真实运动、夹爪或安全配置命令。
