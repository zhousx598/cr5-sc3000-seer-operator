# SC3000、AprilTag 与标定

## 1. Linux 图像链路

本项目不依赖 Windows SCMVS SDK 取图，使用设备公开协议：

```text
GUI/脚本写 Modbus REG0 -> SC3000 执行任务 -> SC3000 通过 FTP 上传 JPG -> GUI读取
```

当前默认：

- SC3000：`192.168.192.11`；
- Modbus TCP：502，Unit ID 1；
- FTP Server：PC 的 2121；
- 被动端口：30000～30009；
- 示例 FTP 用户：`sc3000`；密码应在部署时更换；
- 控制寄存器 offset 0，状态寄存器 offset 1。

触发状态机：

1. 写 `REG0=1`；
2. 轮询 `REG1.0 Trigger Ready`；
3. 写 `REG0=3` 触发；
4. 等待 `REG1.8 Results Available` 或 `REG1.9 Timeout`；
5. 写 `REG0=5` Ack；
6. 写回 `REG0=1`。

相机必须已进入循环运行态，并将 FTP 目标 IP 指向 Ubuntu 有线地址。
FTP 凭据从 `SC3000_FTP_USER` 和 `SC3000_FTP_PASSWORD` 读取；生产环境应覆盖示例
默认值，且不要把真实口令写入源码或采集元数据。

## 2. 实时预览与同步采集

GUI 目标周期为 200 ms，即目标 5 FPS。SC3000 的曝光、任务执行和 JPEG FTP 上传
决定实际帧率，所以这是轮询式预览，不是硬实时视频流。

预览帧存放在内存文件系统：

```text
/dev/shm/dobot_operator_gui_sc3000
```

每帧显示后删除；停止预览和退出 GUI 时再次清理。同步采集则把原始图、关节角、
Tool0 位姿和元数据保存到同一个点位文件夹。

上位机需要独占 FTP 2121。若已有独立 FTP Server，GUI 会拒绝启动第二个监听器。

## 3. 相机内参标定

已验证标定板参数：外观 8×6 方格、内角点 7×5、方格边长 11.2 mm。推荐采集
20～30 张清晰图，覆盖画面中心、四角、边缘、不同距离和倾角，同时保持焦距、
对焦、分辨率和裁剪不变。

标定程序：

```bash
export DOBOT_WS="${HOME}/dobot_ws"
/usr/bin/python3 \
  "${DOBOT_WS}/src/dobot_operator_gui/tools/calibrate_sc3000.py" \
  --input-dir "${DOBOT_WS}/points" \
  --output-dir "${DOBOT_WS}/points/calibration_results" \
  --corners-x 7 --corners-y 5 --square-size-mm 11.2 \
  --distortion-model radial-k1 \
  --output-prefix sc3000_intrinsics_recommended
```

算法流程：

1. `findChessboardCornersSB` 检测角点，失败时回退传统检测；
2. `cornerSubPix` 亚像素优化；
3. 按 11.2 mm 构造平面 3D 点；
4. `calibrateCameraExtended` 联合优化内参、畸变和逐图外参；
5. 计算逐图重投影 RMS；
6. 比较多种畸变模型并做留一法交叉验证。

模型比较：

```bash
/usr/bin/python3 \
  "${DOBOT_WS}/src/dobot_operator_gui/tools/compare_sc3000_models.py" \
  --input-dir "${DOBOT_WS}/points" \
  --output-dir "${DOBOT_WS}/points/calibration_results"
```

当前 22 张样例图的推荐结果为 1408×1024、`radial-k1`，总体 RMS 约 0.608 px。
这些数值只属于当前相机、镜头、分辨率和对焦状态，其他设备必须重新标定。

主要输出：

- `sc3000_intrinsics_recommended.yaml`：OpenCV GUI/脚本读取；
- `sc3000_camera_info.yaml`：ROS CameraInfo 兼容格式；
- `*.json`：完整外参、误差和诊断；
- `*_corners.jpg`：角点与逐图误差拼图。

## 4. AprilTag 2.5D 定位

已验证标签为 `tag36h11`、ID 0，黑色正方形外边长 58.5 mm。尺寸必须测量黑框，
不含白色外边距。尺寸写小 10 倍会使求得的 X/Y/Z 也近似缩小 10 倍。

单帧调用：

```bash
export DOBOT_WS="${HOME}/dobot_ws"
"${DOBOT_WS}/estimate_apriltag_pose.sh" --family tag36h11 --tag-id 0
```

已有图片：

```bash
/usr/bin/python3 \
  "${DOBOT_WS}/src/dobot_operator_gui/tools/estimate_apriltag_pose.py" \
  --image /path/to/image.jpg \
  --tag-size-mm 58.5 --family tag36h11 --tag-id 0 \
  --calibration "${DOBOT_WS}/points/calibration_results/sc3000_intrinsics_recommended.yaml"
```

求解使用：

- OpenCV ArUco 的 AprilTag 字典和角点优化；
- `solvePnPGeneric(..., SOLVEPNP_IPPE_SQUARE)` 获取平面标记候选解；
- 丢弃相机后方解，选择重投影误差最小解；
- `solvePnPRefineLM` 精修；
- 输出重投影 RMS 和相机坐标位姿。

OpenCV 相机坐标定义：X 向图像右，Y 向图像下，Z 沿镜头光轴向前。

## 5. 眼在手上手眼标定

相机固定在 CR5 末端，棋盘固定不动。每一组必须包含同一静止时刻的：

- `GetAngle` 六轴关节角；
- `GetPose(User=0, Tool=0)` Tool0 位姿；
- 清晰棋盘图像。

采集时不仅要改变位置，还要给末端足够多的绕 X/Y/Z 旋转变化。只做平移会使手眼
旋转不可观。

复算命令：

```bash
/usr/bin/python3 \
  "${DOBOT_WS}/src/dobot_operator_gui/tools/calibrate_handeye_first20.py" \
  --points-dir "${DOBOT_WS}/points" \
  --intrinsics "${DOBOT_WS}/points/calibration_results/sc3000_intrinsics_recommended.yaml" \
  --first 1 --last 20 --columns 7 --rows 5 --square-size-mm 11.2 \
  --output-prefix "${DOBOT_WS}/points/calibration_results/sc3000_cr5_handeye_first20"
```

程序比较 Tsai、Park、Horaud、Andreff、Daniilidis。当前样例选择 Park，并明确排除
第 21、22 组。样例一致性为平移 RMS 2.246 mm、旋转 RMS 0.581°，但这个矩阵只
适用于当前相机支架安装。相机或支架移动后必须重做。

## 6. 坐标转换

记号采用 `destination_T_source`：

```text
base_T_tag = base_T_tool0 × tool0_T_camera × camera_T_tag
```

- `camera_T_tag`：AprilTag `solvePnP`；
- `tool0_T_camera`：眼在手上标定；
- `base_T_tool0`：实时 `GetPose(User=0, Tool=0)`；
- `base_T_tag`：GUI 实时显示的 CR5 User0/基座坐标。

实际抓取还缺少一项标定：

```text
base_T_grasp = base_T_tag × tag_T_grasp
```

`tag_T_grasp` 是标签与真实抓取点之间的固定偏移，必须由工装几何或示教得到。
