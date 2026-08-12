#!/usr/bin/env python3
"""Calibrate the eye-in-hand transform from CR5 captures 1..20.

Coordinate transform notation uses ``destination_T_source``.  OpenCV's
``calibrateHandEye`` returns gripper_T_camera for an eye-in-hand setup.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path

import cv2
import numpy as np


METHODS = {
    'Tsai': cv2.CALIB_HAND_EYE_TSAI,
    'Park': cv2.CALIB_HAND_EYE_PARK,
    'Horaud': cv2.CALIB_HAND_EYE_HORAUD,
    'Andreff': cv2.CALIB_HAND_EYE_ANDREFF,
    'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--points-dir', type=Path, default=Path('~/dobot_ws/points').expanduser()
    )
    parser.add_argument(
        '--intrinsics',
        type=Path,
        default=Path(
            '~/dobot_ws/points/calibration_results/'
            'sc3000_intrinsics_recommended.yaml'
        ).expanduser(),
    )
    parser.add_argument('--first', type=int, default=1)
    parser.add_argument('--last', type=int, default=20)
    parser.add_argument('--columns', type=int, default=7)
    parser.add_argument('--rows', type=int, default=5)
    parser.add_argument('--square-size-mm', type=float, default=11.2)
    parser.add_argument('--method', choices=METHODS, default='Park')
    parser.add_argument(
        '--output-prefix',
        type=Path,
        default=Path(
            '~/dobot_ws/points/calibration_results/'
            'sc3000_cr5_handeye_first20'
        ).expanduser(),
    )
    return parser.parse_args()


def load_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray, int, int]:
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise RuntimeError(f'无法打开相机内参：{path}')
    camera_matrix = storage.getNode('camera_matrix').mat()
    distortion = storage.getNode('distortion_coefficients').mat()
    width = int(storage.getNode('image_width').real())
    height = int(storage.getNode('image_height').real())
    storage.release()
    if camera_matrix is None or distortion is None:
        raise RuntimeError(f'相机内参文件缺少矩阵：{path}')
    return camera_matrix, distortion, width, height


def rotation_xyz_deg(rx: float, ry: float, rz: float) -> np.ndarray:
    """Dobot pose convention: fixed-axis XYZ, R = Rz @ Ry @ Rx."""
    x, y, z = np.deg2rad([rx, ry, rz])
    cx, sx = math.cos(x), math.sin(x)
    cy, sy = math.cos(y), math.sin(y)
    cz, sz = math.cos(z), math.sin(z)
    rot_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=float)
    rot_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=float)
    rot_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=float)
    return rot_z @ rot_y @ rot_x


def transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    matrix[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return matrix


def inverse_transform(matrix: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=float)
    rotation = matrix[:3, :3]
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ matrix[:3, 3]
    return result


def mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    u, _, vt = np.linalg.svd(np.sum(rotations, axis=0))
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(u @ vt)
    return u @ correction @ vt


def rotation_error_deg(reference: np.ndarray, sample: np.ndarray) -> float:
    cosine = (np.trace(reference.T @ sample) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


def matrix_to_euler_xyz_deg(rotation: np.ndarray) -> list[float]:
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return np.rad2deg([roll, pitch, yaw]).tolist()


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> list[float]:
    # Eigenvector solution is stable for all rotation angles.
    r = rotation
    k = np.array(
        [
            [r[0, 0] - r[1, 1] - r[2, 2], r[0, 1] + r[1, 0], r[0, 2] + r[2, 0], r[2, 1] - r[1, 2]],
            [r[0, 1] + r[1, 0], r[1, 1] - r[0, 0] - r[2, 2], r[1, 2] + r[2, 1], r[0, 2] - r[2, 0]],
            [r[0, 2] + r[2, 0], r[1, 2] + r[2, 1], r[2, 2] - r[0, 0] - r[1, 1], r[1, 0] - r[0, 1]],
            [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1], r[0, 0] + r[1, 1] + r[2, 2]],
        ],
        dtype=float,
    ) / 3.0
    values, vectors = np.linalg.eigh(k)
    quaternion = vectors[:, np.argmax(values)]
    if quaternion[3] < 0:
        quaternion = -quaternion
    return quaternion.tolist()


def to_list(value: np.ndarray) -> list:
    return np.asarray(value, dtype=float).tolist()


def evaluate(
    base_t_grippers: list[np.ndarray],
    camera_t_targets: list[np.ndarray],
    gripper_t_camera: np.ndarray,
) -> tuple[dict, list[dict], np.ndarray]:
    base_t_targets = [
        base_t_gripper @ gripper_t_camera @ camera_t_target
        for base_t_gripper, camera_t_target in zip(
            base_t_grippers, camera_t_targets
        )
    ]
    translations = np.array([item[:3, 3] for item in base_t_targets])
    mean_translation = translations.mean(axis=0)
    mean_target_rotation = mean_rotation([item[:3, :3] for item in base_t_targets])
    translation_errors = np.linalg.norm(translations - mean_translation, axis=1)
    rotation_errors = np.array(
        [
            rotation_error_deg(mean_target_rotation, item[:3, :3])
            for item in base_t_targets
        ]
    )
    metrics = {
        'translation_rms_mm': float(np.sqrt(np.mean(translation_errors**2))),
        'translation_mean_mm': float(np.mean(translation_errors)),
        'translation_max_mm': float(np.max(translation_errors)),
        'rotation_rms_deg': float(np.sqrt(np.mean(rotation_errors**2))),
        'rotation_mean_deg': float(np.mean(rotation_errors)),
        'rotation_max_deg': float(np.max(rotation_errors)),
    }
    per_frame = [
        {
            'translation_error_mm': float(translation_errors[index]),
            'rotation_error_deg': float(rotation_errors[index]),
            'base_T_target_translation_mm': to_list(item[:3, 3]),
        }
        for index, item in enumerate(base_t_targets)
    ]
    mean_target = transform(mean_target_rotation, mean_translation)
    return metrics, per_frame, mean_target


def read_capture_set(args: argparse.Namespace) -> tuple[list[dict], list[np.ndarray], list[np.ndarray]]:
    camera_matrix, distortion, width, height = load_intrinsics(args.intrinsics)
    object_points = np.zeros((args.columns * args.rows, 3), dtype=np.float64)
    object_points[:, :2] = (
        np.mgrid[0 : args.columns, 0 : args.rows].T.reshape(-1, 2)
        * args.square_size_mm
    )
    frames: list[dict] = []
    base_t_grippers: list[np.ndarray] = []
    camera_t_targets: list[np.ndarray] = []
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY

    for point_id in range(args.first, args.last + 1):
        directory = args.points_dir / str(point_id)
        metadata_path = directory / f'{point_id}_capture.json'
        if not metadata_path.is_file():
            raise RuntimeError(f'缺少采集元数据：{metadata_path}')
        metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
        image_path = directory / metadata.get('image_file', f'{point_id}_image.jpg')
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f'无法读取图像：{image_path}')
        if image.shape[1] != width or image.shape[0] != height:
            raise RuntimeError(
                f'{image_path}分辨率为{image.shape[1]}x{image.shape[0]}，'
                f'内参要求{width}x{height}'
            )
        found, corners = cv2.findChessboardCornersSB(
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
            (args.columns, args.rows),
            flags=flags,
        )
        if not found:
            raise RuntimeError(f'未识别到{args.columns}x{args.rows}内角点：{image_path}')
        solved, rotation_vector, translation = cv2.solvePnP(
            object_points,
            corners,
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not solved or float(translation[2, 0]) <= 0:
            raise RuntimeError(f'solvePnP失败或深度无效：{image_path}')
        target_rotation, _ = cv2.Rodrigues(rotation_vector)
        projected, _ = cv2.projectPoints(
            object_points,
            rotation_vector,
            translation,
            camera_matrix,
            distortion,
        )
        reprojection = projected.reshape(-1, 2) - corners.reshape(-1, 2)
        pnp_rms = float(np.sqrt(np.mean(np.sum(reprojection**2, axis=1))))

        pose = np.asarray(metadata['tool_pose'], dtype=float)
        if pose.shape != (6,):
            raise RuntimeError(f'tool_pose不是6维：{metadata_path}')
        base_t_gripper = transform(rotation_xyz_deg(*pose[3:]), pose[:3])
        camera_t_target = transform(target_rotation, translation)
        base_t_grippers.append(base_t_gripper)
        camera_t_targets.append(camera_t_target)
        frames.append(
            {
                'point_id': point_id,
                'image': str(image_path),
                'metadata': str(metadata_path),
                'tool_pose_mm_deg': pose.tolist(),
                'pnp_reprojection_rms_px': pnp_rms,
                'camera_T_target_translation_mm': translation.reshape(3).tolist(),
            }
        )
    return frames, base_t_grippers, camera_t_targets


def calibrate(args: argparse.Namespace) -> dict:
    frames, base_t_grippers, camera_t_targets = read_capture_set(args)
    method_results = {}
    selected_transform = None
    selected_mean_target = None
    selected_per_frame = None
    for name, method in METHODS.items():
        rotation, translation = cv2.calibrateHandEye(
            [item[:3, :3] for item in base_t_grippers],
            [item[:3, 3] for item in base_t_grippers],
            [item[:3, :3] for item in camera_t_targets],
            [item[:3, 3] for item in camera_t_targets],
            method=method,
        )
        gripper_t_camera = transform(rotation, translation)
        metrics, per_frame, mean_target = evaluate(
            base_t_grippers, camera_t_targets, gripper_t_camera
        )
        method_results[name] = {
            'gripper_T_camera': to_list(gripper_t_camera),
            'validation': metrics,
        }
        if name == args.method:
            selected_transform = gripper_t_camera
            selected_mean_target = mean_target
            selected_per_frame = per_frame

    assert selected_transform is not None
    camera_t_gripper = inverse_transform(selected_transform)
    for frame, errors in zip(frames, selected_per_frame):
        frame.update(errors)
    result = {
        'format_version': 1,
        'generated_utc': datetime.now(timezone.utc).isoformat(),
        'setup': 'eye-in-hand (camera rigidly fixed to CR5 end effector)',
        'transform_notation': 'destination_T_source',
        'input': {
            'points_dir': str(args.points_dir),
            'point_ids_used': list(range(args.first, args.last + 1)),
            'point_ids_explicitly_excluded': [21, 22],
            'intrinsics': str(args.intrinsics),
            'board_inner_corners': [args.columns, args.rows],
            'square_size_mm': args.square_size_mm,
            'robot_tool_pose_convention': (
                '[X,Y,Z,Rx,Ry,Rz] in mm/deg; fixed-axis XYZ; '
                'R = Rz(Rz) @ Ry(Ry) @ Rx(Rx)'
            ),
        },
        'selected_method': args.method,
        'gripper_T_camera': {
            'matrix': to_list(selected_transform),
            'translation_mm': to_list(selected_transform[:3, 3]),
            'euler_xyz_deg': matrix_to_euler_xyz_deg(selected_transform[:3, :3]),
            'quaternion_xyzw': matrix_to_quaternion_xyzw(selected_transform[:3, :3]),
        },
        'camera_T_gripper': {
            'matrix': to_list(camera_t_gripper),
            'translation_mm': to_list(camera_t_gripper[:3, 3]),
            'euler_xyz_deg': matrix_to_euler_xyz_deg(camera_t_gripper[:3, :3]),
            'quaternion_xyzw': matrix_to_quaternion_xyzw(camera_t_gripper[:3, :3]),
        },
        'base_T_target_mean': {
            'matrix': to_list(selected_mean_target),
            'translation_mm': to_list(selected_mean_target[:3, 3]),
            'euler_xyz_deg': matrix_to_euler_xyz_deg(selected_mean_target[:3, :3]),
        },
        'selected_validation': method_results[args.method]['validation'],
        'method_comparison': method_results,
        'frames': frames,
    }
    return result


def write_outputs(prefix: Path, result: dict) -> tuple[Path, Path, Path]:
    prefix.parent.mkdir(parents=True, exist_ok=True)
    json_path = prefix.with_suffix('.json')
    yaml_path = prefix.with_suffix('.yaml')
    report_path = prefix.with_suffix('.md')
    json_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8'
    )

    storage = cv2.FileStorage(str(yaml_path), cv2.FILE_STORAGE_WRITE)
    storage.write('transform_notation', 'destination_T_source')
    storage.write('selected_method', result['selected_method'])
    storage.write('gripper_T_camera', np.array(result['gripper_T_camera']['matrix']))
    storage.write('camera_T_gripper', np.array(result['camera_T_gripper']['matrix']))
    storage.write('base_T_target_mean', np.array(result['base_T_target_mean']['matrix']))
    storage.write(
        'translation_rms_mm',
        result['selected_validation']['translation_rms_mm'],
    )
    storage.write(
        'rotation_rms_deg', result['selected_validation']['rotation_rms_deg']
    )
    storage.release()

    gtc = result['gripper_T_camera']
    validation = result['selected_validation']
    method_rows = '\n'.join(
        '| {} | {:.3f} | {:.3f} | {:.3f} | {:.3f} |'.format(
            name,
            item['validation']['translation_rms_mm'],
            item['validation']['translation_max_mm'],
            item['validation']['rotation_rms_deg'],
            item['validation']['rotation_max_deg'],
        )
        for name, item in result['method_comparison'].items()
    )
    frame_rows = '\n'.join(
        '| {} | {:.3f} | {:.3f} | {:.3f} |'.format(
            frame['point_id'],
            frame['pnp_reprojection_rms_px'],
            frame['translation_error_mm'],
            frame['rotation_error_deg'],
        )
        for frame in result['frames']
    )
    matrix_rows = '\n'.join(
        '    ' + ' '.join(f'{number: .10f}' for number in row)
        for row in gtc['matrix']
    )
    report = f"""# SC3000–CR5 手眼标定（点位1–20）

## 结论

- 布置：眼在手上，相机固定于CR5末端；
- 使用数据：点位1–20，共20组；
- 明确排除：点位21、22；
- 棋盘格：7×5内角点，方格边长11.2 mm；
- 选择算法：OpenCV `{result['selected_method']}`；
- 变换记法：`目标坐标系_T_源坐标系`。

最终的 `gripper_T_camera`（相机坐标转换到末端坐标）为：

```text
{matrix_rows}
```

- 平移 `[x,y,z]` mm：`{[round(x, 6) for x in gtc['translation_mm']]}`
- 固定轴XYZ角 `[Rx,Ry,Rz]` deg：`{[round(x, 6) for x in gtc['euler_xyz_deg']]}`
- 四元数 `[x,y,z,w]`：`{[round(x, 9) for x in gtc['quaternion_xyzw']]}`

## 一致性验证

把每一组棋盘格都变换到CR5基座坐标后，其理论位置应保持不变：

- 平移RMS：{validation['translation_rms_mm']:.3f} mm；
- 平移最大误差：{validation['translation_max_mm']:.3f} mm；
- 旋转RMS：{validation['rotation_rms_deg']:.3f}°；
- 旋转最大误差：{validation['rotation_max_deg']:.3f}°。

### 算法对比

| 算法 | 平移RMS/mm | 平移最大/mm | 旋转RMS/° | 旋转最大/° |
|---|---:|---:|---:|---:|
{method_rows}

### 逐组误差

| 点位 | solvePnP重投影RMS/px | 基座棋盘格平移误差/mm | 旋转误差/° |
|---:|---:|---:|---:|
{frame_rows}

## 坐标转换

实时AprilTag定位得到 `camera_T_tag` 后：

```text
base_T_tag = base_T_gripper × gripper_T_camera × camera_T_tag
```

其中 `base_T_gripper` 必须使用与采集时相同的CR5 Tool/User配置。正式控制机械臂前，
还需要用若干已知位置对该矩阵做独立验证，并加入“标签到实际抓取点”的固定偏移。

## 复算命令

```bash
/usr/bin/python3 \\
  $DOBOT_WS/src/dobot_operator_gui/tools/calibrate_handeye_first20.py
```
"""
    report_path.write_text(report, encoding='utf-8')
    return json_path, yaml_path, report_path


def main() -> int:
    args = parse_args()
    if args.first > args.last:
        raise SystemExit('--first不能大于--last')
    if args.square_size_mm <= 0:
        raise SystemExit('--square-size-mm必须大于0')
    result = calibrate(args)
    json_path, yaml_path, report_path = write_outputs(args.output_prefix, result)
    validation = result['selected_validation']
    print(f"已完成：{len(result['frames'])}组，算法={result['selected_method']}")
    print('gripper_T_camera translation (mm):', np.round(result['gripper_T_camera']['translation_mm'], 6))
    print('gripper_T_camera Euler XYZ (deg):', np.round(result['gripper_T_camera']['euler_xyz_deg'], 6))
    print(
        '验证：平移RMS={:.3f} mm，最大={:.3f} mm；旋转RMS={:.3f}°，最大={:.3f}°'.format(
            validation['translation_rms_mm'],
            validation['translation_max_mm'],
            validation['rotation_rms_deg'],
            validation['rotation_max_deg'],
        )
    )
    print('JSON:', json_path)
    print('YAML:', yaml_path)
    print('报告:', report_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
