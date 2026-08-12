#!/usr/bin/env python3
"""Estimate an AprilTag center pose in the SC3000 camera frame."""

import argparse
import json
import math
import os
from pathlib import Path
import sys

import cv2
import numpy as np


WORKSPACE = Path(
    os.environ.get('DOBOT_WS', str(Path.home() / 'dobot_ws'))
).expanduser()
DEFAULT_CALIBRATION = (
    WORKSPACE / 'points/calibration_results/sc3000_intrinsics_recommended.yaml'
)
DEFAULT_CAPTURE_DIR = Path('/dev/shm/sc3000_apriltag_pose')

APRILTAG_DICTIONARIES = {
    'tag16h5': cv2.aruco.DICT_APRILTAG_16h5,
    'tag25h9': cv2.aruco.DICT_APRILTAG_25h9,
    'tag36h10': cv2.aruco.DICT_APRILTAG_36h10,
    'tag36h11': cv2.aruco.DICT_APRILTAG_36h11,
}


class AprilTagPoseError(RuntimeError):
    pass


def load_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    calibration_path = path.expanduser().resolve()
    storage = cv2.FileStorage(str(calibration_path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise AprilTagPoseError(f'无法打开相机内参：{calibration_path}')
    try:
        camera_matrix = storage.getNode('camera_matrix').mat()
        distortion = storage.getNode('distortion_coefficients').mat()
        width = int(storage.getNode('image_width').real())
        height = int(storage.getNode('image_height').real())
    finally:
        storage.release()
    if camera_matrix is None or camera_matrix.shape != (3, 3):
        raise AprilTagPoseError('内参文件中的 camera_matrix 无效')
    if distortion is None or distortion.size < 4:
        raise AprilTagPoseError('内参文件中的 distortion_coefficients 无效')
    return (
        camera_matrix.astype(np.float64),
        distortion.astype(np.float64).reshape(-1, 1),
        (width, height),
    )


def capture_sc3000(host: str, timeout: float) -> tuple[np.ndarray, str]:
    try:
        from dobot_operator_gui.camera_capture import Sc3000CameraCapture
    except ImportError as exc:
        raise AprilTagPoseError(
            '无法导入上位机相机模块；请先执行：'
            f'source {WORKSPACE}/install/setup.bash'
        ) from exc

    DEFAULT_CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
    camera = Sc3000CameraCapture()
    result = None
    try:
        result = camera.capture(host, DEFAULT_CAPTURE_DIR, timeout=timeout)
        image = cv2.imread(str(result.image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise AprilTagPoseError(f'无法读取SC3000图像：{result.image_path}')
        return image, f'SC3000 {host} / {result.image_path.name}'
    finally:
        if result is not None:
            try:
                result.image_path.unlink()
            except OSError:
                pass
        camera.close()


def load_image(args) -> tuple[np.ndarray, str]:
    if args.capture:
        return capture_sc3000(args.camera_ip, args.timeout)
    path = args.image.expanduser().resolve()
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise AprilTagPoseError(f'无法读取图像：{path}')
    return image, str(path)


def detect_tags(image: np.ndarray, family: str):
    dictionary = cv2.aruco.getPredefinedDictionary(
        APRILTAG_DICTIONARIES[family]
    )
    if hasattr(cv2.aruco, 'DetectorParameters'):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, rejected = detector.detectMarkers(image)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(
            image, dictionary, parameters=parameters
        )
    if ids is None:
        return [], rejected
    return [
        (int(marker_id), marker_corners.reshape(4, 2).astype(np.float64))
        for marker_id, marker_corners in zip(ids.reshape(-1), corners)
    ], rejected


def reprojection_rms(
    object_points,
    image_points,
    rvec,
    tvec,
    camera_matrix,
    distortion,
) -> float:
    projected, unused = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion
    )
    residual = image_points.reshape(-1, 2) - projected.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))


def rotation_to_rpy_degrees(rotation: np.ndarray) -> tuple[float, float, float]:
    sy = math.hypot(rotation[0, 0], rotation[1, 0])
    singular = sy < 1e-8
    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def estimate_pose(
    corners: np.ndarray,
    tag_size_mm: float,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> dict:
    half = tag_size_mm / 2.0
    # Required order for SOLVEPNP_IPPE_SQUARE: top-left, top-right,
    # bottom-right, bottom-left. Marker origin is at its physical center.
    object_points = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )
    generic_result = cv2.solvePnPGeneric(
        object_points,
        corners,
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    solved, rotation_vectors, translation_vectors = generic_result[:3]
    if not solved or not rotation_vectors:
        raise AprilTagPoseError('solvePnP未能求解AprilTag位姿')

    candidates = []
    for rvec, tvec in zip(rotation_vectors, translation_vectors):
        error = reprojection_rms(
            object_points,
            corners,
            rvec,
            tvec,
            camera_matrix,
            distortion,
        )
        if float(tvec.reshape(-1)[2]) > 0:
            candidates.append((error, rvec, tvec))
    if not candidates:
        raise AprilTagPoseError('solvePnP只返回了相机后方的无效解')
    unused_error, rvec, tvec = min(candidates, key=lambda value: value[0])

    rvec, tvec = cv2.solvePnPRefineLM(
        object_points,
        corners,
        camera_matrix,
        distortion,
        rvec,
        tvec,
    )
    error = reprojection_rms(
        object_points,
        corners,
        rvec,
        tvec,
        camera_matrix,
        distortion,
    )
    rotation, unused = cv2.Rodrigues(rvec)
    xyz = tvec.reshape(3)
    return {
        'xyz_mm': xyz.tolist(),
        'xyz_m': (xyz / 1000.0).tolist(),
        'rvec': rvec.reshape(3).tolist(),
        'rotation_matrix': rotation.tolist(),
        'rpy_degrees': list(rotation_to_rpy_degrees(rotation)),
        'reprojection_rms_px': error,
        'object_points': object_points,
        'rvec_array': rvec,
        'tvec_array': tvec,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Estimate AprilTag XYZ in the SC3000 camera frame.'
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--image', type=Path, help='existing SC3000 image')
    source.add_argument(
        '--capture', action='store_true', help='trigger SC3000 and use one frame'
    )
    parser.add_argument(
        '--tag-size-mm',
        type=float,
        required=True,
        help='physical outer black-square side length, excluding white margin',
    )
    parser.add_argument(
        '--family',
        choices=['auto', *sorted(APRILTAG_DICTIONARIES)],
        default='auto',
        help='AprilTag family; auto tries every OpenCV AprilTag dictionary',
    )
    parser.add_argument('--tag-id', type=int, help='only report this tag ID')
    parser.add_argument('--camera-ip', default='192.168.192.11')
    parser.add_argument('--timeout', type=float, default=12.0)
    parser.add_argument('--calibration', type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument('--output-image', type=Path, help='optional annotated image')
    parser.add_argument('--output-json', type=Path, help='optional JSON result')
    args = parser.parse_args()

    if not math.isfinite(args.tag_size_mm) or args.tag_size_mm <= 0:
        raise AprilTagPoseError('--tag-size-mm必须是大于0的实际毫米尺寸')

    camera_matrix, distortion, calibration_size = load_intrinsics(
        args.calibration
    )
    image, image_source = load_image(args)
    image_size = (image.shape[1], image.shape[0])
    if image_size != calibration_size:
        raise AprilTagPoseError(
            f'图像分辨率{image_size}与内参分辨率{calibration_size}不一致；'
            '禁止直接套用该内参'
        )

    families = (
        list(APRILTAG_DICTIONARIES)
        if args.family == 'auto'
        else [args.family]
    )
    detections = []
    rejected_count = 0
    for family in families:
        family_detections, rejected = detect_tags(image, family)
        rejected_count += len(rejected)
        detections.extend(
            (family, tag_id, corners)
            for tag_id, corners in family_detections
        )
    if args.tag_id is not None:
        detections = [item for item in detections if item[1] == args.tag_id]
    if not detections:
        if args.output_image is not None:
            debug_image = args.output_image.expanduser().resolve()
            debug_image.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(debug_image), image)
            print(f'未检测原图：{debug_image}', file=sys.stderr)
        requested = (
            f' ID={args.tag_id}' if args.tag_id is not None else ''
        )
        raise AprilTagPoseError(
            f'未检测到 AprilTag（family={args.family}{requested}）；'
            '请检查标签族、清晰度、曝光和标签是否完整可见'
        )

    results = []
    annotated = image.copy()
    for family, tag_id, corners in detections:
        pose = estimate_pose(
            corners,
            args.tag_size_mm,
            camera_matrix,
            distortion,
        )
        serializable = {
            key: value
            for key, value in pose.items()
            if key not in {'object_points', 'rvec_array', 'tvec_array'}
        }
        serializable.update(
            {
                'id': tag_id,
                'family': family,
                'tag_size_mm': args.tag_size_mm,
                'corners_px': corners.tolist(),
            }
        )
        results.append(serializable)
        cv2.polylines(
            annotated, [corners.astype(np.int32)], True, (0, 255, 0), 2
        )
        cv2.drawFrameAxes(
            annotated,
            camera_matrix,
            distortion,
            pose['rvec_array'],
            pose['tvec_array'],
            args.tag_size_mm * 0.5,
            2,
        )
        x, y, z = pose['xyz_mm']
        origin = tuple(corners.mean(axis=0).astype(int))
        cv2.putText(
            annotated,
            f'ID {tag_id}: X={x:.1f} Y={y:.1f} Z={z:.1f} mm',
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    output = {
        'coordinate_frame': 'opencv_camera_x_right_y_down_z_forward',
        'image_source': image_source,
        'calibration': str(args.calibration.expanduser().resolve()),
        'image_size': list(image_size),
        'detections': results,
        'rejected_candidate_count': rejected_count,
    }
    print('坐标系：相机X向右、Y向下、Z沿镜头向前')
    print(f'图像：{image_source}')
    print(f'搜索标签族：{args.family}，实际边长：{args.tag_size_mm:g} mm')
    for result in results:
        x, y, z = result['xyz_mm']
        roll, pitch, yaw = result['rpy_degrees']
        print(f"AprilTag family={result['family']} ID={result['id']}")
        print(f'  X = {x:.3f} mm')
        print(f'  Y = {y:.3f} mm')
        print(f'  Z = {z:.3f} mm')
        print(f'  RPY = [{roll:.3f}, {pitch:.3f}, {yaw:.3f}] deg')
        print(f"  重投影RMS = {result['reprojection_rms_px']:.4f} px")

    if args.output_image is not None:
        output_image = args.output_image.expanduser().resolve()
        output_image.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output_image), annotated):
            raise AprilTagPoseError(f'无法写入标注图：{output_image}')
        print(f'标注图：{output_image}')
    if args.output_json is not None:
        output_json = args.output_json.expanduser().resolve()
        output_json.parent.mkdir(parents=True, exist_ok=True)
        output_json.write_text(
            json.dumps(output, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        print(f'JSON：{output_json}')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except AprilTagPoseError as exc:
        print(f'错误：{exc}', file=sys.stderr)
        raise SystemExit(2)
