"""Rigid transforms from SC3000 camera coordinates to Dobot User0/base."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np


WORKSPACE = Path(
    os.environ.get('DOBOT_WS', str(Path.home() / 'dobot_ws'))
).expanduser()
DEFAULT_HAND_EYE_CALIBRATION = (
    WORKSPACE / 'points/calibration_results/sc3000_cr5_handeye_first20.json'
)


class HandEyeTransformError(RuntimeError):
    """Raised when the hand-eye file or a pose is invalid."""


def rotation_xyz_degrees(rx: float, ry: float, rz: float) -> np.ndarray:
    """Return Dobot fixed-axis XYZ rotation: Rz(rz) @ Ry(ry) @ Rx(rx)."""
    x, y, z = np.deg2rad([rx, ry, rz])
    cx, sx = math.cos(x), math.sin(x)
    cy, sy = math.cos(y), math.sin(y)
    cz, sz = math.cos(z), math.sin(z)
    rotation_x = np.array(
        [[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]]
    )
    rotation_y = np.array(
        [[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]]
    )
    rotation_z = np.array(
        [[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]]
    )
    return rotation_z @ rotation_y @ rotation_x


def transform_matrix(rotation, translation) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    result[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return result


def dobot_pose_to_matrix(pose) -> np.ndarray:
    values = np.asarray(pose, dtype=np.float64)
    if values.shape != (6,) or not np.all(np.isfinite(values)):
        raise HandEyeTransformError(
            'Tool0位姿必须是有限的[X,Y,Z,Rx,Ry,Rz]六维数组'
        )
    return transform_matrix(
        rotation_xyz_degrees(*values[3:]), values[:3]
    )


def rotation_to_rpy_degrees(rotation: np.ndarray) -> list[float]:
    """Convert Rz(yaw) @ Ry(pitch) @ Rx(roll) to degrees."""
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        roll = math.atan2(-matrix[1, 2], matrix[1, 1])
        yaw = 0.0
    return np.rad2deg([roll, pitch, yaw]).tolist()


def _validate_rigid_transform(matrix: np.ndarray, label: str) -> None:
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise HandEyeTransformError(f'{label}不是有限的4x4矩阵')
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise HandEyeTransformError(f'{label}最后一行无效')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        raise HandEyeTransformError(f'{label}旋转部分不正交')
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-5):
        raise HandEyeTransformError(f'{label}旋转行列式不为1')


class HandEyeTransform:
    """Load Tool0_T_camera and convert AprilTag detections to User0/base."""

    def __init__(
        self, calibration_path: Path = DEFAULT_HAND_EYE_CALIBRATION
    ) -> None:
        self.calibration_path = calibration_path.expanduser().resolve()
        try:
            document = json.loads(
                self.calibration_path.read_text(encoding='utf-8')
            )
        except OSError as exc:
            raise HandEyeTransformError(
                f'无法读取手眼标定文件：{self.calibration_path}：{exc}'
            ) from exc
        except json.JSONDecodeError as exc:
            raise HandEyeTransformError(
                f'手眼标定JSON无效：{self.calibration_path}：{exc}'
            ) from exc

        transform_entry = document.get('tool0_T_camera')
        if transform_entry is None:
            # OpenCV names this output gripper_T_camera.  In this project the
            # gripper input was explicitly GetPose(User=0, Tool=0).
            transform_entry = document.get('gripper_T_camera')
        if (
            not isinstance(transform_entry, dict)
            or 'matrix' not in transform_entry
        ):
            raise HandEyeTransformError(
                '手眼标定JSON缺少gripper_T_camera/tool0_T_camera.matrix'
            )
        self.tool0_T_camera = np.asarray(
            transform_entry['matrix'], dtype=np.float64
        )
        _validate_rigid_transform(self.tool0_T_camera, 'Tool0_T_camera')
        self.selected_method = str(document.get('selected_method', 'unknown'))
        self.point_ids_used = list(
            document.get('input', {}).get('point_ids_used', [])
        )

    def transform_detection(self, tool0_pose, detection: dict) -> dict:
        try:
            camera_translation = detection['xyz_mm']
            camera_rotation = detection['rotation_matrix']
        except (KeyError, TypeError) as exc:
            raise HandEyeTransformError(
                'AprilTag检测缺少xyz_mm或rotation_matrix'
            ) from exc
        camera_T_tag = transform_matrix(
            camera_rotation, camera_translation
        )
        _validate_rigid_transform(camera_T_tag, 'camera_T_tag')
        base_T_tool0 = dobot_pose_to_matrix(tool0_pose)
        base_T_tag = base_T_tool0 @ self.tool0_T_camera @ camera_T_tag
        return {
            'coordinate_frame': 'dobot_user0_base',
            'transform_notation': 'destination_T_source',
            'xyz_mm': base_T_tag[:3, 3].tolist(),
            'rpy_degrees': rotation_to_rpy_degrees(base_T_tag[:3, :3]),
            'rotation_matrix': base_T_tag[:3, :3].tolist(),
            'matrix': base_T_tag.tolist(),
            'tool_pose_user0_tool0_mm_deg': [
                float(value) for value in tool0_pose
            ],
            'handeye_calibration_file': str(self.calibration_path),
            'handeye_method': self.selected_method,
        }

    def calibration_summary(self) -> dict:
        return {
            'file': str(self.calibration_path),
            'method': self.selected_method,
            'point_ids_used': self.point_ids_used,
            'transform_notation': 'destination_T_source',
            'tool0_T_camera': self.tool0_T_camera.tolist(),
        }
