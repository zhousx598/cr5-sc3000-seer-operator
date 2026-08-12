"""AprilTag-referenced rigid correction for taught Cartesian targets."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Sequence

import numpy as np

from .core import OperatorInputError
from .core import validate_point_name
from .handeye_transform import dobot_pose_to_matrix
from .handeye_transform import rotation_to_rpy_degrees


class VisualCorrectionError(RuntimeError):
    """Raised when a reference or live tag measurement is unsafe to use."""


def _rigid_matrix(value: object, label: str) -> np.ndarray:
    try:
        matrix = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise VisualCorrectionError(f'{label}不是数值矩阵') from exc
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise VisualCorrectionError(f'{label}必须是有限的4x4矩阵')
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise VisualCorrectionError(f'{label}最后一行无效')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4):
        raise VisualCorrectionError(f'{label}旋转部分不正交')
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-4):
        raise VisualCorrectionError(f'{label}旋转行列式不为1')
    return matrix.copy()


def rotation_angle_degrees(rotation: object) -> float:
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    cosine = float(np.clip((np.trace(matrix) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def transform_distance(
    first: object, second: object
) -> tuple[float, float]:
    first_matrix = _rigid_matrix(first, 'first_transform')
    second_matrix = _rigid_matrix(second, 'second_transform')
    translation = float(
        np.linalg.norm(first_matrix[:3, 3] - second_matrix[:3, 3])
    )
    rotation = rotation_angle_degrees(
        first_matrix[:3, :3] @ second_matrix[:3, :3].T
    )
    return translation, rotation


def average_transforms(transforms: Sequence[object]) -> np.ndarray:
    """Average translations and project the mean rotation back to SO(3)."""
    matrices = [
        _rigid_matrix(value, f'测量矩阵{index + 1}')
        for index, value in enumerate(transforms)
    ]
    if not matrices:
        raise VisualCorrectionError('没有可用的AprilTag测量')
    translation = np.median(
        np.stack([matrix[:3, 3] for matrix in matrices]), axis=0
    )
    rotation_sum = np.sum(
        np.stack([matrix[:3, :3] for matrix in matrices]), axis=0
    )
    u, unused_singular_values, vt = np.linalg.svd(rotation_sum)
    sign = 1.0 if np.linalg.det(u @ vt) >= 0.0 else -1.0
    rotation = u @ np.diag([1.0, 1.0, sign]) @ vt
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def matrix_to_dobot_pose(matrix: object) -> tuple[float, ...]:
    transform = _rigid_matrix(matrix, '目标位姿')
    rpy = rotation_to_rpy_degrees(transform[:3, :3])
    return tuple(float(value) for value in (*transform[:3, 3], *rpy))


def _capture_metadata_path(points_dir: Path, name: str) -> Path:
    point_name = validate_point_name(name)
    group = Path(points_dir).expanduser().resolve() / point_name
    return group / f'{point_name}_capture.json'


def load_reference_tag_transform(
    points_dir: Path,
    reference_capture: str,
    family: str,
    tag_id: int,
    tag_size_mm: float,
    handeye_transform,
    max_reprojection_rms_px: float,
) -> np.ndarray:
    """Load B_ref_T_tag from a grouped, image-backed teaching capture."""
    metadata_path = _capture_metadata_path(points_dir, reference_capture)
    try:
        document = json.loads(metadata_path.read_text(encoding='utf-8'))
    except FileNotFoundError as exc:
        raise VisualCorrectionError(
            f'参考采集元数据不存在：{metadata_path}'
        ) from exc
    except (OSError, json.JSONDecodeError) as exc:
        raise VisualCorrectionError(f'无法读取参考采集：{exc}') from exc

    try:
        detections = document['apriltag']['detections']
        tool_pose = document['tool_pose']
    except (KeyError, TypeError) as exc:
        raise VisualCorrectionError(
            '参考采集缺少apriltag.detections或tool_pose；'
            '请在理想到站位置重新同步拍照保存'
        ) from exc
    if not isinstance(detections, list):
        raise VisualCorrectionError('参考采集的AprilTag检测格式错误')

    matching = [
        detection
        for detection in detections
        if isinstance(detection, dict)
        and detection.get('family') == family
        and detection.get('id') == tag_id
    ]
    if len(matching) != 1:
        raise VisualCorrectionError(
            f'参考采集中要求唯一 {family} ID={tag_id}，实际 {len(matching)} 个'
        )
    detection = matching[0]
    try:
        captured_size = float(detection['tag_size_mm'])
        reprojection = float(detection['reprojection_rms_px'])
    except (KeyError, TypeError, ValueError) as exc:
        raise VisualCorrectionError('参考AprilTag缺少尺寸或重投影误差') from exc
    if not math.isclose(captured_size, tag_size_mm, abs_tol=1e-6):
        raise VisualCorrectionError(
            f'参考采集标签尺寸为 {captured_size:g} mm，'
            f'当前队列配置为 {tag_size_mm:g} mm'
        )
    if (
        not math.isfinite(reprojection)
        or reprojection > max_reprojection_rms_px
    ):
        raise VisualCorrectionError(
            f'参考AprilTag重投影误差 {reprojection:.3f} px 超过 '
            f'{max_reprojection_rms_px:.3f} px'
        )
    try:
        transformed = handeye_transform.transform_detection(
            tool_pose, detection
        )
        return _rigid_matrix(transformed['matrix'], 'B_ref_T_tag')
    except (KeyError, TypeError, ValueError, OperatorInputError) as exc:
        raise VisualCorrectionError(f'参考AprilTag基座变换失败：{exc}') from exc


@dataclass(frozen=True)
class VisualCorrectionResult:
    """One correction shared by all subsequent corrected moves in a queue."""

    reference_base_T_tag: np.ndarray
    current_base_T_tag: np.ndarray
    correction: np.ndarray
    sample_count: int
    maximum_reprojection_rms_px: float
    sample_translation_spread_mm: float
    sample_rotation_spread_deg: float
    correction_translation_mm: float
    correction_rotation_deg: float
    maximum_target_translation_mm: float
    maximum_target_rotation_deg: float

    def corrected_pose(
        self, taught_pose: Sequence[float]
    ) -> tuple[float, ...]:
        taught = dobot_pose_to_matrix(taught_pose)
        corrected = self.correction @ taught
        target_translation, target_rotation = transform_distance(
            taught, corrected
        )
        if target_translation > self.maximum_target_translation_mm:
            raise VisualCorrectionError(
                f'点位实际纠偏位移 {target_translation:.2f} mm 超过 '
                f'{self.maximum_target_translation_mm:.2f} mm，拒绝运动'
            )
        if target_rotation > self.maximum_target_rotation_deg:
            raise VisualCorrectionError(
                f'点位实际纠偏旋转 {target_rotation:.2f}° 超过 '
                f'{self.maximum_target_rotation_deg:.2f}°，拒绝运动'
            )
        return matrix_to_dobot_pose(corrected)

    @property
    def summary(self) -> str:
        xyz = self.correction[:3, 3]
        rpy = rotation_to_rpy_degrees(self.correction[:3, :3])
        return (
            f'纠偏ΔXYZ=[{xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f}] mm，'
            f'ΔRPY=[{rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}]°，'
            f'总平移={self.correction_translation_mm:.2f} mm，'
            f'总旋转={self.correction_rotation_deg:.2f}°'
        )


def build_visual_correction(
    reference_base_T_tag: object,
    current_measurements: Sequence[object],
    reprojection_errors_px: Sequence[float],
    *,
    max_translation_mm: float,
    max_rotation_deg: float,
    max_sample_translation_spread_mm: float,
    max_sample_rotation_spread_deg: float,
) -> VisualCorrectionResult:
    reference = _rigid_matrix(reference_base_T_tag, 'B_ref_T_tag')
    measurements = [
        _rigid_matrix(value, f'B_now_T_tag[{index}]')
        for index, value in enumerate(current_measurements)
    ]
    if not measurements:
        raise VisualCorrectionError('没有实时AprilTag测量')
    errors = [float(value) for value in reprojection_errors_px]
    if len(errors) != len(measurements) or not all(
        math.isfinite(value) and value >= 0.0 for value in errors
    ):
        raise VisualCorrectionError('重投影误差数量或数值无效')

    current = average_transforms(measurements)
    spreads = [transform_distance(matrix, current) for matrix in measurements]
    translation_spread = max(value[0] for value in spreads)
    rotation_spread = max(value[1] for value in spreads)
    if translation_spread > max_sample_translation_spread_mm:
        raise VisualCorrectionError(
            f'多帧AprilTag平移离散 {translation_spread:.2f} mm 超过 '
            f'{max_sample_translation_spread_mm:.2f} mm'
        )
    if rotation_spread > max_sample_rotation_spread_deg:
        raise VisualCorrectionError(
            f'多帧AprilTag旋转离散 {rotation_spread:.2f}° 超过 '
            f'{max_sample_rotation_spread_deg:.2f}°'
        )

    correction = current @ np.linalg.inv(reference)
    correction_translation = float(np.linalg.norm(correction[:3, 3]))
    correction_rotation = rotation_angle_degrees(correction[:3, :3])
    if correction_translation > max_translation_mm:
        raise VisualCorrectionError(
            f'所需纠偏平移 {correction_translation:.2f} mm 超过 '
            f'{max_translation_mm:.2f} mm，拒绝执行'
        )
    if correction_rotation > max_rotation_deg:
        raise VisualCorrectionError(
            f'所需纠偏旋转 {correction_rotation:.2f}° 超过 '
            f'{max_rotation_deg:.2f}°，拒绝执行'
        )
    return VisualCorrectionResult(
        reference_base_T_tag=reference,
        current_base_T_tag=current,
        correction=correction,
        sample_count=len(measurements),
        maximum_reprojection_rms_px=max(errors),
        sample_translation_spread_mm=translation_spread,
        sample_rotation_spread_deg=rotation_spread,
        correction_translation_mm=correction_translation,
        correction_rotation_deg=correction_rotation,
        maximum_target_translation_mm=max_translation_mm,
        maximum_target_rotation_deg=max_rotation_deg,
    )
