import json

import numpy as np
import pytest

from dobot_operator_gui.handeye_transform import HandEyeTransform
from dobot_operator_gui.handeye_transform import transform_matrix
from dobot_operator_gui.handeye_transform import rotation_xyz_degrees
from dobot_operator_gui.visual_correction import build_visual_correction
from dobot_operator_gui.visual_correction import load_reference_tag_transform
from dobot_operator_gui.visual_correction import VisualCorrectionError


def _write_handeye(path):
    path.write_text(
        json.dumps(
            {
                'selected_method': 'test',
                'tool0_T_camera': {'matrix': np.eye(4).tolist()},
            }
        ),
        encoding='utf-8',
    )


def test_load_reference_capture_and_transform_to_base(tmp_path):
    calibration = tmp_path / 'handeye.json'
    _write_handeye(calibration)
    group = tmp_path / 'REF'
    group.mkdir()
    (group / 'REF_capture.json').write_text(
        json.dumps(
            {
                'tool_pose': [100, 200, 300, 0, 0, 0],
                'apriltag': {
                    'detections': [
                        {
                            'family': 'tag36h11',
                            'id': 0,
                            'tag_size_mm': 58.5,
                            'xyz_mm': [10, 20, 30],
                            'rotation_matrix': np.eye(3).tolist(),
                            'reprojection_rms_px': 0.25,
                        }
                    ]
                },
            }
        ),
        encoding='utf-8',
    )

    matrix = load_reference_tag_transform(
        tmp_path,
        'REF',
        'tag36h11',
        0,
        58.5,
        HandEyeTransform(calibration),
        1.0,
    )

    assert matrix[:3, 3] == pytest.approx([110, 220, 330])
    assert matrix[:3, :3] == pytest.approx(np.eye(3))


def test_build_correction_and_apply_to_taught_pose():
    reference = transform_matrix(np.eye(3), [400, -100, 200])
    delta = transform_matrix(rotation_xyz_degrees(0, 0, 5), [10, 20, 3])
    current = delta @ reference

    result = build_visual_correction(
        reference,
        [current, current, current],
        [0.2, 0.3, 0.25],
        max_translation_mm=50,
        max_rotation_deg=10,
        max_sample_translation_spread_mm=1,
        max_sample_rotation_spread_deg=1,
    )
    corrected = result.corrected_pose([100, 0, 0, 0, 0, 0])

    expected = delta @ transform_matrix(np.eye(3), [100, 0, 0])
    assert corrected[:3] == pytest.approx(expected[:3, 3])
    assert corrected[3:] == pytest.approx([0, 0, 5])
    assert result.correction_translation_mm == pytest.approx(
        np.linalg.norm([10, 20, 3])
    )
    assert result.correction_rotation_deg == pytest.approx(5)


def test_reject_correction_over_limit():
    reference = np.eye(4)
    current = transform_matrix(np.eye(3), [60, 0, 0])
    with pytest.raises(VisualCorrectionError, match='拒绝执行'):
        build_visual_correction(
            reference,
            [current],
            [0.1],
            max_translation_mm=50,
            max_rotation_deg=5,
            max_sample_translation_spread_mm=1,
            max_sample_rotation_spread_deg=1,
        )


def test_reject_target_shift_amplified_by_rotation():
    reference = np.eye(4)
    delta = transform_matrix(rotation_xyz_degrees(0, 0, 5), [0, 0, 0])
    result = build_visual_correction(
        reference,
        [delta],
        [0.1],
        max_translation_mm=20,
        max_rotation_deg=10,
        max_sample_translation_spread_mm=1,
        max_sample_rotation_spread_deg=1,
    )
    with pytest.raises(VisualCorrectionError, match='点位实际纠偏位移'):
        result.corrected_pose([1000, 0, 0, 0, 0, 0])


def test_reject_unstable_multiple_frames():
    reference = np.eye(4)
    first = transform_matrix(np.eye(3), [0, 0, 0])
    second = transform_matrix(np.eye(3), [5, 0, 0])
    with pytest.raises(VisualCorrectionError, match='多帧AprilTag平移离散'):
        build_visual_correction(
            reference,
            [first, second],
            [0.1, 0.1],
            max_translation_mm=50,
            max_rotation_deg=5,
            max_sample_translation_spread_mm=1,
            max_sample_rotation_spread_deg=1,
        )


def test_reference_rejects_wrong_tag_size(tmp_path):
    calibration = tmp_path / 'handeye.json'
    _write_handeye(calibration)
    group = tmp_path / 'REF'
    group.mkdir()
    (group / 'REF_capture.json').write_text(
        json.dumps(
            {
                'tool_pose': [0, 0, 0, 0, 0, 0],
                'apriltag': {
                    'detections': [
                        {
                            'family': 'tag36h11',
                            'id': 0,
                            'tag_size_mm': 5.85,
                            'xyz_mm': [0, 0, 100],
                            'rotation_matrix': np.eye(3).tolist(),
                            'reprojection_rms_px': 0.1,
                        }
                    ]
                },
            }
        ),
        encoding='utf-8',
    )
    with pytest.raises(VisualCorrectionError, match='标签尺寸'):
        load_reference_tag_transform(
            tmp_path,
            'REF',
            'tag36h11',
            0,
            58.5,
            HandEyeTransform(calibration),
            1.0,
        )
