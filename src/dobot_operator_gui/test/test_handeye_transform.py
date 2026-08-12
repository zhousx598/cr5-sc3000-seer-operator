import json

import numpy as np
import pytest

from dobot_operator_gui.handeye_transform import HandEyeTransform
from dobot_operator_gui.handeye_transform import HandEyeTransformError
from dobot_operator_gui.handeye_transform import dobot_pose_to_matrix


def _write_handeye(path, matrix):
    path.write_text(
        json.dumps(
            {
                'selected_method': 'test',
                'input': {'point_ids_used': [1, 2, 3]},
                'gripper_T_camera': {'matrix': matrix},
            }
        ),
        encoding='utf-8',
    )


def test_transform_camera_detection_to_base(tmp_path):
    calibration = tmp_path / 'handeye.json'
    tool0_T_camera = np.eye(4)
    tool0_T_camera[:3, 3] = [10.0, 0.0, -5.0]
    _write_handeye(calibration, tool0_T_camera.tolist())
    transform = HandEyeTransform(calibration)
    detection = {
        'xyz_mm': [1.0, 2.0, 3.0],
        'rotation_matrix': np.eye(3).tolist(),
    }

    result = transform.transform_detection(
        [100.0, 200.0, 300.0, 0.0, 0.0, 0.0], detection
    )

    assert result['xyz_mm'] == pytest.approx([111.0, 202.0, 298.0])
    assert result['rpy_degrees'] == pytest.approx([0.0, 0.0, 0.0])
    assert result['coordinate_frame'] == 'dobot_user0_base'
    assert result['handeye_method'] == 'test'


def test_dobot_pose_uses_fixed_axis_xyz():
    matrix = dobot_pose_to_matrix([0.0, 0.0, 0.0, 0.0, 0.0, 90.0])
    assert matrix[:3, :3] @ [1.0, 0.0, 0.0] == pytest.approx(
        [0.0, 1.0, 0.0]
    )


def test_reject_non_rigid_handeye_matrix(tmp_path):
    calibration = tmp_path / 'bad.json'
    matrix = np.eye(4)
    matrix[0, 0] = 2.0
    _write_handeye(calibration, matrix.tolist())
    with pytest.raises(HandEyeTransformError, match='不正交'):
        HandEyeTransform(calibration)
