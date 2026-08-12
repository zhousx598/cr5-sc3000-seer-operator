from pathlib import Path

import pytest

from dobot_point_manager.move_between_points import load_named_point
from dobot_point_manager.move_between_points import max_joint_error_deg
from dobot_point_manager.move_between_points import parse_joint_values
from dobot_point_manager.move_between_points import PointFileError
from dobot_point_manager.move_between_points import PointMover


def test_parse_saved_get_angle_output():
    text = (
        'response:\n'
        "GetAngle_Response(res=0, angle='{1,-2,3.5,4,5,6}')\n"
    )
    assert parse_joint_values(text) == (1.0, -2.0, 3.5, 4.0, 5.0, 6.0)


def test_reject_wrong_joint_count():
    with pytest.raises(PointFileError):
        parse_joint_values("angle='{1,2,3}'")


def test_reject_path_traversal(tmp_path: Path):
    with pytest.raises(PointFileError):
        load_named_point(tmp_path, '../P1')


def test_wrapped_max_joint_error():
    assert max_joint_error_deg((359, 0, 0, 0, 0, 0), (1, 0, 0, 0, 0, 0)) == 2


class FakePointMover:
    wait_until_arrived = PointMover.wait_until_arrived

    def __init__(self, samples, start_timeout=1.0, motion_timeout=10.0):
        self.samples = iter(samples)
        self.start_timeout = start_timeout
        self.motion_timeout = motion_timeout
        self.now = 0.0
        self.current_mode = '5'

    def get_angles(self):
        angles, self.current_mode = next(self.samples)
        return angles

    def get_robot_mode(self):
        return self.current_mode

    def get_error_ids_for_diagnostic(self):
        return '{}'

    def _monotonic(self):
        return self.now

    def _sleep(self, duration):
        self.now += duration


def test_wait_until_arrived_without_sync():
    start = (0.0,) * 6
    target = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    samples = [
        (start, '5'),
        ((0.1, 0.0, 0.0, 0.0, 0.0, 0.0), '7'),
        (target, '7'),
        (target, '5'),
        (target, '5'),
    ]
    mover = FakePointMover(samples)

    assert mover.wait_until_arrived(start, target, 0.01) == target


def test_wait_reports_queue_that_did_not_start():
    start = (0.0,) * 6
    target = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    mover = FakePointMover([(start, '5')] * 3)

    with pytest.raises(RuntimeError, match='机械臂没有开始运动'):
        mover.wait_until_arrived(start, target, 0.01)


def test_wait_stops_on_alarm_mode():
    start = (0.0,) * 6
    target = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    mover = FakePointMover([(start, '9')])

    with pytest.raises(RuntimeError, match='报警'):
        mover.wait_until_arrived(start, target, 0.01)
