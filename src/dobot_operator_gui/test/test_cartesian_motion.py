import threading

import pytest

from dobot_operator_gui.ros_client import DobotRosClient


class MotionHarness:
    _pose_error = staticmethod(DobotRosClient._pose_error)

    def __init__(self):
        self.motion_cancel = threading.Event()
        self.poses = [
            (0, 0, 0, 0, 0, 0),
            (10, 0, 0, 0, 0, 0),
            (10, 0, 0, 0, 0, 0),
        ]
        self.calls = []

    def get_mode(self):
        return '5'

    def get_pose(self, user=0, tool=0):
        assert (user, tool) == (0, 0)
        return self.poses.pop(0)

    def set_speed_factor(self, value):
        self.calls.append(('speed_factor', value))

    def _call(self, name, request):
        self.calls.append((name, request))


def test_pose_error_handles_rpy_wraparound():
    position, orientation = DobotRosClient._pose_error(
        (0, 0, 0, 0, 0, 179),
        (0, 0, 0, 0, 0, -179),
    )
    assert position == 0
    assert orientation == pytest.approx(2)


def test_linear_pose_move_uses_user0_tool0_and_polls(monkeypatch):
    monkeypatch.setattr(
        'dobot_operator_gui.ros_client.time.sleep', lambda unused: None
    )
    harness = MotionHarness()

    result = DobotRosClient.move_to_pose(
        harness,
        (10, 0, 0, 0, 0, 0),
        'linear',
        5,
        6,
        7,
        1,
        1,
    )

    assert result == (10, 0, 0, 0, 0, 0)
    assert harness.calls[0] == ('speed_factor', 5)
    name, request = harness.calls[1]
    assert name == 'MovL'
    assert request.x == 10
    assert request.param_value == [
        'SpeedL=6', 'AccL=7', 'User=0', 'Tool=0'
    ]
