import math
import threading
import time

import pytest
from sensor_msgs.msg import JointState

from dobot_msgs_v3.msg import ToolVectorActual
from dobot_operator_gui.main_window import DobotOperatorWindow
from dobot_operator_gui.ros_client import DobotRosClient
from dobot_operator_gui.ros_client import DobotServiceError


class FeedbackHarness:
    pass


class LabelHarness:
    def __init__(self):
        self.text = ''
        self.style = ''

    def setText(self, value):
        self.text = value

    def setStyleSheet(self, value):
        self.style = value


def make_feedback_harness():
    client = FeedbackHarness()
    client._robot_feedback_lock = threading.Lock()
    client._feedback_angles = None
    client._feedback_angles_time = 0.0
    client._feedback_pose = None
    client._feedback_pose_time = 0.0
    client._get_feedback_value = (
        lambda field, topic, maximum_age=2.0: (
            DobotRosClient._get_feedback_value(
                client, field, topic, maximum_age
            )
        )
    )
    client._get_feedback_angles = lambda: DobotRosClient._get_feedback_angles(
        client
    )
    client._get_feedback_pose = lambda: DobotRosClient._get_feedback_pose(client)
    return client


def test_joint_feedback_is_reordered_and_converted_to_degrees():
    client = make_feedback_harness()
    message = JointState()
    message.name = ['joint3', 'joint1', 'joint6', 'joint2', 'joint5', 'joint4']
    message.position = [0.3, 0.1, 0.6, 0.2, 0.5, 0.4]

    DobotRosClient._joint_feedback_callback(client, message)

    assert client._feedback_angles == pytest.approx(
        tuple(math.degrees(value) for value in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6))
    )
    assert client._feedback_angles_time > 0.0


def test_invalid_joint_feedback_does_not_replace_last_valid_sample():
    client = make_feedback_harness()
    client._feedback_angles = (1.0,) * 6
    message = JointState()
    message.name = ['joint1']
    message.position = [float('nan')]

    DobotRosClient._joint_feedback_callback(client, message)

    assert client._feedback_angles == (1.0,) * 6


def test_tool_feedback_caches_finite_pose():
    client = make_feedback_harness()
    message = ToolVectorActual()
    values = (100.0, -20.0, 730.0, 10.0, 20.0, -30.0)
    (
        message.x,
        message.y,
        message.z,
        message.rx,
        message.ry,
        message.rz,
    ) = values

    DobotRosClient._tool_feedback_callback(client, message)

    assert client._feedback_pose == values
    assert client._feedback_pose_time > 0.0


def test_read_status_uses_dashboard_when_available():
    client = FeedbackHarness()
    client.get_mode = lambda: '5'
    client.get_angles = lambda: (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    client.get_pose = lambda: (100.0, 200.0, 300.0, 10.0, 20.0, 30.0)

    status = DobotRosClient.read_status(client)

    assert status['mode'] == '5'
    assert status['angles'][0] == 1.0
    assert status['pose'][2] == 300.0
    assert status['fallbacks'] == ()


def test_read_status_falls_back_to_fresh_feedback():
    client = make_feedback_harness()
    client.get_mode = lambda: '4'
    client.get_angles = lambda: (_ for _ in ()).throw(
        DobotServiceError('GetAngle 返回 res=-1')
    )
    client.get_pose = lambda: (_ for _ in ()).throw(
        DobotServiceError('GetPose 返回 res=-1')
    )
    client._feedback_angles = (357.0, 55.0, -112.0, 69.0, 111.0, 204.0)
    client._feedback_angles_time = time.monotonic()
    client._feedback_pose = (133.0, -63.0, 736.0, 78.0, -20.0, 113.0)
    client._feedback_pose_time = time.monotonic()

    status = DobotRosClient.read_status(client)

    assert status['mode'] == '4'
    assert status['angles'][0] == 357.0
    assert status['pose'][2] == 736.0
    assert len(status['fallbacks']) == 2
    assert '/joint_states_robot' in status['fallbacks'][0]
    assert 'ToolVectorActual' in status['fallbacks'][1]


def test_status_ui_reports_connected_feedback_fallback():
    window = FeedbackHarness()
    window.mode_label = LabelHarness()
    window.mode_badge = LabelHarness()
    window.joints_label = LabelHarness()
    window.pose_label = LabelHarness()
    window.driver_badge = LabelHarness()
    window.ros_label = LabelHarness()

    DobotOperatorWindow._show_status(
        window,
        {
            'mode': '4',
            'angles': (357.0, 55.0, -112.0, 69.0, 111.0, 204.0),
            'pose': (133.0, -63.0, 736.0, 78.0, -20.0, 113.0),
            'fallbacks': (
                '关节角使用 /joint_states_robot（GetAngle 返回 res=-1）',
                '末端位姿使用 ToolVectorActual（GetPose 返回 res=-1）',
            ),
        },
    )

    assert window.driver_badge.text == '驱动：已连接（实时反馈兜底）'
    assert 'background:#9b6800' in window.driver_badge.style
    assert 'GetAngle 返回 res=-1' in window.ros_label.text
    assert window.mode_badge.text == '模式：4'


def test_stale_feedback_keeps_status_read_fail_closed():
    client = make_feedback_harness()
    client.get_mode = lambda: '4'
    client.get_angles = lambda: (_ for _ in ()).throw(
        DobotServiceError('GetAngle 返回 res=-1')
    )
    client._feedback_angles = (0.0,) * 6
    client._feedback_angles_time = time.monotonic() - 5.0

    with pytest.raises(DobotServiceError, match='已过期'):
        DobotRosClient.read_status(client)
