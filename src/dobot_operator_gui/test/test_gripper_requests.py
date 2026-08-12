from types import SimpleNamespace

import pytest

from dobot_operator_gui.ros_client import DobotRosClient
from dobot_operator_gui.ros_client import DobotServiceError


class FakeGripperClient:
    _require_gripper = DobotRosClient._require_gripper
    _write_gripper_register = DobotRosClient._write_gripper_register
    initialize_gripper = DobotRosClient.initialize_gripper
    set_gripper_force = DobotRosClient.set_gripper_force
    set_gripper_position = DobotRosClient.set_gripper_position
    read_gripper_status = DobotRosClient.read_gripper_status

    def __init__(self):
        self.gripper_index = 2
        self.calls = []

    def _call(self, name, request, timeout=None):
        self.calls.append((name, request))
        if name == 'GetHoldRegs':
            return SimpleNamespace(value='1,2,875')
        return SimpleNamespace(res=0)


def test_ag95_register_writes_are_correct():
    client = FakeGripperClient()
    client.initialize_gripper()
    client.set_gripper_force(20)
    client.set_gripper_position(1000)

    commands = [
        (name, request.index, request.addr, request.count, request.val_tab)
        for name, request in client.calls
    ]
    assert commands == [
        ('SetHoldRegs', 2, 0x0100, 1, '{1}'),
        ('SetHoldRegs', 2, 0x0101, 1, '{20}'),
        ('SetHoldRegs', 2, 0x0103, 1, '{1000}'),
    ]


def test_ag95_status_read_uses_three_holding_registers():
    client = FakeGripperClient()

    assert client.read_gripper_status() == (1, 2, 875)
    name, request = client.calls[0]
    assert name == 'GetHoldRegs'
    assert request.index == 2
    assert request.addr == 0x0200
    assert request.count == 3
    assert request.val_type == 'U16'


@pytest.mark.parametrize('position', [-1, 1001])
def test_reject_out_of_range_position(position):
    with pytest.raises(DobotServiceError):
        FakeGripperClient().set_gripper_position(position)


@pytest.mark.parametrize('force', [0, 19, 101])
def test_reject_out_of_range_force(force):
    with pytest.raises(DobotServiceError):
        FakeGripperClient().set_gripper_force(force)


class FakeGripperWaitClient:
    initialize_gripper_and_wait = DobotRosClient.initialize_gripper_and_wait
    move_gripper_and_wait = DobotRosClient.move_gripper_and_wait

    def __init__(self, statuses):
        self.statuses = iter(statuses)
        self.calls = []

    def initialize_gripper(self):
        self.calls.append(('initialize',))

    def set_gripper_position(self, position):
        self.calls.append(('position', position))

    def read_gripper_status(self):
        return next(self.statuses)


def test_wait_for_gripper_initialization_feedback():
    client = FakeGripperWaitClient([(1, 1, 1000)])

    client.initialize_gripper_and_wait()

    assert client.calls == [('initialize',)]


def test_wait_for_gripper_position_feedback():
    client = FakeGripperWaitClient([(1, 1, 500)])

    result = client.move_gripper_and_wait(500)

    assert result == (1, 1, 500)
    assert client.calls == [('position', 500)]


def test_gripper_object_detection_completes_closing_step():
    client = FakeGripperWaitClient([(1, 2, 420)])

    assert client.move_gripper_and_wait(0) == (1, 2, 420)


def test_gripper_drop_feedback_fails_step():
    client = FakeGripperWaitClient([(1, 3, 420)])

    with pytest.raises(DobotServiceError, match='物体脱落'):
        client.move_gripper_and_wait(0)


class FakeCaptureClient:
    capture_stable_state = DobotRosClient.capture_stable_state

    def __init__(self, angles):
        self.angles = iter(angles)

    def get_mode(self):
        return '6'

    def get_angles(self):
        return next(self.angles)

    def get_pose(self):
        return (100.0, 200.0, 300.0, 10.0, 20.0, 30.0)


def test_capture_accepts_stationary_robot():
    joints = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    client = FakeCaptureClient([joints, joints])

    captured_joints, pose, mode = client.capture_stable_state()

    assert captured_joints == joints
    assert pose == (100.0, 200.0, 300.0, 10.0, 20.0, 30.0)
    assert mode == '6'


def test_capture_rejects_moving_robot():
    client = FakeCaptureClient(
        [
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
        ]
    )

    with pytest.raises(DobotServiceError, match='仍在移动'):
        client.capture_stable_state()


class FakeResumeClient:
    resume_from_pause = DobotRosClient.resume_from_pause

    def __init__(self, mode='10', errors='{[[],[],[],[],[],[],[]]}'):
        self.mode = mode
        self.errors = errors
        self.calls = []

    def get_mode(self):
        return self.mode

    def get_error_ids(self):
        return self.errors

    def _call(self, name, request, timeout=None):
        self.calls.append(name)
        return SimpleNamespace(res=0)

    def _wait_for_mode(self, accepted, timeout):
        return '5'


def test_resume_pause_checks_state_and_uses_continue():
    client = FakeResumeClient()

    assert client.resume_from_pause() == '5'
    assert client.calls == ['Continue']


def test_resume_pause_rejects_remaining_alarm_ids():
    client = FakeResumeClient(errors='{[[-3],[],[],[],[],[],[]]}')

    with pytest.raises(DobotServiceError, match='仍存在报警码'):
        client.resume_from_pause()
    assert client.calls == []


def test_resume_pause_rejects_wrong_mode():
    client = FakeResumeClient(mode='9')

    with pytest.raises(DobotServiceError, match='RobotMode=10'):
        client.resume_from_pause()
