from pathlib import Path
import threading

import pytest

from dobot_operator_gui.core import OperatorInputError
from dobot_operator_gui.core import save_point
from dobot_operator_gui.task_queue import close_percent_to_position
from dobot_operator_gui.task_queue import load_queue
from dobot_operator_gui.task_queue import QueueCommand
from dobot_operator_gui.task_queue import save_queue
from dobot_operator_gui.task_queue import TaskQueueRunner


def move_command(point='P1'):
    return QueueCommand(
        'move_point',
        {
            'point': point,
            'speed_factor': 5,
            'speed_j': 10,
            'acc_j': 10,
            'tolerance_deg': 0.5,
        },
    )


def agv_navigation_command(station='LM1'):
    return QueueCommand(
        'agv_navigate_station',
        {
            'station_id': station,
            'max_speed_mps': 0.08,
            'timeout_s': 300,
        },
    )


def agv_pose_navigation_command(name='LOCAL1'):
    return QueueCommand(
        'agv_navigate_pose',
        {
            'waypoint_name': name,
            'x': 1.2,
            'y': -0.4,
            'yaw': 0.5,
            'max_speed_mps': 0.05,
            'timeout_s': 300,
        },
    )


def correction_command(reference='REF'):
    return QueueCommand(
        'measure_apriltag_correction',
        {
            'reference_capture': reference,
            'family': 'tag36h11',
            'tag_id': 0,
            'tag_size_mm': 58.5,
            'camera_host': '192.168.192.11',
            'camera_timeout_s': 12,
            'samples': 3,
            'max_reprojection_rms_px': 1.5,
            'max_translation_mm': 50,
            'max_rotation_deg': 5,
            'max_sample_translation_spread_mm': 2,
            'max_sample_rotation_spread_deg': 1,
        },
    )


def corrected_move_command(point='P1'):
    return QueueCommand(
        'move_point_corrected',
        {
            'point': point,
            'motion_type': 'linear',
            'speed_factor': 5,
            'speed': 5,
            'acc': 5,
            'position_tolerance_mm': 1,
            'orientation_tolerance_deg': 1,
        },
    )


def test_close_percent_maps_to_ag95_position():
    assert close_percent_to_position(0) == 1000
    assert close_percent_to_position(50) == 500
    assert close_percent_to_position(100) == 0


@pytest.mark.parametrize('percent', [-1, 101, 1.5, float('nan')])
def test_reject_invalid_close_percent(percent):
    with pytest.raises(OperatorInputError):
        close_percent_to_position(percent)


def test_queue_json_round_trip(tmp_path: Path):
    commands = [
        move_command(),
        QueueCommand('gripper_force', {'force_percent': 20}),
        QueueCommand('gripper_close_percent', {'percent': 75}),
        QueueCommand('wait', {'seconds': 1.25}),
    ]
    path = tmp_path / 'pick.json'

    save_queue(path, commands)

    assert load_queue(path) == commands
    assert '"schema_version": 1' in path.read_text(encoding='utf-8')


def test_queue_loader_rejects_unknown_instruction(tmp_path: Path):
    path = tmp_path / 'bad.json'
    path.write_text(
        '{"schema_version": 1, "commands": '
        '[{"kind": "shell", "params": {}}]}',
        encoding='utf-8',
    )

    with pytest.raises(OperatorInputError, match='不支持'):
        load_queue(path)


class FakeRosClient:
    def __init__(self):
        self.calls = []

    def move_to_joints(
        self,
        target,
        speed_factor,
        speed_j,
        acc_j,
        tolerance_deg,
        progress,
    ):
        self.calls.append(
            (
                'move',
                tuple(target),
                speed_factor,
                speed_j,
                acc_j,
                tolerance_deg,
            )
        )
        progress('moving')

    def agv_navigate_and_wait(
        self,
        station_id,
        max_speed_mps,
        timeout_s,
        cancel_event,
        progress,
        travel_mode='auto',
    ):
        self.calls.append(
            ('agv', station_id, max_speed_mps, timeout_s, travel_mode)
        )
        progress('navigating')
        return not cancel_event.is_set()

    def agv_navigate_pose_and_wait(
        self,
        waypoint_name,
        x,
        y,
        yaw,
        max_speed_mps,
        timeout_s,
        cancel_event,
        progress,
        travel_mode='auto',
    ):
        self.calls.append(
            (
                'agv_pose',
                waypoint_name,
                x,
                y,
                yaw,
                max_speed_mps,
                timeout_s,
                travel_mode,
            )
        )
        progress('pose navigating')
        return not cancel_event.is_set()

    def set_gripper_force(self, force):
        self.calls.append(('force', force))

    def move_gripper_and_wait(self, position):
        self.calls.append(('gripper', position))

    def initialize_gripper_and_wait(self):
        self.calls.append(('initialize',))

    def move_to_pose(
        self,
        target,
        motion_type,
        speed_factor,
        speed,
        acc,
        position_tolerance_mm,
        orientation_tolerance_deg,
        progress,
    ):
        self.calls.append(
            (
                'pose',
                tuple(target),
                motion_type,
                speed_factor,
                speed,
                acc,
                position_tolerance_mm,
                orientation_tolerance_deg,
            )
        )
        progress('pose moving')
        return tuple(target)


class FakeCorrection:
    summary = 'test correction'

    def corrected_pose(self, pose):
        result = list(pose)
        result[0] += 10
        return tuple(result)


def test_runner_executes_steps_in_order(tmp_path: Path):
    save_point(
        tmp_path,
        'P1',
        (1, 2, 3, 4, 5, 6),
        (100, 200, 300, 10, 20, 30),
    )
    commands = [
        move_command(),
        QueueCommand('gripper_force', {'force_percent': 20}),
        QueueCommand('gripper_close_percent', {'percent': 75}),
    ]
    ros = FakeRosClient()
    events = []

    result = TaskQueueRunner(ros).run(
        commands,
        tmp_path,
        threading.Event(),
        lambda index, state, text: events.append((index, state, text)),
    )

    assert result.completed == 3
    assert result.cancelled is False
    assert [call[0] for call in ros.calls] == ['move', 'force', 'gripper']
    assert ros.calls[-1] == ('gripper', 250)
    assert [state for unused_index, state, unused_text in events].count(
        'done'
    ) == 3


def test_runner_can_navigate_before_visual_correction(tmp_path: Path):
    save_point(
        tmp_path,
        'P1',
        (1, 2, 3, 4, 5, 6),
        (100, 200, 300, 10, 20, 30),
    )

    ros = FakeRosClient()
    result = TaskQueueRunner(
        ros,
        lambda unused_params, unused_dir, unused_cancel, unused_progress: (
            FakeCorrection()
        ),
    ).run(
        [
            agv_navigation_command(),
            correction_command(),
            corrected_move_command(),
        ],
        tmp_path,
        threading.Event(),
    )

    assert result.completed == 3
    assert ros.calls[0] == ('agv', 'LM1', 0.08, 300.0, 'auto')
    assert ros.calls[1][0] == 'pose'


def test_runner_executes_local_pose_navigation(tmp_path: Path):
    ros = FakeRosClient()

    result = TaskQueueRunner(ros).run(
        [agv_pose_navigation_command()],
        tmp_path,
        threading.Event(),
    )

    assert result.completed == 1
    assert ros.calls == [
        (
            'agv_pose', 'LOCAL1', 1.2, -0.4, 0.5, 0.05, 300.0,
            'auto',
        )
    ]


def test_old_queue_navigation_defaults_to_auto(tmp_path: Path):
    path = tmp_path / 'old.json'
    path.write_text(
        '{"schema_version":1,"commands":[{"kind":'
        '"agv_navigate_station","params":{"station_id":"LM1",'
        '"max_speed_mps":0.08,"timeout_s":300}}]}',
        encoding='utf-8',
    )

    command = load_queue(path)[0]

    assert command.params['travel_mode'] == 'auto'


@pytest.mark.parametrize('travel_mode', ['auto', 'forward', 'backward'])
def test_queue_accepts_supported_agv_travel_modes(travel_mode):
    command = agv_navigation_command()
    params = dict(command.params)
    params['travel_mode'] = travel_mode

    assert QueueCommand('agv_navigate_station', params).params[
        'travel_mode'
    ] == travel_mode


def test_queue_rejects_invalid_agv_travel_mode():
    command = agv_navigation_command()
    params = dict(command.params)
    params['travel_mode'] = 'sideways'

    with pytest.raises(OperatorInputError, match='行驶方式'):
        QueueCommand('agv_navigate_station', params)


@pytest.mark.parametrize(
    'params',
    [
        {
            'waypoint_name': '',
            'x': 1.0,
            'y': 2.0,
            'yaw': 0.0,
            'max_speed_mps': 0.05,
            'timeout_s': 300,
        },
        {
            'waypoint_name': 'P1',
            'x': float('nan'),
            'y': 2.0,
            'yaw': 0.0,
            'max_speed_mps': 0.05,
            'timeout_s': 300,
        },
    ],
)
def test_reject_invalid_agv_pose_navigation_command(params):
    with pytest.raises(OperatorInputError):
        QueueCommand('agv_navigate_pose', params)


@pytest.mark.parametrize(
    'params',
    [
        {'station_id': '', 'max_speed_mps': 0.08, 'timeout_s': 300},
        {'station_id': 'LM1', 'max_speed_mps': 0.5, 'timeout_s': 300},
        {'station_id': 'LM1', 'max_speed_mps': 0.08, 'timeout_s': 2},
    ],
)
def test_reject_invalid_agv_navigation_command(params):
    with pytest.raises(OperatorInputError):
        QueueCommand('agv_navigate_station', params)


def test_runner_respects_stop_before_first_step(tmp_path: Path):
    ros = FakeRosClient()
    cancel = threading.Event()
    cancel.set()
    events = []

    result = TaskQueueRunner(ros).run(
        [QueueCommand('gripper_force', {'force_percent': 20})],
        tmp_path,
        cancel,
        lambda index, state, text: events.append((index, state, text)),
    )

    assert result.completed == 0
    assert result.cancelled is True
    assert ros.calls == []
    assert events[0][1] == 'skipped'


def test_runner_measures_once_then_executes_corrected_pose(tmp_path: Path):
    save_point(
        tmp_path,
        'P1',
        (1, 2, 3, 4, 5, 6),
        (100, 200, 300, 10, 20, 30),
    )
    provider_calls = []

    def provider(params, points_dir, cancel_event, progress):
        provider_calls.append((params['reference_capture'], points_dir))
        progress('measured')
        return FakeCorrection()

    ros = FakeRosClient()
    result = TaskQueueRunner(ros, provider).run(
        [correction_command(), corrected_move_command()],
        tmp_path,
        threading.Event(),
    )

    assert result.completed == 2
    assert provider_calls == [('REF', tmp_path)]
    assert ros.calls == [
        ('pose', (110, 200, 300, 10, 20, 30), 'linear', 5, 5, 5, 1.0, 1.0)
    ]


def test_corrected_move_rejects_missing_measurement(tmp_path: Path):
    save_point(
        tmp_path,
        'P1',
        (1, 2, 3, 4, 5, 6),
        (100, 200, 300, 10, 20, 30),
    )
    with pytest.raises(OperatorInputError, match='没有成功执行'):
        TaskQueueRunner(FakeRosClient()).run(
            [corrected_move_command()], tmp_path, threading.Event()
        )


def test_agv_navigation_invalidates_previous_visual_correction(
    tmp_path: Path,
):
    save_point(
        tmp_path,
        'P1',
        (1, 2, 3, 4, 5, 6),
        (100, 200, 300, 10, 20, 30),
    )
    provider = (
        lambda unused_params, unused_dir, unused_cancel, unused_progress: (
            FakeCorrection()
        )
    )

    with pytest.raises(OperatorInputError, match='没有成功执行'):
        TaskQueueRunner(FakeRosClient(), provider).run(
            [
                correction_command(),
                agv_navigation_command(),
                corrected_move_command(),
            ],
            tmp_path,
            threading.Event(),
        )
