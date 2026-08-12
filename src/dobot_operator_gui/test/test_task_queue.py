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

    def set_gripper_force(self, force):
        self.calls.append(('force', force))

    def move_gripper_and_wait(self, position):
        self.calls.append(('gripper', position))

    def initialize_gripper_and_wait(self):
        self.calls.append(('initialize',))


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
