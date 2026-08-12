"""Validated, serializable command queues for the Dobot operator GUI."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import tempfile
import threading
from typing import Callable, Sequence

from .core import OperatorInputError
from .core import load_point
from .core import validate_point_name


SCHEMA_VERSION = 1
SUPPORTED_KINDS = {
    'move_point',
    'gripper_close_percent',
    'gripper_position',
    'gripper_force',
    'gripper_initialize',
    'wait',
}


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool):
        raise OperatorInputError(f'{label}必须是数值')
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise OperatorInputError(f'{label}必须是数值') from exc
    if not math.isfinite(number):
        raise OperatorInputError(f'{label}必须是有限数值')
    return number


def _integer_in_range(
    value: object, label: str, minimum: int, maximum: int
) -> int:
    number = _finite_number(value, label)
    if not number.is_integer():
        raise OperatorInputError(f'{label}必须是整数')
    result = int(number)
    if not minimum <= result <= maximum:
        raise OperatorInputError(f'{label}必须在 {minimum}~{maximum} 之间')
    return result


@dataclass(frozen=True)
class QueueCommand:
    """One validated queue instruction."""

    kind: str
    params: dict[str, object]

    def __post_init__(self) -> None:
        normalized = _normalize_command(self.kind, self.params)
        object.__setattr__(self, 'params', normalized)

    @property
    def title(self) -> str:
        return {
            'move_point': '移动到点位',
            'gripper_close_percent': '夹爪闭合比例',
            'gripper_position': '夹爪目标位置',
            'gripper_force': '设置夹持力',
            'gripper_initialize': '初始化夹爪',
            'wait': '等待',
        }[self.kind]

    @property
    def description(self) -> str:
        if self.kind == 'move_point':
            return (
                f'{self.params["point"]}；全局速度 '
                f'{self.params["speed_factor"]}%，SpeedJ '
                f'{self.params["speed_j"]}%，AccJ '
                f'{self.params["acc_j"]}%，容差 '
                f'{self.params["tolerance_deg"]:.2f}°'
            )
        if self.kind == 'gripper_close_percent':
            percent = self.params['percent']
            return (
                f'闭合 {percent}%（协议位置 '
                f'{close_percent_to_position(percent)}）'
            )
        if self.kind == 'gripper_position':
            return f'{self.params["position"]}（0闭合，1000打开）'
        if self.kind == 'gripper_force':
            return f'{self.params["force_percent"]}%'
        if self.kind == 'gripper_initialize':
            return '执行 DH-AG95 初始化'
        return f'{self.params["seconds"]:.2f} 秒'

    def to_dict(self) -> dict[str, object]:
        return {'kind': self.kind, 'params': dict(self.params)}

    @classmethod
    def from_dict(cls, value: object) -> 'QueueCommand':
        if not isinstance(value, dict):
            raise OperatorInputError('队列指令必须是 JSON 对象')
        if set(value) != {'kind', 'params'}:
            raise OperatorInputError('队列指令字段必须为 kind 和 params')
        kind = value['kind']
        params = value['params']
        if not isinstance(kind, str) or not isinstance(params, dict):
            raise OperatorInputError('队列指令 kind 或 params 格式错误')
        return cls(kind, params)


def _normalize_command(
    kind: str, params: dict[str, object]
) -> dict[str, object]:
    if kind not in SUPPORTED_KINDS:
        raise OperatorInputError(f'不支持的队列指令：{kind!r}')
    if not isinstance(params, dict):
        raise OperatorInputError('队列指令参数必须是对象')

    expected_keys = {
        'move_point': {
            'point', 'speed_factor', 'speed_j', 'acc_j', 'tolerance_deg'
        },
        'gripper_close_percent': {'percent'},
        'gripper_position': {'position'},
        'gripper_force': {'force_percent'},
        'gripper_initialize': set(),
        'wait': {'seconds'},
    }[kind]
    if set(params) != expected_keys:
        raise OperatorInputError(
            f'{kind} 参数字段错误；需要 {sorted(expected_keys)}'
        )

    if kind == 'move_point':
        point = params['point']
        if not isinstance(point, str):
            raise OperatorInputError('点位名称必须是字符串')
        tolerance = _finite_number(params['tolerance_deg'], '到位容差')
        if not 0.05 <= tolerance <= 5.0:
            raise OperatorInputError('到位容差必须在 0.05~5.0° 之间')
        return {
            'point': validate_point_name(point),
            'speed_factor': _integer_in_range(
                params['speed_factor'], '全局速度', 1, 100
            ),
            'speed_j': _integer_in_range(
                params['speed_j'], 'SpeedJ', 1, 100
            ),
            'acc_j': _integer_in_range(params['acc_j'], 'AccJ', 1, 100),
            'tolerance_deg': tolerance,
        }
    if kind == 'gripper_close_percent':
        return {
            'percent': _integer_in_range(
                params['percent'], '夹爪闭合比例', 0, 100
            )
        }
    if kind == 'gripper_position':
        return {
            'position': _integer_in_range(
                params['position'], '夹爪位置', 0, 1000
            )
        }
    if kind == 'gripper_force':
        return {
            'force_percent': _integer_in_range(
                params['force_percent'], '夹持力', 20, 100
            )
        }
    if kind == 'gripper_initialize':
        return {}
    seconds = _finite_number(params['seconds'], '等待时间')
    if not 0.1 <= seconds <= 3600.0:
        raise OperatorInputError('等待时间必须在 0.1~3600 秒之间')
    return {'seconds': seconds}


def close_percent_to_position(percent: object) -> int:
    """Map user-facing closure (0=open) to DH-AG95 position (1000=open)."""
    value = _integer_in_range(percent, '夹爪闭合比例', 0, 100)
    return round(1000 * (100 - value) / 100)


def save_queue(path: Path, commands: Sequence[QueueCommand]) -> None:
    """Atomically save a queue as versioned UTF-8 JSON."""
    destination = Path(path).expanduser()
    payload = {
        'schema_version': SCHEMA_VERSION,
        'commands': [command.to_dict() for command in commands],
    }
    destination.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(payload, ensure_ascii=False, indent=2) + '\n'
    with tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        dir=destination.parent,
        prefix=f'.{destination.name}.',
        delete=False,
    ) as temporary:
        temporary.write(text)
        temporary_path = Path(temporary.name)
    temporary_path.replace(destination)


def load_queue(path: Path) -> list[QueueCommand]:
    """Load and validate every instruction before returning anything."""
    source = Path(path).expanduser()
    try:
        payload = json.loads(source.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise OperatorInputError(f'无法读取队列文件：{exc}') from exc
    if not isinstance(payload, dict):
        raise OperatorInputError('队列文件顶层必须是 JSON 对象')
    if set(payload) != {'schema_version', 'commands'}:
        raise OperatorInputError('队列文件字段不完整或包含未知字段')
    if payload['schema_version'] != SCHEMA_VERSION:
        raise OperatorInputError(
            f'不支持的队列版本：{payload["schema_version"]!r}'
        )
    if not isinstance(payload['commands'], list):
        raise OperatorInputError('commands 必须是数组')
    return [QueueCommand.from_dict(item) for item in payload['commands']]


@dataclass(frozen=True)
class QueueRunResult:
    completed: int
    total: int
    cancelled: bool


QueueProgress = Callable[[int, str, str], None]


class TaskQueueRunner:
    """Run already-confirmed instructions one at a time."""

    def __init__(self, ros_client) -> None:
        self.ros = ros_client

    def run(
        self,
        commands: Sequence[QueueCommand],
        points_dir: Path,
        cancel_event: threading.Event,
        progress: QueueProgress | None = None,
    ) -> QueueRunResult:
        steps = list(commands)
        completed = 0

        def report(index: int, state: str, text: str) -> None:
            if progress is not None:
                progress(index, state, text)

        for index, command in enumerate(steps):
            if cancel_event.is_set():
                self._mark_remaining_skipped(steps, index, report)
                return QueueRunResult(completed, len(steps), True)
            report(index, 'running', f'正在执行：{command.description}')
            try:
                step_completed = self._execute(
                    command, points_dir, cancel_event, index, report
                )
            except Exception as exc:
                report(index, 'error', f'{type(exc).__name__}: {exc}')
                self._mark_remaining_skipped(steps, index + 1, report)
                raise
            if not step_completed:
                self._mark_remaining_skipped(steps, index + 1, report)
                return QueueRunResult(completed, len(steps), True)
            completed += 1
            report(index, 'done', '完成')

        return QueueRunResult(completed, len(steps), False)

    def _execute(
        self,
        command: QueueCommand,
        points_dir: Path,
        cancel_event: threading.Event,
        index: int,
        report: QueueProgress,
    ) -> bool:
        params = command.params
        if command.kind == 'move_point':
            point = load_point(points_dir, params['point'])
            self.ros.move_to_joints(
                point.joints,
                params['speed_factor'],
                params['speed_j'],
                params['acc_j'],
                tolerance_deg=params['tolerance_deg'],
                progress=lambda text: report(index, 'running', text),
            )
        elif command.kind == 'gripper_close_percent':
            self.ros.move_gripper_and_wait(
                close_percent_to_position(params['percent'])
            )
        elif command.kind == 'gripper_position':
            self.ros.move_gripper_and_wait(params['position'])
        elif command.kind == 'gripper_force':
            self.ros.set_gripper_force(params['force_percent'])
        elif command.kind == 'gripper_initialize':
            self.ros.initialize_gripper_and_wait()
        elif cancel_event.wait(params['seconds']):
            report(index, 'cancelled', '等待期间收到停止请求')
            return False
        return True

    @staticmethod
    def _mark_remaining_skipped(
        steps: Sequence[QueueCommand],
        start: int,
        report: QueueProgress,
    ) -> None:
        for index in range(start, len(steps)):
            report(index, 'skipped', '未执行')
