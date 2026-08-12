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
TRAVEL_MODE_LABELS = {
    'auto': '自动',
    'forward': '正走',
    'backward': '倒走',
}
SUPPORTED_KINDS = {
    'agv_navigate_pose',
    'agv_navigate_station',
    'move_point',
    'measure_apriltag_correction',
    'move_point_corrected',
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
            'agv_navigate_pose': 'AGV导航到用户航点',
            'agv_navigate_station': 'AGV导航到航点',
            'move_point': '移动到点位',
            'measure_apriltag_correction': '测量AprilTag纠偏',
            'move_point_corrected': '移动到纠偏点位',
            'gripper_close_percent': '夹爪闭合比例',
            'gripper_position': '夹爪目标位置',
            'gripper_force': '设置夹持力',
            'gripper_initialize': '初始化夹爪',
            'wait': '等待',
        }[self.kind]

    @property
    def description(self) -> str:
        if self.kind == 'agv_navigate_pose':
            return (
                f'{self.params["waypoint_name"]}；'
                f'X={self.params["x"]:.3f} m，Y={self.params["y"]:.3f} m，'
                f'Yaw={math.degrees(self.params["yaw"]):.1f}°；'
                f'行驶方式 {TRAVEL_MODE_LABELS[self.params["travel_mode"]]}；'
                f'最大速度 {self.params["max_speed_mps"]:.2f} m/s'
            )
        if self.kind == 'agv_navigate_station':
            return (
                f'{self.params["station_id"]}；最大速度 '
                f'{self.params["max_speed_mps"]:.2f} m/s；'
                f'行驶方式 {TRAVEL_MODE_LABELS[self.params["travel_mode"]]}；'
                f'到站超时 {self.params["timeout_s"]:.0f} s'
            )
        if self.kind == 'move_point':
            return (
                f'{self.params["point"]}；全局速度 '
                f'{self.params["speed_factor"]}%，SpeedJ '
                f'{self.params["speed_j"]}%，AccJ '
                f'{self.params["acc_j"]}%，容差 '
                f'{self.params["tolerance_deg"]:.2f}°'
            )
        if self.kind == 'measure_apriltag_correction':
            return (
                f'参考采集={self.params["reference_capture"]}；'
                f'{self.params["family"]} ID={self.params["tag_id"]}；'
                f'{self.params["samples"]}帧；最大纠偏'
                f'{self.params["max_translation_mm"]:.1f} mm / '
                f'{self.params["max_rotation_deg"]:.1f}°'
            )
        if self.kind == 'move_point_corrected':
            motion = (
                '直线MovL'
                if self.params['motion_type'] == 'linear'
                else '关节MovJ'
            )
            return (
                f'{self.params["point"]}；{motion}；全局速度'
                f'{self.params["speed_factor"]}%；运动速度'
                f'{self.params["speed"]}%；加速度{self.params["acc"]}%'
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

    # Version-1 queues saved before travel mode support remain valid and use
    # controller/path defaults.  New saves always persist the explicit field.
    params = dict(params)
    if kind in {'agv_navigate_pose', 'agv_navigate_station'}:
        params.setdefault('travel_mode', 'auto')

    expected_keys = {
        'agv_navigate_pose': {
            'waypoint_name', 'x', 'y', 'yaw',
            'max_speed_mps', 'timeout_s', 'travel_mode',
        },
        'agv_navigate_station': {
            'station_id', 'max_speed_mps', 'timeout_s', 'travel_mode'
        },
        'move_point': {
            'point', 'speed_factor', 'speed_j', 'acc_j', 'tolerance_deg'
        },
        'measure_apriltag_correction': {
            'reference_capture', 'family', 'tag_id', 'tag_size_mm',
            'camera_host', 'camera_timeout_s', 'samples',
            'max_reprojection_rms_px', 'max_translation_mm',
            'max_rotation_deg', 'max_sample_translation_spread_mm',
            'max_sample_rotation_spread_deg',
        },
        'move_point_corrected': {
            'point', 'motion_type', 'speed_factor', 'speed', 'acc',
            'position_tolerance_mm', 'orientation_tolerance_deg',
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

    if kind == 'agv_navigate_pose':
        name = params['waypoint_name']
        if not isinstance(name, str):
            raise OperatorInputError('AGV用户航点名称必须是字符串')
        name = name.strip()
        if not name or len(name) > 64:
            raise OperatorInputError('AGV用户航点名称不能为空且不能超过64字符')
        if any(ord(character) < 32 for character in name):
            raise OperatorInputError('AGV用户航点名称不能包含控制字符')
        x = _finite_number(params['x'], '用户航点X')
        y = _finite_number(params['y'], '用户航点Y')
        yaw = _finite_number(params['yaw'], '用户航点Yaw')
        if not all(-10000.0 <= value <= 10000.0 for value in (x, y)):
            raise OperatorInputError('用户航点XY超出允许范围')
        max_speed = _finite_number(params['max_speed_mps'], 'AGV最大速度')
        timeout = _finite_number(params['timeout_s'], 'AGV到站超时')
        if not 0.01 <= max_speed <= 0.2:
            raise OperatorInputError('AGV最大速度必须在0.01~0.20 m/s之间')
        if not 10.0 <= timeout <= 3600.0:
            raise OperatorInputError('AGV到站超时必须在10~3600秒之间')
        travel_mode = params['travel_mode']
        if (
            not isinstance(travel_mode, str)
            or travel_mode not in TRAVEL_MODE_LABELS
        ):
            raise OperatorInputError('AGV行驶方式必须是自动、正走或倒走')
        return {
            'waypoint_name': name,
            'x': x,
            'y': y,
            'yaw': math.atan2(math.sin(yaw), math.cos(yaw)),
            'max_speed_mps': max_speed,
            'timeout_s': timeout,
            'travel_mode': travel_mode,
        }
    if kind == 'agv_navigate_station':
        station_id = params['station_id']
        if not isinstance(station_id, str):
            raise OperatorInputError('AGV航点ID必须是字符串')
        station_id = station_id.strip()
        if not station_id or len(station_id) > 128:
            raise OperatorInputError('AGV航点ID不能为空且不能超过128字符')
        if any(ord(character) < 32 for character in station_id):
            raise OperatorInputError('AGV航点ID不能包含控制字符')
        max_speed = _finite_number(params['max_speed_mps'], 'AGV最大速度')
        timeout = _finite_number(params['timeout_s'], 'AGV到站超时')
        if not 0.01 <= max_speed <= 0.2:
            raise OperatorInputError('AGV最大速度必须在0.01~0.20 m/s之间')
        if not 10.0 <= timeout <= 3600.0:
            raise OperatorInputError('AGV到站超时必须在10~3600秒之间')
        travel_mode = params['travel_mode']
        if (
            not isinstance(travel_mode, str)
            or travel_mode not in TRAVEL_MODE_LABELS
        ):
            raise OperatorInputError('AGV行驶方式必须是自动、正走或倒走')
        return {
            'station_id': station_id,
            'max_speed_mps': max_speed,
            'timeout_s': timeout,
            'travel_mode': travel_mode,
        }
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
    if kind == 'measure_apriltag_correction':
        reference = params['reference_capture']
        family = params['family']
        host = params['camera_host']
        if not isinstance(reference, str):
            raise OperatorInputError('参考采集名称必须是字符串')
        if family not in {'tag16h5', 'tag25h9', 'tag36h10', 'tag36h11'}:
            raise OperatorInputError('视觉纠偏必须指定唯一AprilTag族')
        if not isinstance(host, str) or not host.strip() or len(host) > 253:
            raise OperatorInputError('SC3000地址无效')
        if any(character.isspace() for character in host):
            raise OperatorInputError('SC3000地址不能包含空白字符')

        tag_size = _finite_number(params['tag_size_mm'], 'AprilTag边长')
        camera_timeout = _finite_number(params['camera_timeout_s'], '相机超时')
        reprojection = _finite_number(
            params['max_reprojection_rms_px'], '最大重投影误差'
        )
        max_translation = _finite_number(
            params['max_translation_mm'], '最大纠偏平移'
        )
        max_rotation = _finite_number(
            params['max_rotation_deg'], '最大纠偏旋转'
        )
        translation_spread = _finite_number(
            params['max_sample_translation_spread_mm'], '多帧平移离散限制'
        )
        rotation_spread = _finite_number(
            params['max_sample_rotation_spread_deg'], '多帧旋转离散限制'
        )
        for value, label, minimum, maximum in (
            (tag_size, 'AprilTag边长', 0.1, 1000.0),
            (camera_timeout, '相机超时', 3.0, 60.0),
            (reprojection, '最大重投影误差', 0.05, 10.0),
            (max_translation, '最大纠偏平移', 0.1, 500.0),
            (max_rotation, '最大纠偏旋转', 0.1, 45.0),
            (translation_spread, '多帧平移离散限制', 0.05, 20.0),
            (rotation_spread, '多帧旋转离散限制', 0.05, 10.0),
        ):
            if not minimum <= value <= maximum:
                raise OperatorInputError(
                    f'{label}必须在 {minimum:g}~{maximum:g} 之间'
                )
        return {
            'reference_capture': validate_point_name(reference),
            'family': family,
            'tag_id': _integer_in_range(
                params['tag_id'], 'AprilTag ID', 0, 999999
            ),
            'tag_size_mm': tag_size,
            'camera_host': host.strip(),
            'camera_timeout_s': camera_timeout,
            'samples': _integer_in_range(
                params['samples'], '采样帧数', 1, 10
            ),
            'max_reprojection_rms_px': reprojection,
            'max_translation_mm': max_translation,
            'max_rotation_deg': max_rotation,
            'max_sample_translation_spread_mm': translation_spread,
            'max_sample_rotation_spread_deg': rotation_spread,
        }
    if kind == 'move_point_corrected':
        point = params['point']
        motion_type = params['motion_type']
        if not isinstance(point, str):
            raise OperatorInputError('点位名称必须是字符串')
        if motion_type not in {'joint', 'linear'}:
            raise OperatorInputError('纠偏运动类型必须为joint或linear')
        position_tolerance = _finite_number(
            params['position_tolerance_mm'], '位置容差'
        )
        orientation_tolerance = _finite_number(
            params['orientation_tolerance_deg'], '姿态容差'
        )
        if not 0.1 <= position_tolerance <= 20.0:
            raise OperatorInputError('位置容差必须在0.1~20.0 mm之间')
        if not 0.1 <= orientation_tolerance <= 5.0:
            raise OperatorInputError('姿态容差必须在0.1~5.0°之间')
        return {
            'point': validate_point_name(point),
            'motion_type': motion_type,
            'speed_factor': _integer_in_range(
                params['speed_factor'], '全局速度', 1, 100
            ),
            'speed': _integer_in_range(params['speed'], '运动速度', 1, 100),
            'acc': _integer_in_range(params['acc'], '运动加速度', 1, 100),
            'position_tolerance_mm': position_tolerance,
            'orientation_tolerance_deg': orientation_tolerance,
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

    def __init__(self, ros_client, visual_correction_provider=None) -> None:
        self.ros = ros_client
        self.visual_correction_provider = visual_correction_provider
        self.visual_correction = None
        self._completion_text = '完成'

    def run(
        self,
        commands: Sequence[QueueCommand],
        points_dir: Path,
        cancel_event: threading.Event,
        progress: QueueProgress | None = None,
    ) -> QueueRunResult:
        steps = list(commands)
        completed = 0
        self.visual_correction = None

        def report(index: int, state: str, text: str) -> None:
            if progress is not None:
                progress(index, state, text)

        for index, command in enumerate(steps):
            if cancel_event.is_set():
                self._mark_remaining_skipped(steps, index, report)
                return QueueRunResult(completed, len(steps), True)
            report(index, 'running', f'正在执行：{command.description}')
            self._completion_text = '完成'
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
            report(index, 'done', self._completion_text)

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
        if command.kind == 'agv_navigate_pose':
            self.visual_correction = None
            completed = self.ros.agv_navigate_pose_and_wait(
                params['waypoint_name'],
                params['x'],
                params['y'],
                params['yaw'],
                params['max_speed_mps'],
                params['timeout_s'],
                cancel_event=cancel_event,
                progress=lambda text: report(index, 'running', text),
                travel_mode=params['travel_mode'],
            )
            if not completed:
                return False
            self._completion_text = (
                f'AGV已到达用户航点 {params["waypoint_name"]}'
            )
        elif command.kind == 'agv_navigate_station':
            # A base motion invalidates every tag-to-base measurement from the
            # previous station, even when the destination has the same name.
            self.visual_correction = None
            completed = self.ros.agv_navigate_and_wait(
                params['station_id'],
                params['max_speed_mps'],
                params['timeout_s'],
                cancel_event=cancel_event,
                progress=lambda text: report(index, 'running', text),
                travel_mode=params['travel_mode'],
            )
            if not completed:
                return False
            self._completion_text = f'AGV已到达航点 {params["station_id"]}'
        elif command.kind == 'move_point':
            point = load_point(points_dir, params['point'])
            self.ros.move_to_joints(
                point.joints,
                params['speed_factor'],
                params['speed_j'],
                params['acc_j'],
                tolerance_deg=params['tolerance_deg'],
                progress=lambda text: report(index, 'running', text),
            )
        elif command.kind == 'measure_apriltag_correction':
            if self.visual_correction_provider is None:
                raise OperatorInputError('任务队列未配置AprilTag纠偏测量器')
            self.visual_correction = self.visual_correction_provider(
                params,
                points_dir,
                cancel_event,
                lambda text: report(index, 'running', text),
            )
            self._completion_text = self.visual_correction.summary
        elif command.kind == 'move_point_corrected':
            if self.visual_correction is None:
                raise OperatorInputError(
                    '纠偏点位前没有成功执行“测量AprilTag纠偏”指令'
                )
            point = load_point(points_dir, params['point'])
            if point.pose is None:
                raise OperatorInputError(
                    f'点位 {point.name} 没有Tool0笛卡尔位姿，不能纠偏'
                )
            target_pose = self.visual_correction.corrected_pose(point.pose)
            actual = self.ros.move_to_pose(
                target_pose,
                params['motion_type'],
                params['speed_factor'],
                params['speed'],
                params['acc'],
                params['position_tolerance_mm'],
                params['orientation_tolerance_deg'],
                progress=lambda text: report(index, 'running', text),
            )
            self._completion_text = (
                f'已到纠偏点位 {point.name}；实际Tool0='
                f'[{", ".join(f"{value:.3f}" for value in actual)}]'
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
