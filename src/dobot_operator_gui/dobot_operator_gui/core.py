"""Pure helpers shared by the GUI and its tests."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import shutil
import socket
import tempfile
from typing import Iterable, Sequence
import uuid


JOINT_COUNT = 6
POSE_COUNT = 6
POINT_NAME_PATTERN = re.compile(r'^[A-Za-z0-9_.-]+$')
BRACED_VALUES_PATTERN = re.compile(r'\{([^{}]+)\}')
NUMBER_PATTERN = re.compile(
    r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
)

ROBOT_MODE_DESCRIPTIONS = {
    '1': '初始化',
    '2': '抱闸松开',
    '3': '未上电',
    '4': '未使能',
    '5': '已使能且空闲',
    '6': '拖动模式',
    '7': '运行中',
    '8': '单次运动/Jog',
    '9': '报警',
    '10': '暂停',
    '11': '碰撞',
}


class OperatorInputError(ValueError):
    """Raised when local user input or a saved point is invalid."""


@dataclass(frozen=True)
class SavedPoint:
    """A named joint point with an optional Cartesian pose."""

    name: str
    joints: tuple[float, ...]
    pose: tuple[float, ...] | None = None


@dataclass(frozen=True)
class SavedCapture:
    """A point and its camera image stored in one capture directory."""

    point: SavedPoint
    directory: Path
    image_path: Path
    metadata_path: Path


def validate_point_name(name: str) -> str:
    """Validate a filename-safe point name and return its stripped value."""
    value = name.strip()
    if not value or not POINT_NAME_PATTERN.fullmatch(value):
        raise OperatorInputError(
            '点名只能包含字母、数字、点、下划线和连字符'
        )
    return value


def parse_numeric_values(text: str, expected_count: int) -> tuple[float, ...]:
    """Parse finite numeric values, preferring a braced protocol payload."""
    candidates = BRACED_VALUES_PATTERN.findall(text)
    if not candidates:
        candidates = [text]

    for candidate in candidates:
        fields = [field.strip() for field in candidate.split(',')]
        if len(fields) != expected_count:
            continue
        try:
            values = tuple(float(field) for field in fields)
        except ValueError:
            continue
        if all(math.isfinite(value) for value in values):
            return values

    raise OperatorInputError(
        f'未找到包含 {expected_count} 个有限数值的数据：{text!r}'
    )


def parse_integer_values(text: str, minimum_count: int = 1) -> tuple[int, ...]:
    """Parse a Modbus integer response such as ``0,1,1000`` or ``{...}``."""
    match = BRACED_VALUES_PATTERN.search(text)
    payload = match.group(1) if match else text
    values = tuple(int(token) for token in NUMBER_PATTERN.findall(payload))
    if len(values) < minimum_count:
        raise OperatorInputError(f'无法解析 Modbus 返回值：{text!r}')
    return values


def format_values(values: Iterable[float]) -> str:
    return '{' + ','.join(f'{value:.6f}' for value in values) + '}'


def format_joints(values: Iterable[float]) -> str:
    return ', '.join(f'{value:.3f}°' for value in values)


def mode_text(mode: str) -> str:
    return f'{mode}（{ROBOT_MODE_DESCRIPTIONS.get(mode, "未知状态")}）'


def angular_error_deg(actual: float, target: float) -> float:
    return (actual - target + 180.0) % 360.0 - 180.0


def max_joint_error_deg(
    actual: Sequence[float], target: Sequence[float]
) -> float:
    if len(actual) != JOINT_COUNT or len(target) != JOINT_COUNT:
        raise OperatorInputError('关节角必须包含 6 个数值')
    return max(
        abs(angular_error_deg(actual_value, target_value))
        for actual_value, target_value in zip(actual, target)
    )


def _atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        dir=path.parent,
        prefix=f'.{path.name}.',
        delete=False,
    ) as temporary:
        temporary.write(text)
        temporary_path = Path(temporary.name)
    temporary_path.replace(path)


def save_point(
    points_dir: Path,
    name: str,
    joints: Sequence[float],
    pose: Sequence[float],
) -> SavedPoint:
    """Save a point in the legacy files already used in this workspace."""
    point_name = validate_point_name(name)
    joint_values = tuple(float(value) for value in joints)
    pose_values = tuple(float(value) for value in pose)
    if len(joint_values) != JOINT_COUNT or not all(
        math.isfinite(value) for value in joint_values
    ):
        raise OperatorInputError('关节角必须是 6 个有限数值')
    if len(pose_values) != POSE_COUNT or not all(
        math.isfinite(value) for value in pose_values
    ):
        raise OperatorInputError('末端位姿必须是 6 个有限数值')

    joint_text = (
        'response:\n'
        'dobot_msgs_v3.srv.GetAngle_Response('
        f"res=0, angle='{format_values(joint_values)}')\n"
    )
    pose_text = (
        'response:\n'
        'dobot_msgs_v3.srv.GetPose_Response('
        f"res=0, pose='{format_values(pose_values)}')\n"
    )
    _atomic_write_text(points_dir / f'{point_name}_joint.txt', joint_text)
    _atomic_write_text(points_dir / f'{point_name}_pose.txt', pose_text)
    return SavedPoint(point_name, joint_values, pose_values)


def save_capture_group(
    points_dir: Path,
    name: str,
    joints: Sequence[float],
    pose: Sequence[float],
    source_image: Path,
    metadata: dict | None = None,
    *,
    overwrite: bool = False,
) -> SavedCapture:
    """Atomically store coordinates, image and metadata in one directory."""
    point_name = validate_point_name(name)
    root = Path(points_dir).expanduser().resolve()
    image_source = Path(source_image).expanduser().resolve()
    if not image_source.is_file():
        raise OperatorInputError(f'相机图像不存在：{image_source}')
    suffix = image_source.suffix.lower()
    if suffix not in {'.jpg', '.jpeg', '.png', '.bmp'}:
        raise OperatorInputError(f'不支持的图像格式：{suffix or "无扩展名"}')

    root.mkdir(parents=True, exist_ok=True)
    target = root / point_name
    if target.exists() and not overwrite:
        raise OperatorInputError(f'采集组已存在：{target}')
    if target.exists() and not target.is_dir():
        raise OperatorInputError(f'采集组路径不是目录：{target}')

    staging = Path(
        tempfile.mkdtemp(prefix=f'.{point_name}.staging-', dir=root)
    )
    backup = root / f'.{point_name}.backup-{uuid.uuid4().hex}'
    try:
        point = save_point(staging, point_name, joints, pose)
        image_name = f'{point_name}_image{suffix}'
        staged_image = staging / image_name
        shutil.copy2(image_source, staged_image)

        capture_metadata = dict(metadata or {})
        capture_metadata.update(
            {
                'format_version': 1,
                'point_name': point_name,
                'joint_angles_deg': list(point.joints),
                'tool_pose': list(point.pose or ()),
                'units': {
                    'joint_angles': 'degree',
                    'tool_position': 'millimeter',
                    'tool_rotation': 'degree',
                },
                'image_file': image_name,
            }
        )
        metadata_name = f'{point_name}_capture.json'
        _atomic_write_text(
            staging / metadata_name,
            json.dumps(capture_metadata, ensure_ascii=False, indent=2) + '\n',
        )

        if target.exists():
            target.rename(backup)
        try:
            staging.rename(target)
        except Exception:
            if backup.exists() and not target.exists():
                backup.rename(target)
            raise
        if backup.exists():
            shutil.rmtree(backup)
    finally:
        if staging.exists():
            shutil.rmtree(staging)

    return SavedCapture(
        point=point,
        directory=target,
        image_path=target / image_name,
        metadata_path=target / metadata_name,
    )


def load_point(points_dir: Path, name: str) -> SavedPoint:
    point_name = validate_point_name(name)
    root = Path(points_dir)
    grouped_dir = root / point_name
    if grouped_dir.is_dir():
        joint_path = grouped_dir / f'{point_name}_joint.txt'
        pose_path = grouped_dir / f'{point_name}_pose.txt'
    else:
        joint_path = root / f'{point_name}_joint.txt'
        pose_path = root / f'{point_name}_pose.txt'
    try:
        joints = parse_numeric_values(
            joint_path.read_text(encoding='utf-8'), JOINT_COUNT
        )
    except FileNotFoundError as exc:
        raise OperatorInputError(f'点位文件不存在：{joint_path}') from exc
    except OSError as exc:
        raise OperatorInputError(f'无法读取点位：{exc}') from exc

    pose = None
    if pose_path.exists():
        try:
            pose = parse_numeric_values(
                pose_path.read_text(encoding='utf-8'), POSE_COUNT
            )
        except (OSError, OperatorInputError) as exc:
            raise OperatorInputError(f'无法读取点位姿态：{exc}') from exc
    return SavedPoint(point_name, joints, pose)


def list_point_names(points_dir: Path) -> list[str]:
    if not points_dir.exists():
        return []
    legacy_names = {
        path.name[:-len('_joint.txt')]
        for path in points_dir.glob('*_joint.txt')
        if POINT_NAME_PATTERN.fullmatch(path.name[:-len('_joint.txt')])
    }
    grouped_names = {
        path.name
        for path in points_dir.iterdir()
        if path.is_dir()
        and POINT_NAME_PATTERN.fullmatch(path.name)
        and (path / f'{path.name}_joint.txt').is_file()
    }
    return sorted(legacy_names | grouped_names)


def point_image_path(points_dir: Path, name: str) -> Path | None:
    """Return the image belonging to a grouped point, if present."""
    point_name = validate_point_name(name)
    group = Path(points_dir) / point_name
    if not group.is_dir():
        return None
    for suffix in ('.jpg', '.jpeg', '.png', '.bmp'):
        candidate = group / f'{point_name}_image{suffix}'
        if candidate.is_file():
            return candidate
    return None


def check_tcp_ports(
    host: str,
    ports: Sequence[int] = (29999, 30003, 30005),
    timeout: float = 1.0,
) -> dict[int, str]:
    """Check Dobot TCP ports without treating an ICMP result as authoritative."""
    results: dict[int, str] = {}
    for port in ports:
        try:
            with socket.create_connection((host, port), timeout=timeout):
                results[port] = '可连接'
        except OSError as exc:
            results[port] = f'失败：{exc}'
    return results
