"""Map-scoped operator waypoints stored outside the Robokit map file."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import tempfile
from typing import Sequence


SCHEMA_VERSION = 1


class AgvWaypointError(RuntimeError):
    """Raised when local waypoint data is missing or malformed."""


def _clean_text(value: object, label: str, maximum: int) -> str:
    if not isinstance(value, str):
        raise AgvWaypointError(f'{label}必须是字符串')
    result = value.strip()
    if not result or len(result) > maximum:
        raise AgvWaypointError(f'{label}不能为空且不能超过{maximum}字符')
    if any(ord(character) < 32 for character in result):
        raise AgvWaypointError(f'{label}不能包含控制字符')
    return result


def normalize_yaw(yaw: float) -> float:
    value = float(yaw)
    if not math.isfinite(value):
        raise AgvWaypointError('航点朝向必须是有限数值')
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class AgvWaypoint:
    """One world-frame pose associated with one loaded map name."""

    name: str
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        name = _clean_text(self.name, '航点名称', 64)
        try:
            x = float(self.x)
            y = float(self.y)
        except (TypeError, ValueError) as exc:
            raise AgvWaypointError('航点XY必须是数值') from exc
        if not all(math.isfinite(value) for value in (x, y)):
            raise AgvWaypointError('航点XY必须是有限数值')
        if not all(-10000.0 <= value <= 10000.0 for value in (x, y)):
            raise AgvWaypointError('航点XY超出允许范围')
        object.__setattr__(self, 'name', name)
        object.__setattr__(self, 'x', x)
        object.__setattr__(self, 'y', y)
        object.__setattr__(self, 'yaw', normalize_yaw(self.yaw))

    @classmethod
    def from_dict(cls, value: object) -> 'AgvWaypoint':
        if not isinstance(value, dict):
            raise AgvWaypointError('航点记录必须是JSON对象')
        if set(value) != {'name', 'x', 'y', 'yaw'}:
            raise AgvWaypointError('航点字段必须为name、x、y、yaw')
        return cls(value['name'], value['x'], value['y'], value['yaw'])

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def _empty_document() -> dict[str, object]:
    return {'schema_version': SCHEMA_VERSION, 'maps': {}}


def _read_document(path: Path) -> dict[str, object]:
    source = Path(path).expanduser()
    if not source.exists():
        return _empty_document()
    try:
        document = json.loads(source.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise AgvWaypointError(f'无法读取用户航点文件：{exc}') from exc
    if not isinstance(document, dict):
        raise AgvWaypointError('用户航点文件根节点必须是JSON对象')
    if document.get('schema_version') != SCHEMA_VERSION:
        raise AgvWaypointError('不支持的用户航点文件版本')
    maps = document.get('maps')
    if not isinstance(maps, dict):
        raise AgvWaypointError('用户航点文件缺少maps对象')
    return document


def load_waypoints(path: Path, map_name: str) -> list[AgvWaypoint]:
    key = _clean_text(map_name, '地图名称', 255)
    document = _read_document(path)
    values = document['maps'].get(key, [])
    if not isinstance(values, list):
        raise AgvWaypointError(f'地图 {key} 的航点列表格式错误')
    waypoints = [AgvWaypoint.from_dict(value) for value in values]
    names = [waypoint.name for waypoint in waypoints]
    if len(names) != len(set(names)):
        raise AgvWaypointError(f'地图 {key} 中存在重名用户航点')
    return waypoints


def save_waypoints(
    path: Path, map_name: str, waypoints: Sequence[AgvWaypoint]
) -> None:
    destination = Path(path).expanduser()
    key = _clean_text(map_name, '地图名称', 255)
    normalized = [
        waypoint
        if isinstance(waypoint, AgvWaypoint)
        else AgvWaypoint.from_dict(waypoint)
        for waypoint in waypoints
    ]
    names = [waypoint.name for waypoint in normalized]
    if len(names) != len(set(names)):
        raise AgvWaypointError('同一地图不能保存重名用户航点')
    document = _read_document(destination)
    document['maps'][key] = [waypoint.to_dict() for waypoint in normalized]
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            dir=destination.parent,
            prefix=f'.{destination.name}.',
            suffix='.tmp',
            delete=False,
        ) as temporary:
            json.dump(document, temporary, ensure_ascii=False, indent=2)
            temporary.write('\n')
            temporary.flush()
            temporary_path = Path(temporary.name)
        temporary_path.replace(destination)
    except OSError as exc:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
        raise AgvWaypointError(f'无法保存用户航点：{exc}') from exc


def upsert_waypoint(
    path: Path, map_name: str, waypoint: AgvWaypoint
) -> list[AgvWaypoint]:
    values = load_waypoints(path, map_name)
    updated = False
    result = []
    for current in values:
        if current.name == waypoint.name:
            result.append(waypoint)
            updated = True
        else:
            result.append(current)
    if not updated:
        result.append(waypoint)
    save_waypoints(path, map_name, result)
    return result


def delete_waypoint(
    path: Path, map_name: str, waypoint_name: str
) -> list[AgvWaypoint]:
    name = _clean_text(waypoint_name, '航点名称', 64)
    values = load_waypoints(path, map_name)
    result = [waypoint for waypoint in values if waypoint.name != name]
    if len(result) == len(values):
        raise AgvWaypointError(f'用户航点不存在：{name}')
    save_waypoints(path, map_name, result)
    return result
