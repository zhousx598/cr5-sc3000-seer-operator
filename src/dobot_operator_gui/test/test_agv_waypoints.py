import json
import math
from pathlib import Path

import pytest

from dobot_operator_gui.agv_waypoints import AgvWaypoint
from dobot_operator_gui.agv_waypoints import AgvWaypointError
from dobot_operator_gui.agv_waypoints import delete_waypoint
from dobot_operator_gui.agv_waypoints import load_waypoints
from dobot_operator_gui.agv_waypoints import upsert_waypoint


def test_waypoints_are_saved_per_map_and_round_trip_unicode(tmp_path: Path):
    path = tmp_path / 'agv_waypoints.json'

    upsert_waypoint(path, 'map_a', AgvWaypoint('按钮前', 1.2, -0.4, 0.25))
    upsert_waypoint(path, 'map_b', AgvWaypoint('充电区', -2.0, 3.5, -0.5))

    assert load_waypoints(path, 'map_a') == [
        AgvWaypoint('按钮前', 1.2, -0.4, 0.25)
    ]
    assert load_waypoints(path, 'map_b') == [
        AgvWaypoint('充电区', -2.0, 3.5, -0.5)
    ]
    document = json.loads(path.read_text(encoding='utf-8'))
    assert document['schema_version'] == 1
    assert set(document['maps']) == {'map_a', 'map_b'}


def test_upsert_updates_only_same_name_and_delete_is_explicit(tmp_path: Path):
    path = tmp_path / 'agv_waypoints.json'
    upsert_waypoint(path, 'map_a', AgvWaypoint('P1', 1, 2, 0))
    upsert_waypoint(path, 'map_a', AgvWaypoint('P2', 3, 4, 0.5))
    upsert_waypoint(path, 'map_a', AgvWaypoint('P1', 5, 6, 1.0))

    assert load_waypoints(path, 'map_a') == [
        AgvWaypoint('P1', 5, 6, 1.0),
        AgvWaypoint('P2', 3, 4, 0.5),
    ]
    assert delete_waypoint(path, 'map_a', 'P1') == [
        AgvWaypoint('P2', 3, 4, 0.5)
    ]
    with pytest.raises(AgvWaypointError, match='不存在'):
        delete_waypoint(path, 'map_a', 'P1')


def test_waypoint_normalizes_heading_and_rejects_bad_data(tmp_path: Path):
    waypoint = AgvWaypoint('P1', 1, 2, 3 * math.pi)
    assert waypoint.yaw == pytest.approx(math.pi)

    with pytest.raises(AgvWaypointError):
        AgvWaypoint('', 1, 2, 0)
    with pytest.raises(AgvWaypointError):
        AgvWaypoint('P1', float('nan'), 2, 0)

    path = tmp_path / 'bad.json'
    path.write_text(
        '{"schema_version":1,"maps":{"map_a":[{"name":"P1"}]}}',
        encoding='utf-8',
    )
    with pytest.raises(AgvWaypointError, match='字段'):
        load_waypoints(path, 'map_a')
