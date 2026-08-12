from seer_agv_driver.seer_agv_node import map_to_gui_data
from seer_agv_driver.seer_client import Footprint


def test_map_to_gui_data_preserves_geometry_catalog_and_footprint():
    map_data = {
        "normalPosList": [{"x": 1, "y": 2}, {"bad": True}],
        "advancedLineList": [
            {
                "line": {
                    "startPos": {"x": 0, "y": 0},
                    "endPos": {"x": 3, "y": 4},
                }
            }
        ],
        "advancedPointList": [
            {"instanceName": "P", "pos": {"x": -1, "y": 0.5}}
        ],
    }
    stations = {
        "stations": [{"id": "LM1", "x": 1.5, "y": -2.0, "r": 0.25}]
    }

    result = map_to_gui_data(
        map_data,
        stations,
        "map_a",
        ["map_a", "map_b"],
        Footprint(width=0.7, head=0.52, tail=0.48),
    )

    assert result["current_map"] == "map_a"
    assert result["maps"] == ["map_a", "map_b"]
    assert result["normal_points"] == [[1.0, 2.0]]
    assert result["feature_lines"] == [[[0.0, 0.0], [3.0, 4.0]]]
    assert result["advanced_points"] == [{"name": "P", "x": -1.0, "y": 0.5}]
    assert result["stations"][0] == {
        "id": "LM1",
        "x": 1.5,
        "y": -2.0,
        "yaw": 0.25,
    }
    assert result["footprint"] == {"width": 0.7, "head": 0.52, "tail": 0.48}
