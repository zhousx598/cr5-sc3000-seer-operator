import json
import math
import threading
import time

from std_msgs.msg import String

from dobot_operator_gui.ros_client import DobotRosClient
from dobot_operator_gui.main_window import DobotOperatorWindow


class Harness:
    pass


def test_agv_status_and_station_callbacks_keep_ros_data_only():
    client = Harness()
    client._agv_state_lock = threading.Lock()
    client._agv_status = {}
    client._agv_status_time = 0.0
    client._agv_stations = []

    status = String()
    status.data = json.dumps(
        {"connected": True, "safe_for_teleop": False, "nav_status": 0}
    )
    DobotRosClient._agv_status_callback(client, status)
    stations = String()
    stations.data = json.dumps(
        [{"id": "LM1"}, {"id": "LM2"}, {"missing_id": True}]
    )
    DobotRosClient._agv_stations_callback(client, stations)

    current = DobotRosClient.agv_status(client)
    assert current["connected"] is True
    assert current["safe_for_teleop"] is False
    assert [item["id"] for item in DobotRosClient.agv_stations(client)] == [
        "LM1",
        "LM2",
    ]


def test_agv_status_is_fail_closed_when_stale():
    client = Harness()
    client._agv_state_lock = threading.Lock()
    client._agv_status = {"connected": True, "safe_for_teleop": True}
    client._agv_status_time = time.monotonic() - 10.0
    status = DobotRosClient.agv_status(client, maximum_age=2.0)
    assert status["connected"] is False
    assert "过期" in status["safety_reason"]


def test_agv_velocity_bridge_never_publishes_lateral_speed():
    published = []
    client = Harness()
    client._agv_cmd_pub = type(
        "Publisher",
        (),
        {"publish": lambda unused_self, msg: published.append(msg)},
    )()
    DobotRosClient.agv_publish_velocity(client, 0.02, -0.1)
    assert len(published) == 1
    assert published[0].linear.x == 0.02
    assert published[0].linear.y == 0.0
    assert published[0].angular.z == -0.1


def test_agv_localization_confirmation_carries_operator_pose_snapshot():
    captured = {}
    client = Harness()

    def fake_call(name, request, timeout):
        captured.update(name=name, request=request, timeout=timeout)
        return type("Response", (), {"message": "confirmed"})()

    client._call_agv = fake_call
    result = DobotRosClient.agv_confirm_localization(
        client, 1.25, -0.75, 0.5
    )

    assert result == "confirmed"
    assert captured["name"] == "confirm_localization"
    assert captured["timeout"] == 8.0
    assert captured["request"].operator_confirmed is True
    assert captured["request"].expected_x == 1.25
    assert captured["request"].expected_y == -0.75
    assert captured["request"].expected_yaw == 0.5


def test_agv_map_callback_is_versioned_and_copied():
    client = Harness()
    client._agv_state_lock = threading.Lock()
    client._agv_map_data = {}
    client._agv_map_generation = 0
    message = String()
    message.data = json.dumps({"current_map": "map_a", "maps": ["map_a"]})

    DobotRosClient._agv_map_data_callback(client, message)
    generation, data = DobotRosClient.agv_map_data(client)
    same_generation, unchanged = DobotRosClient.agv_map_data(
        client, generation
    )

    assert generation == 1
    assert same_generation == 1
    assert data == {"current_map": "map_a", "maps": ["map_a"]}
    assert unchanged is None


def test_agv_map_and_relocalization_requests_are_typed():
    captured = []
    client = Harness()

    def fake_call(name, request, timeout):
        captured.append((name, request, timeout))
        return type("Response", (), {"message": "ok"})()

    client._call_agv = fake_call
    assert DobotRosClient.agv_load_map(client, "map_a") == "ok"
    assert DobotRosClient.agv_relocalize(client, 1.0, 2.0, 0.3, 0.5) == "ok"
    assert DobotRosClient.agv_cancel_relocalization(client) == "ok"

    assert captured[0][0] == "load_map"
    assert captured[0][1].operator_confirmed is True
    assert captured[0][1].map_name == "map_a"
    assert captured[1][0] == "relocalize"
    assert captured[1][1].operator_confirmed is True
    assert captured[1][1].x == 1.0
    assert captured[1][1].y == 2.0
    assert captured[1][1].yaw == 0.3
    assert captured[1][1].radius == 0.5
    assert captured[2][0] == "cancel_relocalization"


def test_agv_navigation_waits_for_new_running_and_complete_status():
    client = Harness()
    statuses = iter(
        [
            {
                "connected": True,
                "safe_to_start_navigation": True,
                "gui_status_generation": 10,
            },
            {
                "connected": True,
                "localized": True,
                "map_loaded": True,
                "nav_status": 2,
                "gui_status_generation": 11,
            },
            {
                "connected": True,
                "localized": True,
                "map_loaded": True,
                "nav_status": 4,
                "pose": {"current_station": "LM1"},
                "gui_status_generation": 12,
            },
        ]
    )
    client.agv_status = lambda maximum_age=2.0: next(statuses)
    client.agv_navigate_to_station = lambda station, speed: ("task-1", "ok")
    client.agv_cancel_navigation = lambda: "cancelled"
    progress = []

    completed = DobotRosClient.agv_navigate_and_wait(
        client,
        "LM1",
        0.08,
        30.0,
        progress=progress.append,
    )

    assert completed is True
    assert any("运行中" in text for text in progress)
    assert any("完成" in text for text in progress)


def test_agv_navigation_honors_cancel_before_sending():
    client = Harness()
    sent = []
    cancel = threading.Event()
    cancel.set()
    client.agv_navigate_to_station = lambda station, speed: sent.append(
        (station, speed)
    )

    completed = DobotRosClient.agv_navigate_and_wait(
        client, "LM1", 0.08, 30.0, cancel_event=cancel
    )

    assert completed is False
    assert sent == []


def test_station_navigation_request_includes_catalog_heading():
    captured = {}
    client = Harness()
    client.agv_stations = lambda: [
        {"id": "LM1", "x": 1.0, "y": 2.0, "r": 0.75}
    ]

    def fake_call(name, request, timeout):
        captured.update(name=name, request=request, timeout=timeout)
        return type("Response", (), {"task_id": "t1", "message": "ok"})()

    client._call_agv = fake_call
    result = DobotRosClient.agv_navigate_to_station(client, "LM1", 0.05)

    assert result == ("t1", "ok")
    assert captured["name"] == "navigate"
    assert captured["request"].use_target_yaw is True
    assert captured["request"].target_yaw == 0.75


def test_pose_navigation_and_plan_requests_are_typed():
    captured = []
    client = Harness()

    def fake_call(name, request, timeout):
        captured.append((name, request, timeout))
        if name == "navigate_pose":
            return type("Response", (), {"task_id": "t2", "message": "ok"})()
        return type("Response", (), {"station_ids": ["LM1", "LM2"]})()

    client._call_agv = fake_call
    assert DobotRosClient.agv_navigate_to_pose(
        client, "P1", 1.0, 2.0, 0.5, 0.05
    ) == ("t2", "ok")
    assert DobotRosClient.agv_plan_to_station(client, "LM2") == (
        "LM1",
        "LM2",
    )
    pose_request = captured[0][1]
    assert (pose_request.waypoint_name, pose_request.x, pose_request.y) == (
        "P1",
        1.0,
        2.0,
    )
    assert pose_request.yaw == 0.5
    assert pose_request.max_speed == 0.05
    assert captured[1][1].target_station_id == "LM2"


def test_pose_completion_check_handles_wrapped_heading_and_tolerance():
    expected = (1.0, 2.0, math.pi - 0.02)
    good_status = {
        "pose": {"x": 1.05, "y": 2.02, "angle": -math.pi + 0.02}
    }
    bad_status = {
        "pose": {"x": 1.2, "y": 2.0, "angle": math.pi - 0.02}
    }

    assert DobotRosClient._pose_completion_error(good_status, expected) is None
    assert "超出校验范围" in DobotRosClient._pose_completion_error(
        bad_status, expected
    )


class Value:
    def __init__(self, value):
        self._value = value

    def value(self):
        return self._value


class Toggle:
    def isChecked(self):
        return True


class DownButton:
    def __init__(self, down=True):
        self.down = down

    def isDown(self):
        return self.down


class TextSink:
    def setText(self, text):
        self.text = text


class RosHarness:
    def __init__(self):
        self.published = []

    def agv_status(self):
        return {"safe_for_teleop": True}

    def agv_publish_velocity(self, vx, angular_z):
        self.published.append((vx, angular_z))


def test_forward_button_publishes_positive_vx_until_its_own_release():
    window = Harness()
    window.ros = RosHarness()
    window.agv_teleop_enable = Toggle()
    window.agv_forward_speed_spin = Value(0.03)
    window.agv_backward_speed_spin = Value(0.03)
    window.agv_turn_speed_spin = Value(0.1)
    window.agv_teleop_feedback_label = TextSink()
    window._agv_teleop_command = (0.0, 0.0)
    window._agv_teleop_active = False
    window._agv_pressed_button = None
    window._closing = False
    window._publish_agv_teleop = (
        lambda: DobotOperatorWindow._publish_agv_teleop(window)
    )
    window._stop_agv_teleop = lambda: DobotOperatorWindow._stop_agv_teleop(
        window
    )
    button = DownButton()

    DobotOperatorWindow._start_agv_teleop(window, "forward", button)
    assert window.ros.published[-1] == (0.03, 0.0)

    button.down = False
    DobotOperatorWindow._publish_agv_teleop(window)
    assert window.ros.published[-1] == (0.0, 0.0)
