import json
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
        "Publisher", (), {"publish": lambda unused_self, msg: published.append(msg)}
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
    same_generation, unchanged = DobotRosClient.agv_map_data(client, generation)

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
    window._publish_agv_teleop = lambda: DobotOperatorWindow._publish_agv_teleop(
        window
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
