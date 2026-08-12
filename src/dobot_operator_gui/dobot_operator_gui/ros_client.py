"""ROS 2 client used by the Dobot operator GUI."""

import copy
import json
import math
import re
import threading
import time
from typing import Callable, Sequence

from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dobot_msgs_v3.msg import ToolVectorActual
from dobot_msgs_v3.srv import ClearError
from dobot_msgs_v3.srv import Continues
from dobot_msgs_v3.srv import DisableRobot
from dobot_msgs_v3.srv import EmergencyStop
from dobot_msgs_v3.srv import EnableRobot
from dobot_msgs_v3.srv import GetAngle
from dobot_msgs_v3.srv import GetErrorID
from dobot_msgs_v3.srv import GetHoldRegs
from dobot_msgs_v3.srv import GetPose
from dobot_msgs_v3.srv import JointMovJ
from dobot_msgs_v3.srv import ModbusClose
from dobot_msgs_v3.srv import ModbusCreate
from dobot_msgs_v3.srv import MovJ
from dobot_msgs_v3.srv import MovL
from dobot_msgs_v3.srv import RobotMode
from dobot_msgs_v3.srv import SetCollisionLevel
from dobot_msgs_v3.srv import SetHoldRegs
from dobot_msgs_v3.srv import SetSafeSkin
from dobot_msgs_v3.srv import SpeedFactor
from dobot_msgs_v3.srv import StartDrag
from dobot_msgs_v3.srv import StopDrag
from seer_agv_msgs.srv import ConfirmLocalization, LoadMap, NavigateToPose
from seer_agv_msgs.srv import NavigateToStation, PlanToStation
from seer_agv_msgs.srv import Relocalize

from .core import format_joints
from .core import max_joint_error_deg
from .core import mode_text
from .core import parse_integer_values
from .core import parse_numeric_values
from .handeye_transform import dobot_pose_to_matrix
from .visual_correction import rotation_angle_degrees


SERVICE_PREFIX = '/dobot_bringup_v3/srv'
MOTION_START_DELTA_DEG = 0.05
TARGET_STABLE_SAMPLES = 2
ROBOT_FEEDBACK_MAX_AGE = 2.0
ROBOT_JOINT_NAMES = tuple(f'joint{index}' for index in range(1, 7))


class DobotServiceError(RuntimeError):
    """Raised when a ROS service is missing, times out, or rejects a command."""


class AgvServiceError(RuntimeError):
    """Raised when the ROS-owned SEER AGV driver rejects a GUI request."""


class DobotRosClient(Node):
    """Typed, synchronous facade over asynchronous Dobot ROS 2 clients.

    A ROS executor must be spinning this node in another thread. Calls are
    serialized because the upstream bringup node shares one controller socket.
    """

    def __init__(self, service_timeout: float = 5.0) -> None:
        super().__init__('dobot_operator_gui')
        self.service_timeout = service_timeout
        self._callback_group = ReentrantCallbackGroup()
        self._call_lock = threading.Lock()
        self._agv_call_lock = threading.Lock()
        self._agv_state_lock = threading.Lock()
        self._robot_feedback_lock = threading.Lock()
        self.motion_cancel = threading.Event()
        self.gripper_index: int | None = None
        self._service_clients = {}

        service_types = {
            'RobotMode': RobotMode,
            'GetAngle': GetAngle,
            'GetPose': GetPose,
            'GetErrorID': GetErrorID,
            'ClearError': ClearError,
            'Continue': Continues,
            'EnableRobot': EnableRobot,
            'DisableRobot': DisableRobot,
            'SpeedFactor': SpeedFactor,
            'SetCollisionLevel': SetCollisionLevel,
            'SetSafeSkin': SetSafeSkin,
            'StartDrag': StartDrag,
            'StopDrag': StopDrag,
            'JointMovJ': JointMovJ,
            'MovJ': MovJ,
            'MovL': MovL,
            'EmergencyStop': EmergencyStop,
            'ModbusCreate': ModbusCreate,
            'ModbusClose': ModbusClose,
            'SetHoldRegs': SetHoldRegs,
            'GetHoldRegs': GetHoldRegs,
        }
        for name, service_type in service_types.items():
            self._service_clients[name] = self.create_client(
                service_type,
                f'{SERVICE_PREFIX}/{name}',
                callback_group=self._callback_group,
            )

        self._feedback_angles: tuple[float, ...] | None = None
        self._feedback_angles_time = 0.0
        self._feedback_pose: tuple[float, ...] | None = None
        self._feedback_pose_time = 0.0
        self.create_subscription(
            JointState,
            '/joint_states_robot',
            self._joint_feedback_callback,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            ToolVectorActual,
            '/dobot_msgs_v3/msg/ToolVectorActual',
            self._tool_feedback_callback,
            10,
            callback_group=self._callback_group,
        )

        self._agv_status: dict[str, object] = {}
        self._agv_status_time = 0.0
        self._agv_status_generation = 0
        self._agv_stations: list[dict[str, object]] = []
        self._agv_map_data: dict[str, object] = {}
        self._agv_map_generation = 0
        self._agv_cmd_pub = self.create_publisher(
            Twist, '/seer_agv/cmd_vel', 10
        )
        self.create_subscription(
            String,
            '/seer_agv/status',
            self._agv_status_callback,
            10,
            callback_group=self._callback_group,
        )
        station_qos = QoSProfile(depth=1)
        station_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            '/seer_agv/stations',
            self._agv_stations_callback,
            station_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            '/seer_agv/map_data',
            self._agv_map_data_callback,
            station_qos,
            callback_group=self._callback_group,
        )
        self._agv_clients = {
            'stop': self.create_client(
                Trigger,
                '/seer_agv/stop',
                callback_group=self._callback_group,
            ),
            'cancel_nav': self.create_client(
                Trigger,
                '/seer_agv/cancel_nav',
                callback_group=self._callback_group,
            ),
            'confirm_localization': self.create_client(
                ConfirmLocalization,
                '/seer_agv/confirm_localization',
                callback_group=self._callback_group,
            ),
            'load_map': self.create_client(
                LoadMap,
                '/seer_agv/load_map',
                callback_group=self._callback_group,
            ),
            'relocalize': self.create_client(
                Relocalize,
                '/seer_agv/relocalize',
                callback_group=self._callback_group,
            ),
            'cancel_relocalization': self.create_client(
                Trigger,
                '/seer_agv/cancel_relocalization',
                callback_group=self._callback_group,
            ),
            'download_map': self.create_client(
                Trigger,
                '/seer_agv/download_map',
                callback_group=self._callback_group,
            ),
            'navigate': self.create_client(
                NavigateToStation,
                '/seer_agv/navigate_to_station',
                callback_group=self._callback_group,
            ),
            'navigate_pose': self.create_client(
                NavigateToPose,
                '/seer_agv/navigate_to_pose',
                callback_group=self._callback_group,
            ),
            'plan_to_station': self.create_client(
                PlanToStation,
                '/seer_agv/plan_to_station',
                callback_group=self._callback_group,
            ),
        }

    def driver_ready(self) -> bool:
        return self._service_clients['RobotMode'].service_is_ready()

    def available_services(self) -> tuple[int, int]:
        ready = sum(
            client.service_is_ready()
            for client in self._service_clients.values()
        )
        return ready, len(self._service_clients)

    def _joint_feedback_callback(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in ROBOT_JOINT_NAMES):
            return
        angles = tuple(
            math.degrees(float(positions[name])) for name in ROBOT_JOINT_NAMES
        )
        if not all(math.isfinite(value) for value in angles):
            return
        with self._robot_feedback_lock:
            self._feedback_angles = angles
            self._feedback_angles_time = time.monotonic()

    def _tool_feedback_callback(self, message: ToolVectorActual) -> None:
        pose = tuple(
            float(value)
            for value in (
                message.x,
                message.y,
                message.z,
                message.rx,
                message.ry,
                message.rz,
            )
        )
        if not all(math.isfinite(value) for value in pose):
            return
        with self._robot_feedback_lock:
            self._feedback_pose = pose
            self._feedback_pose_time = time.monotonic()

    def _get_feedback_value(
        self,
        field: str,
        topic: str,
        maximum_age: float = ROBOT_FEEDBACK_MAX_AGE,
    ) -> tuple[float, ...]:
        with self._robot_feedback_lock:
            value = getattr(self, f'_feedback_{field}')
            timestamp = getattr(self, f'_feedback_{field}_time')
            age = time.monotonic() - timestamp
        if value is None:
            raise DobotServiceError(f'尚未收到实时反馈话题 {topic}')
        if age > maximum_age:
            raise DobotServiceError(
                f'实时反馈话题 {topic} 已过期（{age:.1f}s）'
            )
        return value

    def _get_feedback_angles(self) -> tuple[float, ...]:
        return self._get_feedback_value('angles', '/joint_states_robot')

    def _get_feedback_pose(self) -> tuple[float, ...]:
        return self._get_feedback_value(
            'pose', '/dobot_msgs_v3/msg/ToolVectorActual'
        )

    def _agv_status_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        if not isinstance(value, dict):
            return
        with self._agv_state_lock:
            self._agv_status = value
            self._agv_status_time = time.monotonic()
            self._agv_status_generation = (
                getattr(self, '_agv_status_generation', 0) + 1
            )

    def _agv_stations_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        if not isinstance(value, list):
            return
        stations = [
            station
            for station in value
            if isinstance(station, dict) and station.get('id')
        ]
        with self._agv_state_lock:
            self._agv_stations = stations

    def _agv_map_data_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        if not isinstance(value, dict):
            return
        with self._agv_state_lock:
            self._agv_map_data = value
            self._agv_map_generation += 1

    def agv_driver_ready(self) -> bool:
        return self._agv_clients['stop'].service_is_ready()

    def agv_status(self, maximum_age: float = 2.0) -> dict[str, object]:
        with self._agv_state_lock:
            status = copy.deepcopy(self._agv_status)
            age = time.monotonic() - self._agv_status_time
            generation = getattr(self, '_agv_status_generation', 0)
        if not status or age > maximum_age:
            return {
                'connected': False,
                'safety_reason': 'AGV状态未收到或已经过期',
                'teleop_reason': 'AGV状态未收到或已经过期',
            }
        status['gui_status_age_sec'] = age
        status['gui_status_generation'] = generation
        return status

    def agv_stations(self) -> list[dict[str, object]]:
        with self._agv_state_lock:
            return copy.deepcopy(self._agv_stations)

    def agv_map_data(
        self, after_generation: int = -1
    ) -> tuple[int, dict[str, object] | None]:
        with self._agv_state_lock:
            generation = self._agv_map_generation
            if generation == after_generation:
                return generation, None
            return generation, copy.deepcopy(self._agv_map_data)

    def _call_agv(
        self,
        name: str,
        request,
        timeout: float = 5.0,
    ):
        client = self._agv_clients[name]
        if not client.wait_for_service(timeout_sec=min(timeout, 1.0)):
            raise AgvServiceError(f'AGV ROS服务不可用：{client.srv_name}')
        with self._agv_call_lock:
            future = client.call_async(request)
            completed = threading.Event()
            future.add_done_callback(lambda unused_future: completed.set())
            if not completed.wait(timeout):
                future.cancel()
                raise AgvServiceError(f'调用AGV {name} 超时（{timeout:.1f}s）')
            exception = future.exception()
            if exception is not None:
                raise AgvServiceError(f'调用AGV {name} 失败：{exception}')
            response = future.result()
        if response is None:
            raise AgvServiceError(f'AGV {name} 未返回响应')
        if hasattr(response, 'success') and not response.success:
            raise AgvServiceError(response.message or f'AGV {name}被拒绝')
        return response

    def agv_publish_velocity(self, vx: float, angular_z: float) -> None:
        message = Twist()
        message.linear.x = float(vx)
        message.linear.y = 0.0
        message.angular.z = float(angular_z)
        self._agv_cmd_pub.publish(message)

    def agv_stop(self) -> str:
        self.agv_publish_velocity(0.0, 0.0)
        response = self._call_agv('stop', Trigger.Request(), timeout=4.0)
        return response.message

    def agv_cancel_navigation(self) -> str:
        self.agv_publish_velocity(0.0, 0.0)
        response = self._call_agv(
            'cancel_nav', Trigger.Request(), timeout=5.0
        )
        return response.message

    def agv_confirm_localization(
        self, expected_x: float, expected_y: float, expected_yaw: float
    ) -> str:
        request = ConfirmLocalization.Request()
        request.operator_confirmed = True
        request.expected_x = float(expected_x)
        request.expected_y = float(expected_y)
        request.expected_yaw = float(expected_yaw)
        response = self._call_agv(
            'confirm_localization', request, timeout=8.0
        )
        return response.message

    def agv_load_map(self, map_name: str) -> str:
        request = LoadMap.Request()
        request.operator_confirmed = True
        request.map_name = map_name.strip()
        response = self._call_agv('load_map', request, timeout=25.0)
        return response.message

    def agv_relocalize(
        self, x: float, y: float, yaw: float, radius: float
    ) -> str:
        request = Relocalize.Request()
        request.operator_confirmed = True
        request.x = float(x)
        request.y = float(y)
        request.yaw = float(yaw)
        request.radius = float(radius)
        response = self._call_agv('relocalize', request, timeout=8.0)
        return response.message

    def agv_cancel_relocalization(self) -> str:
        response = self._call_agv(
            'cancel_relocalization', Trigger.Request(), timeout=5.0
        )
        return response.message

    def agv_download_map(self) -> str:
        response = self._call_agv(
            'download_map', Trigger.Request(), timeout=20.0
        )
        return response.message

    def agv_navigate_to_station(
        self,
        station_id: str,
        max_speed: float,
        target_yaw: float | None = None,
        use_station_yaw: bool = True,
    ) -> tuple[str, str]:
        request = NavigateToStation.Request()
        target = station_id.strip()
        request.station_id = target
        request.max_speed = float(max_speed)
        if target_yaw is None and use_station_yaw:
            for station in self.agv_stations():
                if str(station.get('id')) != target:
                    continue
                value = station.get('r', station.get('angle'))
                if isinstance(value, (int, float)) and math.isfinite(value):
                    target_yaw = float(value)
                break
        request.use_target_yaw = target_yaw is not None
        request.target_yaw = 0.0 if target_yaw is None else float(target_yaw)
        response = self._call_agv('navigate', request, timeout=8.0)
        return response.task_id, response.message

    def agv_navigate_to_pose(
        self,
        waypoint_name: str,
        x: float,
        y: float,
        yaw: float,
        max_speed: float,
    ) -> tuple[str, str]:
        request = NavigateToPose.Request()
        request.waypoint_name = waypoint_name.strip()
        request.x = float(x)
        request.y = float(y)
        request.yaw = float(yaw)
        request.max_speed = float(max_speed)
        response = self._call_agv('navigate_pose', request, timeout=8.0)
        return response.task_id, response.message

    def agv_plan_to_station(
        self, target_station_id: str
    ) -> tuple[str, ...]:
        request = PlanToStation.Request()
        request.target_station_id = target_station_id.strip()
        response = self._call_agv(
            'plan_to_station', request, timeout=8.0
        )
        return tuple(response.station_ids)

    def agv_navigate_and_wait(
        self,
        station_id: str,
        max_speed: float,
        timeout_s: float,
        cancel_event: threading.Event | None = None,
        progress: Callable[[str], None] | None = None,
    ) -> bool:
        target = station_id.strip()
        return DobotRosClient._agv_start_and_wait(
            self,
            target_label=target,
            start_navigation=lambda: self.agv_navigate_to_station(
                target, max_speed
            ),
            timeout_s=timeout_s,
            cancel_event=cancel_event,
            progress=progress,
            completion_check=lambda status: (
                DobotRosClient._station_completion_error(status, target)
            ),
        )

    def agv_navigate_pose_and_wait(
        self,
        waypoint_name: str,
        x: float,
        y: float,
        yaw: float,
        max_speed: float,
        timeout_s: float,
        cancel_event: threading.Event | None = None,
        progress: Callable[[str], None] | None = None,
    ) -> bool:
        target = waypoint_name.strip()
        expected = (float(x), float(y), float(yaw))
        return DobotRosClient._agv_start_and_wait(
            self,
            target_label=target,
            start_navigation=lambda: self.agv_navigate_to_pose(
                target, expected[0], expected[1], expected[2], max_speed
            ),
            timeout_s=timeout_s,
            cancel_event=cancel_event,
            progress=progress,
            completion_check=lambda status: (
                DobotRosClient._pose_completion_error(status, expected)
            ),
        )

    @staticmethod
    def _station_completion_error(status: dict, target: str) -> str | None:
        pose = status.get('pose')
        pose = pose if isinstance(pose, dict) else {}
        current_station = str(pose.get('current_station') or '')
        if current_station and current_station != target:
            return (
                f'AGV报告导航完成，但当前航点为 {current_station}，'
                f'不是目标 {target}'
            )
        return None

    @staticmethod
    def _pose_completion_error(
        status: dict, expected: tuple[float, float, float]
    ) -> str | None:
        pose = status.get('pose')
        pose = pose if isinstance(pose, dict) else {}
        yaw = pose.get('angle', pose.get('yaw'))
        try:
            actual = (
                float(pose.get('x')),
                float(pose.get('y')),
                float(yaw),
            )
        except (TypeError, ValueError):
            return 'AGV报告导航完成，但最终位姿不可用'
        position_error = math.hypot(
            actual[0] - expected[0], actual[1] - expected[1]
        )
        yaw_error = abs(
            math.atan2(
                math.sin(actual[2] - expected[2]),
                math.cos(actual[2] - expected[2]),
            )
        )
        if position_error > 0.10 or yaw_error > math.radians(8.0):
            return (
                'AGV报告导航完成，但最终位姿超出校验范围：'
                f'位置误差={position_error:.3f} m，'
                f'角度误差={math.degrees(yaw_error):.1f}°'
            )
        return None

    def _agv_start_and_wait(
        self,
        target_label: str,
        start_navigation: Callable[[], tuple[str, str]],
        timeout_s: float,
        cancel_event: threading.Event | None,
        progress: Callable[[str], None] | None,
        completion_check: Callable[[dict], str | None],
    ) -> bool:
        """Start navigation and wait for a status newer than the request."""
        report = progress or (lambda unused_text: None)
        cancel = cancel_event or threading.Event()
        if cancel.is_set():
            return False

        initial = self.agv_status(maximum_age=2.0)
        if not initial.get('connected'):
            raise AgvServiceError(
                initial.get('safety_reason') or 'AGV状态不可用'
            )
        if not initial.get('safe_to_start_navigation'):
            raise AgvServiceError(
                'AGV导航安全门拒绝：'
                + str(initial.get('safety_reason') or '原因未知')
            )
        baseline_generation = int(initial.get('gui_status_generation', 0))
        task_id, unused_message = start_navigation()
        report(f'AGV导航任务已接受：{task_id}，目标={target_label}')

        started_at = time.monotonic()
        last_generation = baseline_generation
        last_nav_status = None
        active_seen = False
        unavailable_since = None

        def request_cancel(reason: str) -> str:
            try:
                message = self.agv_cancel_navigation()
                return f'{reason}；取消导航已确认：{message}'
            except Exception as exc:
                return f'{reason}；取消导航失败：{exc}'

        while True:
            if cancel.is_set():
                report(request_cancel('收到停止队列请求'))
                return False
            elapsed = time.monotonic() - started_at
            if elapsed > timeout_s:
                raise AgvServiceError(
                    request_cancel(
                        f'AGV到站超时（{timeout_s:.1f}s），目标={target_label}'
                    )
                )

            status = self.agv_status(maximum_age=2.0)
            if not status.get('connected'):
                if unavailable_since is None:
                    unavailable_since = time.monotonic()
                if time.monotonic() - unavailable_since > 3.0:
                    raise AgvServiceError(
                        request_cancel('AGV状态连续3秒不可用')
                    )
                time.sleep(0.2)
                continue
            unavailable_since = None

            generation = int(status.get('gui_status_generation', 0))
            if generation <= last_generation:
                time.sleep(0.2)
                continue
            last_generation = generation

            terminal_reason = ''
            if status.get('emergency_active'):
                terminal_reason = 'AGV急停已激活'
            elif status.get('has_alarm'):
                terminal_reason = 'AGV存在活动报警'
            elif status.get('control_locked'):
                terminal_reason = 'AGV控制权被占用'
            elif status.get('charging'):
                terminal_reason = 'AGV进入充电状态'
            elif not status.get('map_loaded'):
                terminal_reason = 'AGV地图不再处于加载状态'
            elif not status.get('localized'):
                terminal_reason = 'AGV定位状态丢失'
            if terminal_reason:
                raise AgvServiceError(request_cancel(terminal_reason))

            try:
                nav_status = int(status.get('nav_status'))
            except (TypeError, ValueError):
                nav_status = None
            if nav_status != last_nav_status:
                label = {
                    0: '无任务',
                    1: '等待',
                    2: '运行中',
                    3: '暂停',
                    4: '完成',
                    5: '失败',
                    6: '取消',
                }.get(nav_status, '未知')
                report(f'AGV导航状态：{label}（{nav_status}）')
                last_nav_status = nav_status

            if nav_status in {1, 2, 3}:
                active_seen = True
            elif nav_status == 4:
                error = completion_check(status)
                if error:
                    raise AgvServiceError(error)
                return True
            elif nav_status in {5, 6}:
                raise AgvServiceError(
                    f'AGV导航未完成：状态={nav_status}，目标={target_label}'
                )
            elif nav_status == 0 and active_seen:
                error = completion_check(status)
                if error is None:
                    return True
                raise AgvServiceError(
                    f'AGV导航任务提前消失：{error}'
                )
            elif not active_seen and elapsed > 8.0:
                raise AgvServiceError(
                    request_cancel('AGV任务已接受但8秒内未进入导航状态')
                )
            time.sleep(0.2)

    def _call(self, name: str, request, timeout: float | None = None):
        wait_timeout = self.service_timeout if timeout is None else timeout
        client = self._service_clients[name]
        if not client.wait_for_service(timeout_sec=min(wait_timeout, 1.0)):
            raise DobotServiceError(f'ROS 服务不可用：{SERVICE_PREFIX}/{name}')

        with self._call_lock:
            future = client.call_async(request)
            completed = threading.Event()
            future.add_done_callback(lambda unused_future: completed.set())
            if not completed.wait(wait_timeout):
                future.cancel()
                raise DobotServiceError(
                    f'调用 {name} 超时（{wait_timeout:.1f}s）'
                )
            exception = future.exception()
            if exception is not None:
                raise DobotServiceError(f'调用 {name} 失败：{exception}')
            response = future.result()

        if response is None:
            raise DobotServiceError(f'{name} 未返回响应')
        if hasattr(response, 'res') and response.res != 0:
            raise DobotServiceError(f'{name} 返回 res={response.res}')
        return response

    def get_mode(self) -> str:
        return self._call('RobotMode', RobotMode.Request()).mode.strip()

    def get_angles(self) -> tuple[float, ...]:
        response = self._call('GetAngle', GetAngle.Request())
        return parse_numeric_values(response.angle, 6)

    def get_pose(
        self,
        user: int = 0,
        tool: int = 0,
        timeout: float | None = None,
    ) -> tuple[float, ...]:
        request = GetPose.Request()
        request.user = user
        request.tool = tool
        response = self._call('GetPose', request, timeout=timeout)
        return parse_numeric_values(response.pose, 6)

    def get_error_ids(self) -> str:
        response = self._call('GetErrorID', GetErrorID.Request())
        return response.error_id.strip() or '{}'

    def read_status(self) -> dict[str, object]:
        mode = self.get_mode()
        fallbacks = []
        try:
            angles = self.get_angles()
        except DobotServiceError as dashboard_error:
            try:
                angles = self._get_feedback_angles()
            except DobotServiceError as feedback_error:
                raise DobotServiceError(
                    f'{dashboard_error}；关节角反馈兜底失败：{feedback_error}'
                ) from dashboard_error
            fallbacks.append(
                f'关节角使用 /joint_states_robot（{dashboard_error}）'
            )

        try:
            pose = self.get_pose()
        except DobotServiceError as dashboard_error:
            try:
                pose = self._get_feedback_pose()
            except DobotServiceError as feedback_error:
                raise DobotServiceError(
                    f'{dashboard_error}；末端位姿反馈兜底失败：{feedback_error}'
                ) from dashboard_error
            fallbacks.append(
                '末端位姿使用 '
                f'/dobot_msgs_v3/msg/ToolVectorActual（{dashboard_error}）'
            )

        return {
            'mode': mode,
            'angles': angles,
            'pose': pose,
            'fallbacks': tuple(fallbacks),
        }

    def capture_stable_state(
        self, maximum_motion_deg: float = 0.05
    ) -> tuple[tuple[float, ...], tuple[float, ...], str]:
        """Read a consistent point only while the robot is effectively still."""
        mode = self.get_mode()
        if mode not in {'5', '6'}:
            raise DobotServiceError(
                f'只能在机械臂空闲或拖动模式取点；当前为 {mode_text(mode)}'
            )
        before = self.get_angles()
        pose = self.get_pose()
        after = self.get_angles()
        motion = max_joint_error_deg(before, after)
        if motion > maximum_motion_deg:
            raise DobotServiceError(
                f'机械臂仍在移动（采样变化 {motion:.3f}°），请保持静止后重试'
            )
        return after, pose, mode

    def clear_error(self) -> tuple[str, str]:
        before = self.get_error_ids()
        self._call('ClearError', ClearError.Request())
        time.sleep(0.3)
        return before, self.get_mode()

    def resume_from_pause(self) -> str:
        """Resume an explicitly confirmed pause only when no alarm IDs remain."""
        mode = self.get_mode()
        if mode != '10':
            raise DobotServiceError(
                f'解除暂停要求 RobotMode=10；当前为 {mode_text(mode)}'
            )
        error_ids = self.get_error_ids()
        if re.search(r'-?\d+', error_ids):
            raise DobotServiceError(
                f'仍存在报警码 {error_ids}，拒绝执行 Continue；'
                '请先排除报警原因'
            )

        self._call('Continue', Continues.Request())
        return self._wait_for_mode({'5', '7'}, timeout=8.0)

    def enable_robot(self, load: float, speed_factor: int) -> str:
        mode = self.get_mode()
        if mode == '9':
            raise DobotServiceError('机械臂处于报警模式，请先查明报警原因')
        if mode not in {'4', '5'}:
            raise DobotServiceError(
                f'当前模式为 {mode_text(mode)}，不能执行使能操作'
            )
        if mode == '4':
            request = EnableRobot.Request()
            request.load = float(load)
            self._call('EnableRobot', request)
        self.set_speed_factor(speed_factor)
        return self._wait_for_mode({'5'}, timeout=8.0)

    def disable_robot(self) -> str:
        mode = self.get_mode()
        if mode != '5':
            raise DobotServiceError(
                f'仅允许在模式5（已使能且空闲）下使机械臂下使能；'
                f'当前为 {mode_text(mode)}'
            )
        self._call('DisableRobot', DisableRobot.Request())
        return self._wait_for_mode({'4'}, timeout=8.0)

    def set_speed_factor(self, ratio: int) -> None:
        if not 1 <= ratio <= 100:
            raise DobotServiceError('速度百分比必须在 1~100 之间')
        request = SpeedFactor.Request()
        request.ratio = ratio
        self._call('SpeedFactor', request)

    def set_collision_level(self, level: int) -> None:
        if not 0 <= level <= 5:
            raise DobotServiceError('碰撞检测等级必须在 0~5 之间')
        request = SetCollisionLevel.Request()
        request.level = level
        self._call('SetCollisionLevel', request)

    def set_safe_skin(self, enabled: bool) -> None:
        request = SetSafeSkin.Request()
        request.status = 1 if enabled else 0
        self._call('SetSafeSkin', request)

    def start_drag(
        self,
        disable_collision: bool,
        disable_safe_skin: bool,
        restore_collision_level: int = 3,
    ) -> str:
        mode = self.get_mode()
        if mode != '5':
            raise DobotServiceError(
                f'进入拖动前必须为模式5；当前为 {mode_text(mode)}'
            )
        collision_changed = False
        safe_skin_changed = False
        try:
            if disable_collision:
                self.set_collision_level(0)
                collision_changed = True
            if disable_safe_skin:
                self.set_safe_skin(False)
                safe_skin_changed = True
            self._call('StartDrag', StartDrag.Request())
            return self._wait_for_mode({'6'}, timeout=8.0)
        except Exception as exc:
            rollback_errors = []
            if safe_skin_changed:
                try:
                    self.set_safe_skin(True)
                except DobotServiceError as rollback_exc:
                    rollback_errors.append(f'电子皮肤恢复失败：{rollback_exc}')
            if collision_changed:
                try:
                    self.set_collision_level(restore_collision_level)
                except DobotServiceError as rollback_exc:
                    rollback_errors.append(f'碰撞检测恢复失败：{rollback_exc}')
            if rollback_errors:
                raise DobotServiceError(
                    f'{exc}；安全保护回滚不完整：{"；".join(rollback_errors)}'
                ) from exc
            raise

    def stop_drag(
        self,
        restore_collision_level: int | None,
        restore_safe_skin: bool = False,
    ) -> str:
        errors = []
        try:
            self._call('StopDrag', StopDrag.Request())
        except DobotServiceError as exc:
            errors.append(f'StopDrag失败：{exc}')

        if restore_safe_skin:
            try:
                self.set_safe_skin(True)
            except DobotServiceError as exc:
                errors.append(f'电子皮肤恢复失败：{exc}')
        if restore_collision_level is not None:
            try:
                self.set_collision_level(restore_collision_level)
            except DobotServiceError as exc:
                errors.append(f'碰撞检测恢复失败：{exc}')
        if errors:
            raise DobotServiceError('；'.join(errors))
        return self._wait_for_mode({'5'}, timeout=8.0)

    def restore_drag_protections(
        self,
        collision_level: int,
        restore_safe_skin: bool = True,
    ) -> None:
        errors = []
        if restore_safe_skin:
            try:
                self.set_safe_skin(True)
            except DobotServiceError as exc:
                errors.append(f'电子皮肤恢复失败：{exc}')
        try:
            self.set_collision_level(collision_level)
        except DobotServiceError as exc:
            errors.append(f'碰撞检测恢复失败：{exc}')
        if errors:
            raise DobotServiceError('；'.join(errors))

    def emergency_stop(self) -> None:
        self.motion_cancel.set()
        self._call('EmergencyStop', EmergencyStop.Request(), timeout=3.0)

    def _wait_for_mode(self, accepted: set[str], timeout: float) -> str:
        deadline = time.monotonic() + timeout
        last_mode = ''
        while time.monotonic() < deadline:
            last_mode = self.get_mode()
            if last_mode in accepted:
                return last_mode
            time.sleep(0.25)
        raise DobotServiceError(
            f'等待模式 {sorted(accepted)} 超时，当前为 {mode_text(last_mode)}'
        )

    def move_to_joints(
        self,
        target: Sequence[float],
        speed_factor: int,
        speed_j: int,
        acc_j: int,
        tolerance_deg: float = 0.5,
        start_timeout: float = 10.0,
        motion_timeout: float = 300.0,
        progress: Callable[[str], None] | None = None,
    ) -> tuple[float, ...]:
        """Move to a joint point and poll state without calling Sync."""
        if len(target) != 6:
            raise DobotServiceError('目标点必须包含6个关节角')
        for label, value in (
            ('全局速度', speed_factor),
            ('关节速度', speed_j),
            ('关节加速度', acc_j),
        ):
            if not 1 <= value <= 100:
                raise DobotServiceError(f'{label}必须在 1~100 之间')

        mode = self.get_mode()
        if mode != '5':
            raise DobotServiceError(
                f'点位运动前必须为模式5；当前为 {mode_text(mode)}'
            )
        start = self.get_angles()
        if max_joint_error_deg(start, target) <= tolerance_deg:
            return start

        self.motion_cancel.clear()
        self.set_speed_factor(speed_factor)
        request = JointMovJ.Request()
        (
            request.j1,
            request.j2,
            request.j3,
            request.j4,
            request.j5,
            request.j6,
        ) = [float(value) for value in target]
        request.param_value = [f'SpeedJ={speed_j}', f'AccJ={acc_j}']
        self._call('JointMovJ', request)

        started_at = time.monotonic()
        deadline = started_at + motion_timeout
        motion_started = False
        stable_samples = 0
        actual = start
        last_report = 0.0

        while time.monotonic() < deadline:
            if self.motion_cancel.is_set():
                raise DobotServiceError('点位运动监控已停止（急停已请求）')
            actual = self.get_angles()
            mode = self.get_mode()
            now = time.monotonic()
            start_delta = max_joint_error_deg(actual, start)
            target_error = max_joint_error_deg(actual, target)
            if mode == '7' or start_delta >= MOTION_START_DELTA_DEG:
                motion_started = True

            if mode not in {'5', '7'}:
                diagnostic = ''
                if mode == '9':
                    try:
                        diagnostic = f'，报警码={self.get_error_ids()}'
                    except DobotServiceError:
                        pass
                raise DobotServiceError(
                    f'运动中检测到 {mode_text(mode)}{diagnostic}'
                )
            if not motion_started and now - started_at >= start_timeout:
                raise DobotServiceError(
                    '运动命令已接受，但机械臂没有开始运动；请检查报警、'
                    '控制器队列和示教器状态，不要调用 Sync'
                )

            if target_error <= tolerance_deg and mode == '5':
                stable_samples += 1
                if stable_samples >= TARGET_STABLE_SAMPLES:
                    return actual
            else:
                stable_samples = 0

            if progress is not None and now - last_report >= 1.0:
                progress(
                    f'运动中：模式 {mode}，最大目标误差 '
                    f'{target_error:.3f}°，当前 [{format_joints(actual)}]'
                )
                last_report = now
            time.sleep(0.35)

        raise DobotServiceError(
            f'等待点位运动完成超时；最后角度 [{format_joints(actual)}]'
        )

    @staticmethod
    def _pose_error(
        actual: Sequence[float], target: Sequence[float]
    ) -> tuple[float, float]:
        actual_matrix = dobot_pose_to_matrix(actual)
        target_matrix = dobot_pose_to_matrix(target)
        position_error = math.dist(
            actual_matrix[:3, 3], target_matrix[:3, 3]
        )
        orientation_error = rotation_angle_degrees(
            actual_matrix[:3, :3] @ target_matrix[:3, :3].T
        )
        return position_error, orientation_error

    def move_to_pose(
        self,
        target: Sequence[float],
        motion_type: str,
        speed_factor: int,
        speed: int,
        acc: int,
        position_tolerance_mm: float = 1.0,
        orientation_tolerance_deg: float = 1.0,
        start_timeout: float = 10.0,
        motion_timeout: float = 300.0,
        progress: Callable[[str], None] | None = None,
    ) -> tuple[float, ...]:
        """Execute a User0/Tool0 Cartesian target and poll without Sync."""
        if motion_type not in {'joint', 'linear'}:
            raise DobotServiceError('笛卡尔运动类型必须为joint或linear')
        if len(target) != 6 or not all(
            math.isfinite(float(value)) for value in target
        ):
            raise DobotServiceError('目标位姿必须包含6个有限数值')
        target_pose = tuple(float(value) for value in target)
        for label, value in (
            ('全局速度', speed_factor),
            ('运动速度', speed),
            ('运动加速度', acc),
        ):
            if not 1 <= value <= 100:
                raise DobotServiceError(f'{label}必须在 1~100 之间')
        if not 0.1 <= position_tolerance_mm <= 20.0:
            raise DobotServiceError('位置容差必须在0.1~20.0 mm之间')
        if not 0.1 <= orientation_tolerance_deg <= 5.0:
            raise DobotServiceError('姿态容差必须在0.1~5.0°之间')

        mode = self.get_mode()
        if mode != '5':
            raise DobotServiceError(
                f'笛卡尔运动前必须为模式5；当前为 {mode_text(mode)}'
            )
        start = self.get_pose(user=0, tool=0)
        start_position_error, start_orientation_error = self._pose_error(
            start, target_pose
        )
        if (
            start_position_error <= position_tolerance_mm
            and start_orientation_error <= orientation_tolerance_deg
        ):
            return start

        self.motion_cancel.clear()
        self.set_speed_factor(speed_factor)
        request_type = MovL if motion_type == 'linear' else MovJ
        service_name = 'MovL' if motion_type == 'linear' else 'MovJ'
        request = request_type.Request()
        (
            request.x,
            request.y,
            request.z,
            request.rx,
            request.ry,
            request.rz,
        ) = target_pose
        speed_name = 'SpeedL' if motion_type == 'linear' else 'SpeedJ'
        acc_name = 'AccL' if motion_type == 'linear' else 'AccJ'
        request.param_value = [
            f'{speed_name}={speed}',
            f'{acc_name}={acc}',
            'User=0',
            'Tool=0',
        ]
        self._call(service_name, request)

        started_at = time.monotonic()
        deadline = started_at + motion_timeout
        motion_started = False
        stable_samples = 0
        actual = start
        last_report = 0.0
        while time.monotonic() < deadline:
            if self.motion_cancel.is_set():
                raise DobotServiceError('笛卡尔运动监控已停止（急停已请求）')
            actual = self.get_pose(user=0, tool=0)
            mode = self.get_mode()
            now = time.monotonic()
            position_error, orientation_error = self._pose_error(
                actual, target_pose
            )
            moved_position, moved_orientation = self._pose_error(actual, start)
            if (
                mode == '7'
                or moved_position >= 0.2
                or moved_orientation >= 0.1
            ):
                motion_started = True

            if mode not in {'5', '7'}:
                diagnostic = ''
                if mode == '9':
                    try:
                        diagnostic = f'，报警码={self.get_error_ids()}'
                    except DobotServiceError:
                        pass
                raise DobotServiceError(
                    f'笛卡尔运动中检测到 {mode_text(mode)}{diagnostic}'
                )
            if not motion_started and now - started_at >= start_timeout:
                raise DobotServiceError(
                    '笛卡尔命令已接受，但机械臂没有开始运动；请检查报警、'
                    '目标可达性和控制器队列'
                )

            if (
                position_error <= position_tolerance_mm
                and orientation_error <= orientation_tolerance_deg
                and mode == '5'
            ):
                stable_samples += 1
                if stable_samples >= TARGET_STABLE_SAMPLES:
                    return actual
            else:
                stable_samples = 0

            if progress is not None and now - last_report >= 1.0:
                progress(
                    f'{service_name}运动中：模式 {mode}，位置误差 '
                    f'{position_error:.2f} mm，姿态误差 '
                    f'{orientation_error:.2f}°'
                )
                last_report = now
            time.sleep(0.35)

        raise DobotServiceError(
            f'等待笛卡尔运动完成超时；最后Tool0位姿 '
            f'[{", ".join(f"{value:.3f}" for value in actual)}]'
        )

    def connect_gripper(
        self,
        ip: str = '127.0.0.1',
        port: int = 60000,
        slave_id: int = 1,
    ) -> int:
        if self.gripper_index is not None:
            return self.gripper_index
        request = ModbusCreate.Request()
        request.ip = ip
        request.port = port
        request.slave_id = slave_id
        request.is_rtu = 1
        response = self._call('ModbusCreate', request)
        match = re.search(r'-?\d+', response.index)
        if match is None:
            raise DobotServiceError(
                f'无法解析 ModbusCreate index：{response.index!r}'
            )
        index = int(match.group(0))
        if not 0 <= index <= 4:
            raise DobotServiceError(f'控制器返回了无效 Modbus index={index}')
        self.gripper_index = index
        return index

    def disconnect_gripper(self) -> None:
        if self.gripper_index is None:
            return
        request = ModbusClose.Request()
        request.index = self.gripper_index
        self._call('ModbusClose', request)
        self.gripper_index = None

    def _require_gripper(self) -> int:
        if self.gripper_index is None:
            raise DobotServiceError('请先创建夹爪 Modbus 通道')
        return self.gripper_index

    def _write_gripper_register(self, address: int, value: int) -> None:
        request = SetHoldRegs.Request()
        request.index = self._require_gripper()
        request.addr = address
        request.count = 1
        request.val_tab = f'{{{value}}}'
        request.val_type = 'U16'
        self._call('SetHoldRegs', request)

    def initialize_gripper(self) -> None:
        self._write_gripper_register(0x0100, 1)

    def initialize_gripper_and_wait(self, timeout: float = 15.0) -> None:
        """Initialize the gripper and wait for its feedback to confirm it."""
        self.initialize_gripper()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            initialized, unused_grip_state, unused_position = (
                self.read_gripper_status()
            )
            if initialized == 1:
                return
            time.sleep(0.2)
        raise DobotServiceError('等待夹爪初始化完成超时')

    def set_gripper_force(self, force_percent: int) -> None:
        if not 20 <= force_percent <= 100:
            raise DobotServiceError('夹持力必须在 20~100% 之间')
        self._write_gripper_register(0x0101, force_percent)

    def set_gripper_position(self, position: int) -> None:
        if not 0 <= position <= 1000:
            raise DobotServiceError('夹爪位置必须在 0~1000 之间')
        self._write_gripper_register(0x0103, position)

    def move_gripper_and_wait(
        self,
        position: int,
        timeout: float = 15.0,
        position_tolerance: int = 10,
    ) -> tuple[int, int, int]:
        """Set a position and wait for arrival or an object-gripped state."""
        self.set_gripper_position(position)
        deadline = time.monotonic() + timeout
        last_status = (0, 0, 0)
        while time.monotonic() < deadline:
            last_status = self.read_gripper_status()
            initialized, grip_state, actual_position = last_status
            if initialized != 1:
                raise DobotServiceError('夹爪尚未初始化或初始化状态异常')
            if grip_state == 3:
                raise DobotServiceError('夹爪反馈物体脱落（状态3）')
            if grip_state == 1 and (
                abs(actual_position - position) <= position_tolerance
            ):
                return last_status
            if grip_state == 2 and position <= actual_position:
                return last_status
            time.sleep(0.2)
        raise DobotServiceError(
            f'等待夹爪到位超时；最后状态={last_status}'
        )

    def read_gripper_status(self) -> tuple[int, int, int]:
        request = GetHoldRegs.Request()
        request.index = self._require_gripper()
        request.addr = 0x0200
        request.count = 3
        request.val_type = 'U16'
        response = self._call('GetHoldRegs', request)
        values = parse_integer_values(response.value, minimum_count=3)
        return values[0], values[1], values[2]
