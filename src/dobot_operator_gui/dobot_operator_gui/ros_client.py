"""ROS 2 client used by the Dobot operator GUI."""

import copy
import json
import re
import threading
import time
from typing import Callable, Sequence

from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

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
from dobot_msgs_v3.srv import RobotMode
from dobot_msgs_v3.srv import SetCollisionLevel
from dobot_msgs_v3.srv import SetHoldRegs
from dobot_msgs_v3.srv import SetSafeSkin
from dobot_msgs_v3.srv import SpeedFactor
from dobot_msgs_v3.srv import StartDrag
from dobot_msgs_v3.srv import StopDrag
from seer_agv_msgs.srv import ConfirmLocalization, LoadMap, NavigateToStation
from seer_agv_msgs.srv import Relocalize

from .core import format_joints
from .core import max_joint_error_deg
from .core import mode_text
from .core import parse_integer_values
from .core import parse_numeric_values


SERVICE_PREFIX = '/dobot_bringup_v3/srv'
MOTION_START_DELTA_DEG = 0.05
TARGET_STABLE_SAMPLES = 2


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

        self._agv_status: dict[str, object] = {}
        self._agv_status_time = 0.0
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
        }

    def driver_ready(self) -> bool:
        return self._service_clients['RobotMode'].service_is_ready()

    def available_services(self) -> tuple[int, int]:
        ready = sum(
            client.service_is_ready()
            for client in self._service_clients.values()
        )
        return ready, len(self._service_clients)

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
        if not status or age > maximum_age:
            return {
                'connected': False,
                'safety_reason': 'AGV状态未收到或已经过期',
                'teleop_reason': 'AGV状态未收到或已经过期',
            }
        status['gui_status_age_sec'] = age
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
        self, station_id: str, max_speed: float
    ) -> tuple[str, str]:
        request = NavigateToStation.Request()
        request.station_id = station_id.strip()
        request.max_speed = float(max_speed)
        response = self._call_agv('navigate', request, timeout=8.0)
        return response.task_id, response.message

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
        angles = self.get_angles()
        pose = self.get_pose()
        return {'mode': mode, 'angles': angles, 'pose': pose}

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
