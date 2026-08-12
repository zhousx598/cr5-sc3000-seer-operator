import json
import math
import threading
import time
from typing import Any
import uuid

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from seer_agv_driver.seer_client import DEFAULT_HOST, Footprint, SeerClient, SafetyState
from seer_agv_driver.seer_client import evaluate_safety_state
from seer_agv_driver.seer_client import normalize_travel_mode
from seer_agv_msgs.srv import ConfirmLocalization, LoadMap, NavigateToPose
from seer_agv_msgs.srv import NavigateToStation, PlanToStation
from seer_agv_msgs.srv import Relocalize


DEFAULT_FOOTPRINT = Footprint(width=0.7, head=0.52, tail=0.48, height=0.3)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def numeric(value: Any, default: float = 0.0) -> float:
    return float(value) if isinstance(value, (int, float)) else default


def make_color(r: float, g: float, b: float, a: float) -> Any:
    color = type("Color", (), {})()
    color.r = r
    color.g = g
    color.b = b
    color.a = a
    return color


class SeerAgvNode(Node):
    """Single TCP owner for SEER status, navigation, and motion commands."""

    def __init__(self) -> None:
        super().__init__("seer_agv_node")
        self.declare_parameter("host", DEFAULT_HOST)
        self.declare_parameter("timeout_sec", 2.0)
        self.declare_parameter("fast_status_period_sec", 0.2)
        self.declare_parameter("slow_status_period_sec", 0.5)
        self.declare_parameter("fast_status_timeout_sec", 0.8)
        self.declare_parameter("slow_status_timeout_sec", 5.0)
        self.declare_parameter("enable_cmd_vel", False)
        self.declare_parameter("cmd_vel_topic", "/seer_agv/cmd_vel")
        self.declare_parameter("cmd_period_sec", 0.1)
        self.declare_parameter("watchdog_timeout_sec", 0.4)
        self.declare_parameter("differential_drive", True)
        self.declare_parameter("max_vx", 0.1)
        self.declare_parameter("max_w", 0.2)
        self.declare_parameter("max_accel_xy", 0.1)
        self.declare_parameter("max_accel_w", 0.2)
        self.declare_parameter("motion_duration_ms", 300)
        self.declare_parameter("max_navigation_speed", 0.08)
        self.declare_parameter("debug_motion", False)
        self.declare_parameter("download_map_on_start", True)
        self.declare_parameter("map_name", "")
        self.declare_parameter("frame_id", "seer_map")
        self.declare_parameter("base_frame_id", "seer_base_link")
        self.declare_parameter("odom_topic", "/seer_agv/odom")
        self.declare_parameter("publish_tf", True)

        self._fast_group = MutuallyExclusiveCallbackGroup()
        self._slow_group = MutuallyExclusiveCallbackGroup()
        self._control_group = MutuallyExclusiveCallbackGroup()
        self._service_group = ReentrantCallbackGroup()
        self._state_lock = threading.RLock()
        self._control_state_lock = threading.Lock()

        host = str(self.get_parameter("host").value)
        timeout = float(self.get_parameter("timeout_sec").value)
        self.differential_drive = bool(
            self.get_parameter("differential_drive").value
        )
        self.client = SeerClient(
            host=host,
            timeout=timeout,
            max_vx=float(self.get_parameter("max_vx").value),
            max_vy=0.0 if self.differential_drive else 0.1,
            max_w=float(self.get_parameter("max_w").value),
            max_duration_ms=int(self.get_parameter("motion_duration_ms").value),
        )
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.enable_cmd_vel = bool(self.get_parameter("enable_cmd_vel").value)
        self.watchdog_timeout_sec = float(
            self.get_parameter("watchdog_timeout_sec").value
        )
        self.cmd_period_sec = float(self.get_parameter("cmd_period_sec").value)
        self.motion_duration_ms = int(
            self.get_parameter("motion_duration_ms").value
        )
        self.max_navigation_speed = min(
            0.08,
            abs(float(self.get_parameter("max_navigation_speed").value)),
        )
        self.fast_status_timeout_sec = float(
            self.get_parameter("fast_status_timeout_sec").value
        )
        self.slow_status_timeout_sec = float(
            self.get_parameter("slow_status_timeout_sec").value
        )
        self.debug_motion = bool(self.get_parameter("debug_motion").value)
        self.max_accel_xy = abs(
            float(self.get_parameter("max_accel_xy").value)
        )
        self.max_accel_w = abs(float(self.get_parameter("max_accel_w").value))

        self.pose_pub = self.create_publisher(PoseStamped, "/seer_agv/pose", 10)
        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self.status_pub = self.create_publisher(String, "/seer_agv/status", 10)
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.stations_pub = self.create_publisher(
            String, "/seer_agv/stations", map_qos
        )
        self.map_pub = self.create_publisher(
            MarkerArray, "/seer_agv/map_markers", map_qos
        )
        self.map_data_pub = self.create_publisher(
            String, "/seer_agv/map_data", map_qos
        )
        self.footprint_pub = self.create_publisher(
            MarkerArray, "/seer_agv/footprint_markers", 10
        )

        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None
        )
        self.stop_srv = self.create_service(
            Trigger,
            "/seer_agv/stop",
            self._handle_stop,
            callback_group=self._control_group,
        )
        self.confirm_localization_srv = self.create_service(
            ConfirmLocalization,
            "/seer_agv/confirm_localization",
            self._handle_confirm_localization,
            callback_group=self._control_group,
        )
        self.load_map_srv = self.create_service(
            LoadMap,
            "/seer_agv/load_map",
            self._handle_load_map,
            callback_group=self._control_group,
        )
        self.relocalize_srv = self.create_service(
            Relocalize,
            "/seer_agv/relocalize",
            self._handle_relocalize,
            callback_group=self._control_group,
        )
        self.cancel_relocalization_srv = self.create_service(
            Trigger,
            "/seer_agv/cancel_relocalization",
            self._handle_cancel_relocalization,
            callback_group=self._control_group,
        )
        self.cancel_nav_srv = self.create_service(
            Trigger,
            "/seer_agv/cancel_nav",
            self._handle_cancel_nav,
            callback_group=self._service_group,
        )
        self.download_map_srv = self.create_service(
            Trigger,
            "/seer_agv/download_map",
            self._handle_download_map,
            callback_group=self._service_group,
        )
        self.navigate_srv = self.create_service(
            NavigateToStation,
            "/seer_agv/navigate_to_station",
            self._handle_navigate_to_station,
            callback_group=self._service_group,
        )
        self.navigate_pose_srv = self.create_service(
            NavigateToPose,
            "/seer_agv/navigate_to_pose",
            self._handle_navigate_to_pose,
            callback_group=self._service_group,
        )
        self.plan_to_station_srv = self.create_service(
            PlanToStation,
            "/seer_agv/plan_to_station",
            self._handle_plan_to_station,
            callback_group=self._service_group,
        )

        self._last_status: SafetyState | None = None
        self._fast_status: dict[str, Any] | None = None
        self._slow_status: dict[str, dict[str, Any]] = {}
        self._slow_status_times: dict[str, float] = {}
        self._last_fast_status_time = 0.0
        self._last_cmd_time = 0.0
        self._target_cmd = (0.0, 0.0, 0.0)
        self._sent_cmd = (0.0, 0.0, 0.0)
        self._last_sent_time = time.monotonic()
        self._stopped = True
        self._safety_stop_latched = True
        self._had_safe_state = False
        self._slow_query_index = 0
        self._slow_queries = (
            ("loc_status", self.client.get_loc_status),
            ("map_status", self.client.get_map_status),
            ("battery", self.client.get_battery),
            ("control_owner", self.client.get_control_owner),
        )
        self._map_loaded_once = False
        self._map_markers: MarkerArray | None = None
        self._map_data_message: String | None = None
        self._current_map = ""
        self._stored_maps: list[str] = []
        self.footprint = DEFAULT_FOOTPRINT
        self._load_footprint_from_model()

        self.create_timer(
            float(self.get_parameter("fast_status_period_sec").value),
            self._poll_fast_status,
            callback_group=self._fast_group,
        )
        self.create_timer(
            float(self.get_parameter("slow_status_period_sec").value),
            self._poll_slow_status,
            callback_group=self._slow_group,
        )
        self.create_timer(
            1.0, self._republish_map, callback_group=self._service_group
        )

        if self.enable_cmd_vel:
            topic = str(self.get_parameter("cmd_vel_topic").value)
            self.cmd_sub = self.create_subscription(
                Twist,
                topic,
                self._cmd_vel_cb,
                10,
                callback_group=self._control_group,
            )
            self.create_timer(
                self.cmd_period_sec,
                self._send_cmd_timer,
                callback_group=self._control_group,
            )
            self.get_logger().warn(
                f"cmd_vel enabled on {topic}; differential-drive and safety gates active"
            )
        else:
            self.get_logger().info(
                "cmd_vel disabled; set enable_cmd_vel:=true to allow low-speed jog"
            )

        if bool(self.get_parameter("download_map_on_start").value):
            self.create_timer(
                0.5,
                self._download_map_once,
                callback_group=self._service_group,
            )

    def _poll_fast_status(self) -> None:
        try:
            fast = self.client.get_all2()
        except Exception as exc:
            self._warn_limited(f"all2 status poll failed: {exc}")
            with self._state_lock:
                was_available = self._last_fast_status_time > 0.0
                self._last_status = None
            if was_available and not self._safety_stop_latched:
                self._safety_stop_latched = True
                self._send_stop("fast status unavailable", force=True)
            self._publish_status()
            return

        with self._state_lock:
            self._fast_status = fast
            self._last_fast_status_time = time.monotonic()
        self._publish_pose(fast)
        self._refresh_safety_state()

    def _poll_slow_status(self) -> None:
        key, getter = self._slow_queries[self._slow_query_index]
        self._slow_query_index = (
            self._slow_query_index + 1
        ) % len(self._slow_queries)
        try:
            value = getter()
        except Exception as exc:
            self._warn_limited(f"{key} poll failed: {exc}")
            return
        with self._state_lock:
            self._slow_status[key] = value
            self._slow_status_times[key] = time.monotonic()
        self._refresh_safety_state()

    def _slow_status_is_fresh(self, now: float) -> bool:
        required = ("loc_status", "map_status", "battery", "control_owner")
        return all(
            key in self._slow_status_times
            and now - self._slow_status_times[key] <= self.slow_status_timeout_sec
            for key in required
        )

    def _refresh_safety_state(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            fast = self._fast_status
            if (
                fast is None
                or now - self._last_fast_status_time > self.fast_status_timeout_sec
                or not self._slow_status_is_fresh(now)
            ):
                self._last_status = None
                safety = None
            else:
                safety = evaluate_safety_state(
                    fast,
                    self._slow_status["loc_status"],
                    self._slow_status["map_status"],
                    self._slow_status["battery"],
                    self._slow_status["control_owner"],
                )
                self._last_status = safety

        if safety is not None and safety.safe_to_move:
            self._had_safe_state = True
            self._safety_stop_latched = False
        elif self._had_safe_state and not self._safety_stop_latched:
            self._safety_stop_latched = True
            reason = safety.reason if safety is not None else "status stale"
            self._send_stop(f"safety gate closed: {reason}", force=True)
        self._publish_status()

    def _publish_pose(self, fast: dict[str, Any]) -> None:
        stamp = self.get_clock().now().to_msg()
        x = numeric(fast.get("x"))
        y = numeric(fast.get("y"))
        yaw = numeric(fast.get("angle"), numeric(fast.get("yaw")))
        quat = yaw_to_quaternion(yaw)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation = quat
        self.pose_pub.publish(pose_msg)

        odom = Odometry()
        odom.header = pose_msg.header
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose = pose_msg.pose
        odom.twist.twist.linear.x = numeric(fast.get("vx"))
        odom.twist.twist.linear.y = numeric(fast.get("vy"))
        odom.twist.twist.angular.z = numeric(fast.get("w"))
        self.odom_pub.publish(odom)

        if self.tf_broadcaster:
            transform = TransformStamped()
            transform.header = pose_msg.header
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.rotation = quat
            self.tf_broadcaster.sendTransform(transform)
        self._publish_current_footprint(stamp, x, y, yaw)

    def _publish_status(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            safety = self._last_status
            fast = dict(self._fast_status or {})
            slow = {key: dict(value) for key, value in self._slow_status.items()}
            current_map = self._current_map
            stored_maps = list(self._stored_maps)
            target_cmd = self._target_cmd
            sent_cmd = self._sent_cmd
            fast_fresh = (
                self._last_fast_status_time > 0.0
                and now - self._last_fast_status_time
                <= self.fast_status_timeout_sec
            )
            slow_fresh = self._slow_status_is_fresh(now)

        payload = {
            "connected": bool(fast_fresh and slow_fresh),
            "safe_to_move": bool(safety and safety.safe_to_move),
            "safe_for_teleop": bool(safety and safety.safe_for_teleop),
            "safe_to_start_navigation": bool(
                safety and safety.safe_to_start_navigation
            ),
            "safety_reason": (
                safety.reason if safety is not None else "status incomplete or stale"
            ),
            "teleop_reason": (
                safety.teleop_reason
                if safety is not None
                else "status incomplete or stale"
            ),
            "localized": bool(safety and safety.localized),
            "localization_pending_confirmation": bool(
                safety and safety.localization_pending_confirmation
            ),
            "map_loaded": bool(safety and safety.map_loaded),
            "blocked": bool(safety and safety.blocked),
            "slowed": bool(safety and safety.slowed),
            "has_alarm": bool(safety and safety.has_alarm),
            "emergency_active": bool(safety and safety.emergency_active),
            "charging": bool(safety and safety.charging),
            "control_locked": bool(safety and safety.control_locked),
            "nav_status": safety.nav_status if safety is not None else None,
            "pose": {
                key: fast.get(key)
                for key in ("x", "y", "angle", "yaw", "current_station")
            },
            "speed": {key: fast.get(key) for key in ("vx", "vy", "w", "is_stop")},
            "loc_status": slow.get("loc_status", {}),
            "map_status": slow.get("map_status", {}),
            "battery": slow.get("battery", {}),
            "control_owner": slow.get("control_owner", {}),
            "alarm": {
                key: fast.get(key, [])
                for key in ("fatals", "errors", "warnings")
            },
            "current_map": current_map,
            "stored_maps": stored_maps,
            "command": {
                "target_vx": target_cmd[0],
                "target_vy": target_cmd[1],
                "target_w": target_cmd[2],
                "sent_vx": sent_cmd[0],
                "sent_vy": sent_cmd[1],
                "sent_w": sent_cmd[2],
            },
        }
        message = String()
        message.data = json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        )
        self.status_pub.publish(message)

    def _load_footprint_from_model(self) -> None:
        try:
            self.footprint = self.client.get_footprint()
            self.get_logger().info(
                "loaded robot footprint from model: "
                f"width={self.footprint.width:.3f}, head={self.footprint.head:.3f}, "
                f"tail={self.footprint.tail:.3f}, height={self.footprint.height:.3f}"
            )
        except Exception as exc:
            self.footprint = DEFAULT_FOOTPRINT
            self._warn_limited(
                "robot model footprint download failed, using AMB-300 default "
                f"width={self.footprint.width:.3f}, head={self.footprint.head:.3f}, "
                f"tail={self.footprint.tail:.3f}: {exc}"
            )

    def _publish_current_footprint(self, stamp: Any, x: float, y: float, yaw: float) -> None:
        markers = MarkerArray()
        delete = Marker()
        delete.header.stamp = stamp
        delete.header.frame_id = self.frame_id
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)
        marker_id = 1
        marker_id = _add_footprint(
            markers,
            marker_id,
            "current_agv",
            x,
            y,
            yaw,
            self.footprint,
            self.frame_id,
            fill=(0.05, 0.65, 0.35, 0.32),
            line=(0.02, 0.95, 0.35, 1.0),
            z=0.06,
            height_scale=0.04,
            stamp=stamp,
        )
        _add_heading_line(markers, marker_id, "current_agv_heading", x, y, yaw, self.footprint, self.frame_id, stamp)
        self.footprint_pub.publish(markers)

    def _cmd_vel_cb(self, msg: Twist) -> None:
        if self.differential_drive and abs(msg.linear.y) > 1e-6:
            self._warn_limited(
                "ignored non-zero cmd_vel.linear.y on differential-drive AGV"
            )
        with self._state_lock:
            self._target_cmd = (
                self.client._clamp(msg.linear.x, self.client.max_vx),
                0.0,
                self.client._clamp(msg.angular.z, self.client.max_w),
            )
            self._last_cmd_time = time.monotonic()

    def _send_cmd_timer(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            last_cmd_time = self._last_cmd_time
            target_cmd = self._target_cmd
        if last_cmd_time <= 0.0 or now - last_cmd_time > self.watchdog_timeout_sec:
            with self._state_lock:
                self._target_cmd = (0.0, 0.0, 0.0)
            self._send_stop("cmd_vel watchdog timeout")
            return
        if not self._is_status_fresh_and_safe(now):
            with self._state_lock:
                reason = (
                    self._last_status.teleop_reason
                    if self._last_status is not None
                    else "status unavailable"
                )
            if self.debug_motion:
                self._warn_limited(f"cmd_vel inhibited before 2010 jog: {reason}")
            self._send_stop(f"motion inhibited: {reason}")
            return

        cmd = self._accel_limit(
            self._sent_cmd, target_cmd, now - self._last_sent_time
        )
        self._last_sent_time = now
        if self._is_zero(cmd):
            self._send_stop("zero cmd_vel")
            return
        try:
            # Mark as potentially moving before transmitting.  If the response
            # is lost, the forced stop path must not be skipped.
            self._stopped = False
            self.client.jog(cmd[0], cmd[1], cmd[2], self.motion_duration_ms)
            if self.debug_motion:
                self.get_logger().info(
                    f"sent 2010 jog vx={cmd[0]:.4f}, vy={cmd[1]:.4f}, "
                    f"w={cmd[2]:.4f}, duration={self.motion_duration_ms} ms"
                )
            self._sent_cmd = cmd
        except Exception as exc:
            self._warn_limited(f"2010 jog uncertain/failed, forcing stop: {exc}")
            self._send_stop("jog failed or response lost", force=True)

    def _is_status_fresh_and_safe(self, now: float) -> bool:
        with self._state_lock:
            if self._last_status is None:
                return False
            return (
                self._last_status.safe_for_teleop
                and now - self._last_fast_status_time
                <= self.fast_status_timeout_sec
                and self._slow_status_is_fresh(now)
            )

    def _accel_limit(self, current: tuple[float, float, float], target: tuple[float, float, float], dt: float) -> tuple[float, float, float]:
        return (
            self._step_toward(current[0], target[0], self.max_accel_xy * dt),
            self._step_toward(current[1], target[1], self.max_accel_xy * dt),
            self._step_toward(current[2], target[2], self.max_accel_w * dt),
        )

    @staticmethod
    def _step_toward(current: float, target: float, max_delta: float) -> float:
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        return current + math.copysign(max_delta, delta)

    @staticmethod
    def _is_zero(cmd: tuple[float, float, float]) -> bool:
        return abs(cmd[0]) < 1e-4 and abs(cmd[1]) < 1e-4 and abs(cmd[2]) < 1e-4

    def _send_stop(self, reason: str, force: bool = False) -> bool:
        with self._control_state_lock:
            if self._stopped and not force:
                return True
            try:
                self.client.stop(force_reconnect=force)
            except Exception as exc:
                self._warn_limited(f"2000 stop failed: {exc}")
                self._stopped = False
                return False
            self._sent_cmd = (0.0, 0.0, 0.0)
            self._stopped = True
            self.get_logger().info(f"sent 2000 stop: {reason}")
            return True

    def _handle_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._send_stop("ROS stop service", force=True):
            with self._state_lock:
                self._target_cmd = (0.0, 0.0, 0.0)
            self._sent_cmd = (0.0, 0.0, 0.0)
            self._stopped = True
            response.success = True
            response.message = "2000 stop acknowledged"
        else:
            response.success = False
            response.message = "2000 stop was not acknowledged"
        return response

    def _handle_confirm_localization(
        self,
        request: ConfirmLocalization.Request,
        response: ConfirmLocalization.Response,
    ) -> ConfirmLocalization.Response:
        """Send API 2003 only after a fresh, stopped, operator-checked snapshot."""
        now = time.monotonic()
        with self._state_lock:
            safety = self._last_status
            fast = dict(self._fast_status or {})
            loc_status = dict(self._slow_status.get("loc_status", {}))
            fresh = (
                safety is not None
                and now - self._last_fast_status_time <= self.fast_status_timeout_sec
                and self._slow_status_is_fresh(now)
            )

        actual_x = numeric(fast.get("x"), float("nan"))
        actual_y = numeric(fast.get("y"), float("nan"))
        actual_yaw = numeric(
            fast.get("angle"), numeric(fast.get("yaw"), float("nan"))
        )
        response.actual_x = actual_x
        response.actual_y = actual_y
        response.actual_yaw = actual_yaw
        reloc_value = loc_status.get("reloc_status")
        response.reloc_status = (
            int(reloc_value) if isinstance(reloc_value, (int, float)) else -1
        )

        def reject(message: str) -> ConfirmLocalization.Response:
            response.success = False
            response.message = message
            return response

        if not request.operator_confirmed:
            return reject("operator_confirmed must be true")
        expected = (request.expected_x, request.expected_y, request.expected_yaw)
        actual = (actual_x, actual_y, actual_yaw)
        if not all(math.isfinite(value) for value in expected + actual):
            return reject("pose snapshot contains a non-finite value")
        if not fresh or safety is None:
            return reject("status incomplete or stale")
        if loc_status.get("ret_code", 0) not in (0, None):
            return reject("localization status reports an error")
        if not safety.localization_pending_confirmation or response.reloc_status != 3:
            return reject(
                f"localization is not awaiting confirmation (reloc_status={response.reloc_status})"
            )
        unsafe_reasons = []
        if not safety.map_loaded:
            unsafe_reasons.append("map not loaded")
        if safety.emergency_active:
            unsafe_reasons.append("emergency stop active")
        if safety.charging:
            unsafe_reasons.append("robot charging")
        if safety.control_locked:
            unsafe_reasons.append("control owned by another client")
        if safety.blocked:
            unsafe_reasons.append("robot blocked")
        if safety.slowed:
            unsafe_reasons.append("robot slowed")
        if safety.has_alarm:
            unsafe_reasons.append("active alarm")
        if safety.nav_active:
            unsafe_reasons.append("navigation task active")
        if unsafe_reasons:
            return reject(", ".join(unsafe_reasons))

        vx = numeric(fast.get("vx"), float("nan"))
        vy = numeric(fast.get("vy"), float("nan"))
        angular_speed = numeric(fast.get("w"), float("nan"))
        if not all(math.isfinite(value) for value in (vx, vy, angular_speed)):
            return reject("AGV speed status is unavailable")
        if max(abs(vx), abs(vy)) > 0.005 or abs(angular_speed) > 0.01:
            return reject(
                f"AGV is not stopped (vx={vx:.4f}, vy={vy:.4f}, w={angular_speed:.4f})"
            )

        position_error = math.hypot(
            actual_x - request.expected_x, actual_y - request.expected_y
        )
        yaw_error = abs(
            math.atan2(
                math.sin(actual_yaw - request.expected_yaw),
                math.cos(actual_yaw - request.expected_yaw),
            )
        )
        if position_error > 0.05 or yaw_error > math.radians(5.0):
            return reject(
                "pose changed since operator confirmation: "
                f"position_error={position_error:.3f} m, "
                f"yaw_error={math.degrees(yaw_error):.2f} deg"
            )

        with self._state_lock:
            self._target_cmd = (0.0, 0.0, 0.0)
        if not self._send_stop("before localization confirmation", force=True):
            return reject("failed to acknowledge 2000 stop before API 2003")

        try:
            body = self.client.confirm_localization()
        except Exception as exc:
            return reject(f"API 2003 failed: {exc}")

        verified_status = loc_status
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                verified_status = self.client.get_loc_status()
            except Exception as exc:
                return reject(f"API 2003 acknowledged but status verification failed: {exc}")
            reloc_value = verified_status.get("reloc_status")
            response.reloc_status = (
                int(reloc_value) if isinstance(reloc_value, (int, float)) else -1
            )
            with self._state_lock:
                self._slow_status["loc_status"] = verified_status
                self._slow_status_times["loc_status"] = time.monotonic()
            if response.reloc_status == 1:
                self._refresh_safety_state()
                response.success = True
                response.message = (
                    "API 2003 acknowledged; reloc_status=1; "
                    + json.dumps(body, ensure_ascii=False)
                )
                self.get_logger().warn(
                    "operator-confirmed localization accepted: "
                    f"x={actual_x:.3f}, y={actual_y:.3f}, yaw={actual_yaw:.3f}"
                )
                return response
            time.sleep(0.2)

        self._refresh_safety_state()
        return reject(
            "API 2003 was acknowledged but reloc_status did not become 1 within 3 seconds"
        )

    def _stationary_operation_state(
        self, *, require_map: bool
    ) -> tuple[SafetyState | None, dict[str, Any], str]:
        """Return a fresh stopped snapshot for map/localization operations."""
        now = time.monotonic()
        with self._state_lock:
            safety = self._last_status
            fast = dict(self._fast_status or {})
            loc_status = dict(self._slow_status.get("loc_status", {}))
            fresh = (
                safety is not None
                and now - self._last_fast_status_time <= self.fast_status_timeout_sec
                and self._slow_status_is_fresh(now)
            )
        if not fresh or safety is None:
            return safety, fast, "status incomplete or stale"

        reasons = []
        if require_map and not safety.map_loaded:
            reasons.append("map not loaded")
        if safety.emergency_active:
            reasons.append("emergency stop active")
        if safety.charging:
            reasons.append("robot charging")
        if safety.control_locked:
            reasons.append("control owned by another client")
        if safety.has_alarm:
            reasons.append("active alarm")
        if safety.nav_active:
            reasons.append("navigation task active")
        if loc_status.get("reloc_status") == 2:
            reasons.append("relocalization already in progress")

        speeds = tuple(
            numeric(fast.get(key), float("nan")) for key in ("vx", "vy", "w")
        )
        if not all(math.isfinite(value) for value in speeds):
            reasons.append("AGV speed status is unavailable")
        elif max(abs(speeds[0]), abs(speeds[1])) > 0.005 or abs(speeds[2]) > 0.01:
            reasons.append(
                f"AGV is not stopped (vx={speeds[0]:.4f}, "
                f"vy={speeds[1]:.4f}, w={speeds[2]:.4f})"
            )
        return safety, fast, ", ".join(reasons)

    def _handle_load_map(
        self,
        request: LoadMap.Request,
        response: LoadMap.Response,
    ) -> LoadMap.Response:
        name = request.map_name.strip()
        response.current_map = self._current_map
        if not request.operator_confirmed:
            response.success = False
            response.message = "operator_confirmed must be true"
            return response
        unused_safety, unused_fast, gate_error = self._stationary_operation_state(
            require_map=False
        )
        if gate_error:
            response.success = False
            response.message = gate_error
            return response

        try:
            catalog = self.client.get_map_list()
            maps = catalog.get("maps", [])
            if not isinstance(maps, list):
                maps = []
            available = [str(item) for item in maps]
            current = str(catalog.get("current_map", ""))
            if name not in available:
                raise RuntimeError(
                    f"map {name!r} is not in the controller map list"
                )
            if name != current:
                with self._state_lock:
                    self._target_cmd = (0.0, 0.0, 0.0)
                if not self._send_stop("before loading map", force=True):
                    raise RuntimeError("failed to acknowledge 2000 stop before 2022")
                self.client.load_map(name)

                deadline = time.monotonic() + 15.0
                while time.monotonic() < deadline:
                    map_status = self.client.get_map_status()
                    catalog = self.client.get_map_list()
                    current = str(catalog.get("current_map", ""))
                    with self._state_lock:
                        self._slow_status["map_status"] = map_status
                        self._slow_status_times["map_status"] = time.monotonic()
                    if map_status.get("loadmap_status") == 1 and current == name:
                        break
                    time.sleep(0.25)
                else:
                    raise RuntimeError(
                        "API 2022 was acknowledged but the selected map did not finish loading"
                    )

            count = self._download_and_publish_map(name)
            response.current_map = name
            response.success = True
            response.message = f"map {name!r} loaded; published {count} markers"
            self._refresh_safety_state()
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _handle_relocalize(
        self,
        request: Relocalize.Request,
        response: Relocalize.Response,
    ) -> Relocalize.Response:
        response.reloc_status = -1
        if not request.operator_confirmed:
            response.success = False
            response.message = "operator_confirmed must be true"
            return response
        values = (request.x, request.y, request.yaw, request.radius)
        if not all(math.isfinite(value) for value in values):
            response.success = False
            response.message = "relocalization values must be finite"
            return response
        unused_safety, unused_fast, gate_error = self._stationary_operation_state(
            require_map=True
        )
        if gate_error:
            response.success = False
            response.message = gate_error
            return response

        with self._state_lock:
            self._target_cmd = (0.0, 0.0, 0.0)
        if not self._send_stop("before relocalization", force=True):
            response.success = False
            response.message = "failed to acknowledge 2000 stop before API 2002"
            return response
        try:
            body = self.client.relocalize(
                request.x, request.y, request.yaw, request.radius
            )
            deadline = time.monotonic() + 2.0
            last_status: dict[str, Any] = {}
            while time.monotonic() < deadline:
                last_status = self.client.get_loc_status()
                reloc_value = last_status.get("reloc_status")
                response.reloc_status = (
                    int(reloc_value)
                    if isinstance(reloc_value, (int, float))
                    else -1
                )
                with self._state_lock:
                    self._slow_status["loc_status"] = last_status
                    self._slow_status_times["loc_status"] = time.monotonic()
                if response.reloc_status in (0, 2, 3):
                    break
                time.sleep(0.2)
            self._refresh_safety_state()
            response.success = True
            response.message = (
                "API 2002 acknowledged; confirmation will still be required after "
                "relocation; " + json.dumps(body, ensure_ascii=False)
            )
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _handle_cancel_relocalization(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        try:
            body = self.client.cancel_relocalization()
            stopped = self._send_stop("relocalization canceled", force=True)
            if not stopped:
                raise RuntimeError(
                    "cancel relocation acknowledged but stop was not acknowledged"
                )
            response.success = True
            response.message = json.dumps(body, ensure_ascii=False)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _handle_cancel_nav(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        try:
            body = self.client.cancel_nav()
            stopped = self._send_stop(
                "navigation canceled", force=True
            )
            if not stopped:
                raise RuntimeError("cancel acknowledged but stop was not acknowledged")
            response.success = True
            response.message = json.dumps(body, ensure_ascii=False)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _handle_navigate_to_station(
        self,
        request: NavigateToStation.Request,
        response: NavigateToStation.Response,
    ) -> NavigateToStation.Response:
        station_id = request.station_id.strip()
        if not station_id:
            response.success = False
            response.message = "station_id must not be empty"
            return response
        try:
            travel_mode = normalize_travel_mode(request.travel_mode)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response
        speed = float(request.max_speed) if request.max_speed > 0.0 else 0.08
        if not 0.01 <= speed <= self.max_navigation_speed:
            response.success = False
            response.message = (
                f"max_speed must be 0.01..{self.max_navigation_speed:.2f} m/s"
            )
            return response
        target_yaw = None
        if request.use_target_yaw:
            target_yaw = float(request.target_yaw)
            if not math.isfinite(target_yaw):
                response.success = False
                response.message = "target_yaw must be finite"
                return response

        now = time.monotonic()
        with self._state_lock:
            safety = self._last_status
            fresh = (
                safety is not None
                and now - self._last_fast_status_time <= self.fast_status_timeout_sec
                and self._slow_status_is_fresh(now)
            )
        if not fresh or safety is None:
            response.success = False
            response.message = "status incomplete or stale"
            return response
        if not safety.safe_to_start_navigation:
            response.success = False
            response.message = safety.teleop_reason or "navigation inhibited"
            return response

        with self._state_lock:
            self._target_cmd = (0.0, 0.0, 0.0)
        if not self._send_stop(
            "switching from teleop to navigation", force=True
        ):
            response.success = False
            response.message = "failed to confirm stop before navigation"
            return response

        task_id = f"gui_{station_id}_{uuid.uuid4().hex}"
        try:
            body = self.client.goto_station(
                station_id,
                task_id=task_id,
                max_speed=speed,
                max_wspeed=0.2,
                max_acc=0.1,
                max_wacc=0.1,
                target_yaw=target_yaw,
                travel_mode=travel_mode,
            )
            response.success = True
            response.task_id = task_id
            response.message = json.dumps(body, ensure_ascii=False)
        except Exception as exc:
            response.success = False
            response.task_id = task_id
            response.message = str(exc)
        return response

    def _handle_navigate_to_pose(
        self,
        request: NavigateToPose.Request,
        response: NavigateToPose.Response,
    ) -> NavigateToPose.Response:
        waypoint_name = request.waypoint_name.strip()
        values = (
            float(request.x),
            float(request.y),
            float(request.yaw),
            float(request.max_speed),
        )
        if not waypoint_name:
            response.success = False
            response.message = "waypoint_name must not be empty"
            return response
        try:
            travel_mode = normalize_travel_mode(request.travel_mode)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response
        if not all(math.isfinite(value) for value in values):
            response.success = False
            response.message = "waypoint pose and speed must be finite"
            return response
        speed = values[3] if values[3] > 0.0 else 0.08
        if not 0.01 <= speed <= self.max_navigation_speed:
            response.success = False
            response.message = (
                f"max_speed must be 0.01..{self.max_navigation_speed:.2f} m/s"
            )
            return response

        now = time.monotonic()
        with self._state_lock:
            safety = self._last_status
            fresh = (
                safety is not None
                and now - self._last_fast_status_time
                <= self.fast_status_timeout_sec
                and self._slow_status_is_fresh(now)
            )
        if not fresh or safety is None:
            response.success = False
            response.message = "status incomplete or stale"
            return response
        if not safety.safe_to_start_navigation:
            response.success = False
            response.message = safety.teleop_reason or "navigation inhibited"
            return response

        with self._state_lock:
            self._target_cmd = (0.0, 0.0, 0.0)
        if not self._send_stop(
            "switching from teleop to pose navigation", force=True
        ):
            response.success = False
            response.message = "failed to confirm stop before navigation"
            return response

        task_id = f"gui_pose_{uuid.uuid4().hex}"
        try:
            body = self.client.goto_pose(
                values[0],
                values[1],
                values[2],
                task_id=task_id,
                max_speed=speed,
                travel_mode=travel_mode,
            )
            response.success = True
            response.task_id = task_id
            response.message = json.dumps(body, ensure_ascii=False)
        except Exception as exc:
            response.success = False
            response.task_id = task_id
            response.message = str(exc)
        return response

    def _handle_plan_to_station(
        self,
        request: PlanToStation.Request,
        response: PlanToStation.Response,
    ) -> PlanToStation.Response:
        target = request.target_station_id.strip()
        if not target:
            response.success = False
            response.message = "target station ID must not be empty"
            return response
        try:
            station_ids = self.client.get_path_to_station(target)
            response.success = True
            response.station_ids = station_ids
            response.message = " -> ".join(station_ids)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _handle_download_map(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        try:
            count = self._download_and_publish_map()
            response.success = True
            response.message = f"published {count} map markers"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _download_map_once(self) -> None:
        if self._map_loaded_once:
            return
        try:
            count = self._download_and_publish_map()
            self._map_loaded_once = True
            self.get_logger().info(f"downloaded current map and published {count} markers")
        except Exception as exc:
            self._warn_limited(f"initial map download failed: {exc}")

    def _download_and_publish_map(self, requested_map: str = "") -> int:
        catalog = self.client.get_map_list()
        stored = catalog.get("maps", [])
        if not isinstance(stored, list):
            stored = []
        map_name = requested_map.strip()
        if not map_name:
            map_name = str(self.get_parameter("map_name").value).strip()
        if not map_name:
            map_name = str(catalog.get("current_map", ""))
        if not map_name:
            info = self.client.get_info()
            map_name = str(info.get("current_map", ""))
        if not map_name:
            raise RuntimeError("current map name is empty")
        map_data = self.client.download_map(map_name)
        stations = self.client.get_stations()
        markers = map_to_markers(map_data, stations, self.frame_id, self.footprint)
        gui_data = map_to_gui_data(
            map_data,
            stations,
            map_name,
            [str(item) for item in stored],
            self.footprint,
        )
        map_data_message = String()
        map_data_message.data = json.dumps(
            gui_data, ensure_ascii=False, separators=(",", ":")
        )
        with self._state_lock:
            self._current_map = map_name
            self._stored_maps = [str(item) for item in stored]
        self._map_markers = markers
        self._map_data_message = map_data_message
        self.map_pub.publish(markers)
        self.map_data_pub.publish(map_data_message)
        station_message = String()
        station_message.data = json.dumps(
            stations.get("stations", []),
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.stations_pub.publish(station_message)
        return len(markers.markers)

    def _republish_map(self) -> None:
        if self._map_markers is not None:
            self.map_pub.publish(self._map_markers)

    def _warn_limited(self, text: str) -> None:
        now = time.monotonic()
        last = getattr(self, "_last_warn_time", 0.0)
        if now - last > 2.0:
            self.get_logger().warn(text)
            self._last_warn_time = now


def map_to_markers(
    map_data: dict[str, Any],
    stations: dict[str, Any],
    frame_id: str,
    footprint: Footprint = DEFAULT_FOOTPRINT,
) -> MarkerArray:
    markers = MarkerArray()
    delete = Marker()
    delete.header.frame_id = frame_id
    delete.action = Marker.DELETEALL
    markers.markers.append(delete)

    marker_id = 1
    marker_id = _add_normal_points(markers, marker_id, map_data.get("normalPosList", []), frame_id)
    marker_id = _add_feature_lines(markers, marker_id, map_data.get("advancedLineList", []), frame_id)
    marker_id = _add_curves(markers, marker_id, map_data.get("advancedCurveList", []), frame_id)
    marker_id = _add_advanced_points(markers, marker_id, map_data.get("advancedPointList", []), frame_id)
    _add_stations(markers, marker_id, stations.get("stations", []), frame_id, footprint)
    return markers


def map_to_gui_data(
    map_data: dict[str, Any],
    stations: dict[str, Any],
    current_map: str,
    stored_maps: list[str],
    footprint: Footprint = DEFAULT_FOOTPRINT,
) -> dict[str, Any]:
    """Create a compact, toolkit-neutral 2-D map snapshot for the Qt GUI."""
    normal_points = []
    for item in map_data.get("normalPosList", []):
        if (
            isinstance(item, dict)
            and isinstance(item.get("x"), (int, float))
            and isinstance(item.get("y"), (int, float))
        ):
            normal_points.append([float(item["x"]), float(item["y"])])

    feature_lines = []
    for item in map_data.get("advancedLineList", []):
        line = item.get("line") if isinstance(item, dict) else None
        if not isinstance(line, dict):
            continue
        start = _extract_pos(line.get("startPos"))
        end = _extract_pos(line.get("endPos"))
        if start and end:
            feature_lines.append([list(start), list(end)])

    curves = []
    for item in map_data.get("advancedCurveList", []):
        if not isinstance(item, dict):
            continue
        start = _extract_pos(item.get("startPos"))
        end = _extract_pos(item.get("endPos"))
        control1 = _extract_pos(item.get("controlPos1"))
        control2 = _extract_pos(item.get("controlPos2"))
        if not all((start, end, control1, control2)):
            continue
        curves.append(
            [
                list(_cubic_bezier(start, control1, control2, end, index / 24.0))
                for index in range(25)
            ]
        )

    advanced_points = []
    for item in map_data.get("advancedPointList", []):
        if not isinstance(item, dict):
            continue
        pos = _extract_pos(item.get("pos"))
        if pos:
            advanced_points.append(
                {"name": str(item.get("instanceName", "")), "x": pos[0], "y": pos[1]}
            )

    station_data = []
    for item in stations.get("stations", []):
        if (
            not isinstance(item, dict)
            or not isinstance(item.get("x"), (int, float))
            or not isinstance(item.get("y"), (int, float))
        ):
            continue
        station_data.append(
            {
                "id": str(item.get("id", "")),
                "x": float(item["x"]),
                "y": float(item["y"]),
                "yaw": numeric(item.get("r", item.get("angle", 0.0))),
            }
        )

    return {
        "current_map": current_map,
        "maps": stored_maps,
        "normal_points": normal_points,
        "feature_lines": feature_lines,
        "curves": curves,
        "advanced_points": advanced_points,
        "stations": station_data,
        "footprint": {
            "width": footprint.width,
            "head": footprint.head,
            "tail": footprint.tail,
        },
    }


def _base_marker(marker_id: int, ns: str, marker_type: int, frame_id: str) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def _add_normal_points(markers: MarkerArray, marker_id: int, points: list[Any], frame_id: str) -> int:
    marker = _base_marker(marker_id, "normal_points", Marker.POINTS, frame_id)
    marker.scale.x = 0.04
    marker.scale.y = 0.04
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 0.75
    for item in points:
        if isinstance(item, dict) and isinstance(item.get("x"), (int, float)) and isinstance(item.get("y"), (int, float)):
            marker.points.append(Point(x=float(item["x"]), y=float(item["y"]), z=0.0))
    if marker.points:
        markers.markers.append(marker)
        marker_id += 1
    return marker_id


def _add_feature_lines(markers: MarkerArray, marker_id: int, lines: list[Any], frame_id: str) -> int:
    marker = _base_marker(marker_id, "feature_lines", Marker.LINE_LIST, frame_id)
    marker.scale.x = 0.06
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 0.95
    for item in lines:
        line = item.get("line") if isinstance(item, dict) else None
        if not isinstance(line, dict):
            continue
        start = line.get("startPos")
        end = line.get("endPos")
        if isinstance(start, dict) and isinstance(end, dict):
            marker.points.append(Point(x=numeric(start.get("x")), y=numeric(start.get("y")), z=0.02))
            marker.points.append(Point(x=numeric(end.get("x")), y=numeric(end.get("y")), z=0.02))
    if marker.points:
        markers.markers.append(marker)
        marker_id += 1
    return marker_id


def _add_curves(markers: MarkerArray, marker_id: int, curves: list[Any], frame_id: str) -> int:
    for item in curves:
        if not isinstance(item, dict):
            continue
        start = _extract_pos(item.get("startPos"))
        end = _extract_pos(item.get("endPos"))
        c1 = _extract_pos(item.get("controlPos1"))
        c2 = _extract_pos(item.get("controlPos2"))
        if not all((start, end, c1, c2)):
            continue
        marker = _base_marker(marker_id, "nav_curves", Marker.LINE_STRIP, frame_id)
        marker.scale.x = 0.05
        marker.color.r = 0.95
        marker.color.g = 0.45
        marker.color.b = 0.08
        marker.color.a = 0.95
        for i in range(25):
            t = i / 24.0
            x, y = _cubic_bezier(start, c1, c2, end, t)
            marker.points.append(Point(x=x, y=y, z=0.04))
        markers.markers.append(marker)
        marker_id += 1
    return marker_id


def _add_advanced_points(markers: MarkerArray, marker_id: int, points: list[Any], frame_id: str) -> int:
    for item in points:
        if not isinstance(item, dict):
            continue
        pos = _extract_pos(item.get("pos"))
        if not pos:
            continue
        label = str(item.get("instanceName", ""))
        sphere = _base_marker(marker_id, "advanced_points", Marker.SPHERE, frame_id)
        sphere.pose.position.x = pos[0]
        sphere.pose.position.y = pos[1]
        sphere.pose.position.z = 0.08
        sphere.scale = Vector3(x=0.18, y=0.18, z=0.18)
        sphere.color.r = 0.0
        sphere.color.g = 0.45
        sphere.color.b = 0.38
        sphere.color.a = 0.9
        markers.markers.append(sphere)
        marker_id += 1
        marker_id = _add_text(markers, marker_id, "advanced_point_labels", label, pos[0], pos[1], frame_id)
    return marker_id


def _add_stations(
    markers: MarkerArray,
    marker_id: int,
    stations: list[Any],
    frame_id: str,
    footprint: Footprint,
) -> int:
    for station in stations:
        if not isinstance(station, dict):
            continue
        if not isinstance(station.get("x"), (int, float)) or not isinstance(station.get("y"), (int, float)):
            continue
        x = float(station["x"])
        y = float(station["y"])
        yaw = numeric(station.get("r", station.get("angle", 0.0)))
        marker_id = _add_footprint(
            markers,
            marker_id,
            "station_footprints",
            x,
            y,
            yaw,
            footprint,
            frame_id,
            fill=(0.95, 0.48, 0.16, 0.22),
            line=(1.0, 0.62, 0.22, 0.92),
            z=0.03,
            height_scale=0.03,
        )
        marker = _base_marker(marker_id, "stations", Marker.CYLINDER, frame_id)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.06
        marker.scale = Vector3(x=0.16, y=0.16, z=0.12)
        marker.color.r = 0.05
        marker.color.g = 0.65
        marker.color.b = 0.35
        marker.color.a = 0.95
        markers.markers.append(marker)
        marker_id += 1
        marker_id = _add_text(markers, marker_id, "station_labels", str(station.get("id", "")), x, y, frame_id)
    return marker_id


def _add_footprint(
    markers: MarkerArray,
    marker_id: int,
    ns: str,
    x: float,
    y: float,
    yaw: float,
    footprint: Footprint,
    frame_id: str,
    fill: tuple[float, float, float, float],
    line: tuple[float, float, float, float],
    z: float,
    height_scale: float,
    stamp: Any | None = None,
) -> int:
    fill_marker = _base_marker(marker_id, f"{ns}_fill", Marker.CUBE, frame_id)
    if stamp is not None:
        fill_marker.header.stamp = stamp
    center_offset = (footprint.head - footprint.tail) * 0.5
    fill_marker.pose.position.x = x + center_offset * math.cos(yaw)
    fill_marker.pose.position.y = y + center_offset * math.sin(yaw)
    fill_marker.pose.position.z = z
    fill_marker.pose.orientation = yaw_to_quaternion(yaw)
    fill_marker.scale = Vector3(x=footprint.length, y=footprint.width, z=height_scale)
    _set_color(fill_marker, fill)
    markers.markers.append(fill_marker)
    marker_id += 1

    edge_marker = _base_marker(marker_id, f"{ns}_edge", Marker.LINE_STRIP, frame_id)
    if stamp is not None:
        edge_marker.header.stamp = stamp
    edge_marker.scale.x = 0.045
    _set_color(edge_marker, line)
    points = _footprint_points(x, y, yaw, footprint, z + height_scale * 0.7)
    for px, py, pz in points + [points[0]]:
        edge_marker.points.append(Point(x=px, y=py, z=pz))
    markers.markers.append(edge_marker)
    return marker_id + 1


def _add_heading_line(
    markers: MarkerArray,
    marker_id: int,
    ns: str,
    x: float,
    y: float,
    yaw: float,
    footprint: Footprint,
    frame_id: str,
    stamp: Any,
) -> int:
    marker = _base_marker(marker_id, ns, Marker.LINE_LIST, frame_id)
    marker.header.stamp = stamp
    marker.scale.x = 0.055
    marker.color.r = 1.0
    marker.color.g = 0.05
    marker.color.b = 0.02
    marker.color.a = 1.0
    marker.points.append(Point(x=x, y=y, z=0.16))
    marker.points.append(
        Point(
            x=x + footprint.head * math.cos(yaw),
            y=y + footprint.head * math.sin(yaw),
            z=0.16,
        )
    )
    markers.markers.append(marker)
    return marker_id + 1


def _footprint_points(x: float, y: float, yaw: float, footprint: Footprint, z: float) -> list[tuple[float, float, float]]:
    half_width = footprint.width * 0.5
    local_points = [
        (footprint.head, half_width),
        (footprint.head, -half_width),
        (-footprint.tail, -half_width),
        (-footprint.tail, half_width),
    ]
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return [
        (
            x + lx * cos_yaw - ly * sin_yaw,
            y + lx * sin_yaw + ly * cos_yaw,
            z,
        )
        for lx, ly in local_points
    ]


def _set_color(marker: Marker, rgba: tuple[float, float, float, float]) -> None:
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]


def _add_text(markers: MarkerArray, marker_id: int, ns: str, text: str, x: float, y: float, frame_id: str) -> int:
    marker = _base_marker(marker_id, ns, Marker.TEXT_VIEW_FACING, frame_id)
    marker.pose.position.x = x + 0.12
    marker.pose.position.y = y + 0.12
    marker.pose.position.z = 0.25
    marker.scale.z = 0.22
    marker.color.r = 0.02
    marker.color.g = 0.08
    marker.color.b = 0.16
    marker.color.a = 1.0
    marker.text = text
    markers.markers.append(marker)
    return marker_id + 1


def _extract_pos(value: Any) -> tuple[float, float] | None:
    if isinstance(value, dict) and isinstance(value.get("pos"), dict):
        value = value["pos"]
    if isinstance(value, dict) and isinstance(value.get("x"), (int, float)) and isinstance(value.get("y"), (int, float)):
        return float(value["x"]), float(value["y"])
    return None


def _cubic_bezier(
    start: tuple[float, float],
    c1: tuple[float, float],
    c2: tuple[float, float],
    end: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    u = 1.0 - t
    x = u**3 * start[0] + 3 * u**2 * t * c1[0] + 3 * u * t**2 * c2[0] + t**3 * end[0]
    y = u**3 * start[1] + 3 * u**2 * t * c1[1] + 3 * u * t**2 * c2[1] + t**3 * end[1]
    return x, y


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SeerAgvNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.enable_cmd_vel:
            node._send_stop("node shutting down", force=True)
        executor.shutdown(timeout_sec=3.0)
        node.client.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
