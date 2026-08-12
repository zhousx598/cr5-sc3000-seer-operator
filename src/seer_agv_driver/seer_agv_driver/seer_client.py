import json
import math
import socket
import struct
import threading
from dataclasses import dataclass
from typing import Any


HEADER = b"\x5A\x01"
DEFAULT_HOST = "192.168.192.5"
MAX_RESPONSE_BODY_BYTES = 32 * 1024 * 1024

PORT_STATUS = 19204
PORT_CONTROL = 19205
PORT_NAV = 19206
PORT_CONFIG = 19207


@dataclass(frozen=True)
class ApiSpec:
    port: int
    request: int
    response: int
    default_body: dict[str, Any] | None = None


# API numbers were verified against the complete SEER/Robokit protocol
# documents supplied with the original project handoff.
API_INFO = ApiSpec(PORT_STATUS, 1000, 11000)
API_POSE = ApiSpec(PORT_STATUS, 1004, 11004)
API_SPEED = ApiSpec(PORT_STATUS, 1005, 11005)
API_BLOCKED = ApiSpec(PORT_STATUS, 1006, 11006)
API_BATTERY = ApiSpec(PORT_STATUS, 1007, 11007)
API_EMERGENCY = ApiSpec(PORT_STATUS, 1012, 11012)
API_ALARM = ApiSpec(PORT_STATUS, 1050, 11050)
API_CONTROL_OWNER = ApiSpec(PORT_STATUS, 1060, 11060)
API_ALL2 = ApiSpec(PORT_STATUS, 1101, 11101, {"return_laser": False})
API_NAV_STATUS = ApiSpec(PORT_STATUS, 1020, 11020, {"simple": True})
API_LOC_STATUS = ApiSpec(PORT_STATUS, 1021, 11021)
API_MAP_STATUS = ApiSpec(PORT_STATUS, 1022, 11022)
API_MAP_LIST = ApiSpec(PORT_STATUS, 1300, 11300)
API_STATIONS = ApiSpec(PORT_STATUS, 1301, 11301)
API_DOWNLOAD_MODEL = ApiSpec(PORT_STATUS, 1500, 11500)
API_STOP = ApiSpec(PORT_CONTROL, 2000, 12000)
API_RELOCALIZE = ApiSpec(PORT_CONTROL, 2002, 12002)
API_CONFIRM_LOCALIZATION = ApiSpec(PORT_CONTROL, 2003, 12003)
API_CANCEL_RELOCALIZATION = ApiSpec(PORT_CONTROL, 2004, 12004)
API_JOG = ApiSpec(PORT_CONTROL, 2010, 12010)
API_LOAD_MAP = ApiSpec(PORT_CONTROL, 2022, 12022)
API_CANCEL_NAV = ApiSpec(PORT_NAV, 3003, 13003)
API_GOTO_STATION = ApiSpec(PORT_NAV, 3051, 13051)
API_PLAN_TO_STATION = ApiSpec(PORT_NAV, 3053, 13053)
API_DOWNLOAD_MAP = ApiSpec(PORT_CONFIG, 4011, 14011)


class SeerProtocolError(RuntimeError):
    pass


class SeerApiError(RuntimeError):
    pass


class SeerAmbiguousMotionError(SeerProtocolError):
    """The controller may have accepted a motion command whose reply was lost."""


def build_packet(api_type: int, number: int, body: bytes = b"") -> bytes:
    return (
        HEADER
        + struct.pack(">H", number)
        + struct.pack(">I", len(body))
        + struct.pack(">H", api_type)
        + b"\x00" * 6
        + body
    )


def parse_header(header: bytes) -> tuple[int, int, int]:
    if len(header) != 16:
        raise SeerProtocolError(f"SEER header must be 16 bytes, got {len(header)}")
    if header[:2] != HEADER:
        raise SeerProtocolError(f"unexpected SEER header prefix: {header.hex(' ')}")
    packet_number = struct.unpack(">H", header[2:4])[0]
    body_len = struct.unpack(">I", header[4:8])[0]
    api_type = struct.unpack(">H", header[8:10])[0]
    return packet_number, body_len, api_type


def recv_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("socket closed while receiving")
        data.extend(chunk)
    return bytes(data)


@dataclass
class SeerResponse:
    request_type: int
    response_number: int
    response_type: int
    body: Any


@dataclass
class SafetyState:
    localized: bool
    localization_pending_confirmation: bool
    map_loaded: bool
    blocked: bool
    slowed: bool
    has_alarm: bool
    emergency_active: bool
    charging: bool
    control_locked: bool
    nav_status: int | None
    reason: str

    @property
    def safe_to_move(self) -> bool:
        return not self.reason

    @property
    def nav_active(self) -> bool:
        return self.nav_status in (1, 2, 3)

    @property
    def teleop_reason(self) -> str:
        reasons = [self.reason] if self.reason else []
        if self.nav_active:
            reasons.append("navigation task active")
        return ", ".join(reasons)

    @property
    def safe_for_teleop(self) -> bool:
        return not self.teleop_reason

    @property
    def safe_to_start_navigation(self) -> bool:
        return self.safe_to_move and not self.nav_active


@dataclass(frozen=True)
class Footprint:
    width: float
    head: float
    tail: float
    height: float = 0.3

    @property
    def length(self) -> float:
        return self.head + self.tail


class SeerClient:
    def __init__(
        self,
        host: str = DEFAULT_HOST,
        timeout: float = 2.0,
        max_vx: float = 0.1,
        max_vy: float = 0.1,
        max_w: float = 0.2,
        max_duration_ms: int = 300,
    ) -> None:
        self.host = host
        self.timeout = timeout
        self.max_vx = abs(max_vx)
        self.max_vy = abs(max_vy)
        self.max_w = abs(max_w)
        self.max_duration_ms = int(max_duration_ms)
        self._packet_number = 0
        self._packet_lock = threading.Lock()
        self._port_locks = {
            PORT_STATUS: threading.Lock(),
            PORT_CONTROL: threading.Lock(),
            PORT_NAV: threading.Lock(),
            PORT_CONFIG: threading.Lock(),
        }
        self._control_sock: socket.socket | None = None

    def _next_packet_number(self) -> int:
        with self._packet_lock:
            self._packet_number = (self._packet_number + 1) & 0xFFFF
            return self._packet_number or 1

    def request(
        self,
        spec: ApiSpec,
        body_obj: dict[str, Any] | None = None,
        timeout: float | None = None,
    ) -> SeerResponse:
        if spec.port == PORT_CONTROL:
            return self._persistent_control_request(spec, body_obj, timeout)

        body = self._encode_body(body_obj if body_obj is not None else spec.default_body)

        with self._port_locks[spec.port]:
            number = self._next_packet_number()
            with socket.create_connection((self.host, spec.port), timeout=timeout or self.timeout) as sock:
                sock.settimeout(timeout or self.timeout)
                response_number, response_type, parsed_body = self._request_on_socket(sock, spec.request, number, body)

        return self._checked_response(
            spec, number, response_number, response_type, parsed_body
        )

    def _persistent_control_request(
        self,
        spec: ApiSpec,
        body_obj: dict[str, Any] | None = None,
        timeout: float | None = None,
    ) -> SeerResponse:
        body = self._encode_body(body_obj if body_obj is not None else spec.default_body)
        with self._port_locks[PORT_CONTROL]:
            return self._control_request_locked(spec, body, timeout)

    def _control_request_locked(
        self,
        spec: ApiSpec,
        body: bytes,
        timeout: float | None,
    ) -> SeerResponse:
        # Retrying an idempotent stop is safe.  A 2010 jog must never be
        # replayed after a lost response because the robot may already be
        # moving even though the caller observed an exception.
        attempts = 2 if spec.request == API_STOP.request else 1
        last_exc: Exception | None = None
        for attempt in range(attempts):
            try:
                sock = self._get_control_sock(timeout)
                number = self._next_packet_number()
                response_number, response_type, parsed_body = self._request_on_socket(
                    sock, spec.request, number, body
                )
                return self._checked_response(
                    spec,
                    number,
                    response_number,
                    response_type,
                    parsed_body,
                )
            except (OSError, ConnectionError, SeerProtocolError) as exc:
                last_exc = exc
                self._close_control_sock()
                if spec.request == API_JOG.request:
                    raise SeerAmbiguousMotionError(
                        "2010 jog response was lost; command may have been accepted"
                    ) from exc
                if attempt + 1 >= attempts:
                    raise
        assert last_exc is not None
        raise last_exc

    def _get_control_sock(self, timeout: float | None = None) -> socket.socket:
        if self._control_sock is None:
            self._control_sock = socket.create_connection((self.host, PORT_CONTROL), timeout=timeout or self.timeout)
        self._control_sock.settimeout(timeout or self.timeout)
        return self._control_sock

    def _close_control_sock(self) -> None:
        if self._control_sock is not None:
            try:
                self._control_sock.close()
            finally:
                self._control_sock = None

    def close(self) -> None:
        with self._port_locks[PORT_CONTROL]:
            self._close_control_sock()

    def _encode_body(self, body_obj: dict[str, Any] | None) -> bytes:
        if body_obj is None:
            return b""
        return json.dumps(body_obj, ensure_ascii=False, separators=(",", ":")).encode("utf-8")

    def _request_on_socket(
        self,
        sock: socket.socket,
        request_type: int,
        number: int,
        body: bytes,
    ) -> tuple[int, int, Any]:
        sock.sendall(build_packet(request_type, number, body))
        header = recv_exact(sock, 16)
        response_number, body_len, response_type = parse_header(header)
        if body_len > MAX_RESPONSE_BODY_BYTES:
            raise SeerProtocolError(
                f"SEER response body too large: {body_len} bytes"
            )
        response_body = recv_exact(sock, body_len) if body_len else b""

        parsed_body: Any = None
        if response_body:
            text = response_body.decode("utf-8", errors="replace")
            try:
                parsed_body = json.loads(text)
            except json.JSONDecodeError:
                parsed_body = text
        return response_number, response_type, parsed_body

    def _checked_response(
        self,
        spec: ApiSpec,
        request_number: int,
        response_number: int,
        response_type: int,
        parsed_body: Any,
    ) -> SeerResponse:
        if response_number != request_number:
            raise SeerProtocolError(
                f"response packet number mismatch: got {response_number}, "
                f"expected {request_number} for request {spec.request}"
            )
        if response_type != spec.response:
            raise SeerProtocolError(
                f"unexpected response type: got {response_type}, expected {spec.response} "
                f"for request {spec.request}"
            )
        self._raise_for_ret_code(spec.request, parsed_body)
        return SeerResponse(spec.request, response_number, response_type, parsed_body)

    def _raise_for_ret_code(self, request_type: int, body: Any) -> None:
        if isinstance(body, dict) and body.get("ret_code", 0) not in (0, None):
            raise SeerApiError(
                f"SEER API {request_type} ret_code={body.get('ret_code')}: {body.get('err_msg', '')}"
            )

    def get_info(self) -> dict[str, Any]:
        return self.request(API_INFO).body or {}

    def get_pose(self) -> dict[str, Any]:
        return self.request(API_POSE).body or {}

    def get_speed(self) -> dict[str, Any]:
        return self.request(API_SPEED).body or {}

    def get_blocked(self) -> dict[str, Any]:
        return self.request(API_BLOCKED).body or {}

    def get_battery(self) -> dict[str, Any]:
        return self.request(API_BATTERY).body or {}

    def get_emergency(self) -> dict[str, Any]:
        return self.request(API_EMERGENCY).body or {}

    def get_alarm(self) -> dict[str, Any]:
        return self.request(API_ALARM).body or {}

    def get_control_owner(self) -> dict[str, Any]:
        return self.request(API_CONTROL_OWNER).body or {}

    def get_all2(self) -> dict[str, Any]:
        return self.request(API_ALL2).body or {}

    def get_nav_status(self) -> dict[str, Any]:
        return self.request(API_NAV_STATUS).body or {}

    def get_loc_status(self) -> dict[str, Any]:
        return self.request(API_LOC_STATUS).body or {}

    def get_map_status(self) -> dict[str, Any]:
        return self.request(API_MAP_STATUS).body or {}

    def get_map_list(self) -> dict[str, Any]:
        return self.request(API_MAP_LIST).body or {}

    def get_stations(self) -> dict[str, Any]:
        return self.request(API_STATIONS).body or {}

    def get_path_to_station(
        self, target_station_id: str
    ) -> list[str]:
        target = target_station_id.strip()
        if not target:
            raise SeerApiError('target station ID must not be empty')
        body = self.request(
            API_PLAN_TO_STATION,
            {'id': target},
        ).body or {}
        path = body.get('path', []) if isinstance(body, dict) else []
        if not isinstance(path, list) or not all(
            isinstance(value, str) and value for value in path
        ):
            raise SeerApiError(
                'SEER API 3053 returned an invalid station path'
            )
        return path

    def download_model(self) -> dict[str, Any]:
        return self.request(API_DOWNLOAD_MODEL, timeout=max(10.0, self.timeout)).body or {}

    def get_footprint(self) -> Footprint:
        return parse_footprint(self.download_model())

    def stop(self, force_reconnect: bool = False) -> dict[str, Any]:
        if force_reconnect:
            with self._port_locks[PORT_CONTROL]:
                self._close_control_sock()
                return self._control_request_locked(
                    API_STOP, b"", None
                ).body or {}
        return self.request(API_STOP).body or {}

    def confirm_localization(self) -> dict[str, Any]:
        """Confirm an operator-verified relocation on Robokit < 3.4.6.18."""
        return self.request(API_CONFIRM_LOCALIZATION).body or {}

    def relocalize(
        self, x: float, y: float, yaw: float, radius: float
    ) -> dict[str, Any]:
        values = (float(x), float(y), float(yaw), float(radius))
        if not all(math.isfinite(value) for value in values):
            raise SeerApiError("relocalization values must be finite")
        if not 0.05 <= values[3] <= 5.0:
            raise SeerApiError("relocalization radius must be 0.05..5.0 m")
        return self.request(
            API_RELOCALIZE,
            {
                "x": values[0],
                "y": values[1],
                "angle": values[2],
                "length": values[3],
            },
        ).body or {}

    def cancel_relocalization(self) -> dict[str, Any]:
        return self.request(API_CANCEL_RELOCALIZATION).body or {}

    def load_map(self, map_name: str) -> dict[str, Any]:
        name = map_name.strip()
        if not name or len(name) > 255:
            raise SeerApiError("map_name must not be empty or longer than 255 chars")
        if any(char not in "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ-_." for char in name):
            raise SeerApiError("map_name contains unsupported characters")
        if name in {".", ".."}:
            raise SeerApiError("invalid map_name")
        return self.request(API_LOAD_MAP, {"map_name": name}).body or {}

    def cancel_nav(self) -> dict[str, Any]:
        return self.request(API_CANCEL_NAV).body or {}

    def goto_station(
        self,
        station_id: str,
        source_id: str = "SELF_POSITION",
        task_id: str | None = None,
        max_speed: float = 0.08,
        max_wspeed: float = 0.2,
        max_acc: float = 0.1,
        max_wacc: float = 0.1,
        target_yaw: float | None = None,
    ) -> dict[str, Any]:
        body: dict[str, Any] = {
            "source_id": source_id,
            "id": station_id,
            "max_speed": max(0.01, min(float(max_speed), 0.2)),
            "max_wspeed": max(0.01, min(float(max_wspeed), 0.3)),
            "max_acc": max(0.01, min(float(max_acc), 0.2)),
            "max_wacc": max(0.01, min(float(max_wacc), 0.3)),
        }
        if task_id:
            body["task_id"] = task_id
        if target_yaw is not None:
            yaw = float(target_yaw)
            if not math.isfinite(yaw):
                raise SeerApiError('target_yaw must be finite')
            body['angle'] = math.atan2(math.sin(yaw), math.cos(yaw))
        return self.request(
            API_GOTO_STATION, body, timeout=max(5.0, self.timeout)
        ).body or {}

    def goto_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        task_id: str,
        max_speed: float = 0.08,
    ) -> dict[str, Any]:
        """Navigate to a world pose with Robokit's documented goPath script."""
        values = (float(x), float(y), float(yaw), float(max_speed))
        if not all(math.isfinite(value) for value in values):
            raise SeerApiError('world pose and speed must be finite')
        if not task_id.strip():
            raise SeerApiError('task_id must not be empty')
        normalized_yaw = math.atan2(math.sin(values[2]), math.cos(values[2]))
        body = {
            'script_name': 'syspy/goPath.py',
            'script_args': {
                'x': values[0],
                'y': values[1],
                'theta': normalized_yaw,
                'coordinate': 'world',
                'reachAngle': math.radians(3.0),
                'reachDist': 0.03,
                'backMode': 0,
                'useOdo': 0,
            },
            'operation': 'Script',
            'id': 'SELF_POSITION',
            'source_id': 'SELF_POSITION',
            'task_id': task_id.strip(),
            'max_speed': max(0.01, min(values[3], 0.2)),
            'max_wspeed': 0.2,
            'max_acc': 0.1,
            'max_wacc': 0.1,
        }
        return self.request(
            API_GOTO_STATION, body, timeout=max(5.0, self.timeout)
        ).body or {}

    def download_map(self, map_name: str) -> dict[str, Any]:
        return self.request(API_DOWNLOAD_MAP, {"map_name": map_name}, timeout=max(15.0, self.timeout)).body or {}

    def jog(self, vx: float, vy: float, w: float, duration_ms: int) -> dict[str, Any]:
        if self.max_vy <= 0.0 and abs(float(vy)) > 1e-6:
            raise SeerApiError(
                "lateral velocity is not supported by this differential-drive AGV"
            )
        vx = self._clamp(vx, self.max_vx)
        vy = self._clamp(vy, self.max_vy)
        w = self._clamp(w, self.max_w)
        duration_ms = max(1, min(int(duration_ms), self.max_duration_ms))
        return self.request(API_JOG, {"vx": vx, "vy": vy, "w": w, "duration": duration_ms}).body or {}

    def read_safety_state(self) -> tuple[SafetyState, dict[str, Any]]:
        fast = self.get_all2()
        loc = self.get_loc_status()
        map_status = self.get_map_status()
        battery = self.get_battery()
        control_owner = self.get_control_owner()
        safety = evaluate_safety_state(
            fast, loc, map_status, battery, control_owner
        )
        return safety, {
            "all2": fast,
            "loc_status": loc,
            "map_status": map_status,
            "blocked": fast,
            "alarm": fast,
            "nav_status": fast,
            "speed": fast,
            "emergency": fast,
            "battery": battery,
            "control_owner": control_owner,
        }

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, float(value)))


def evaluate_safety_state(
    fast: dict[str, Any],
    loc_status: dict[str, Any],
    map_status: dict[str, Any],
    battery: dict[str, Any],
    control_owner: dict[str, Any],
) -> SafetyState:
    """Build the common motion gate from documented SEER status fields.

    On Robokit releases before 3.4.6.1800, relocation value 3 means the
    calculation completed but an operator has not confirmed it.  The AMB-300
    in this project runs 3.4.4.6, so only value 1 is accepted here.
    """
    reloc_status = loc_status.get("reloc_status")
    localized = reloc_status == 1
    pending_confirmation = reloc_status == 3
    map_loaded = map_status.get("loadmap_status") == 1
    blocked = bool(fast.get("blocked", False))
    slowed = bool(fast.get("slowed", False))
    has_alarm = any(
        bool(fast.get(key)) for key in ("fatals", "errors", "warnings")
    )
    emergency_active = any(
        bool(fast.get(key))
        for key in ("emergency", "driver_emc", "soft_emc")
    )
    charging = any(
        bool(battery.get(key))
        for key in ("charging", "manual_charge", "auto_charge")
    )
    control_locked = bool(control_owner.get("locked", False))
    nav_value = fast.get("task_status")
    nav_status = int(nav_value) if isinstance(nav_value, (int, float)) else None

    reasons = []
    if pending_confirmation:
        reasons.append("localization awaiting operator confirmation")
    elif not localized:
        reasons.append("localization not successful")
    if not map_loaded:
        reasons.append("map not loaded")
    if emergency_active:
        reasons.append("emergency stop active")
    if charging:
        reasons.append("robot charging")
    if control_locked:
        owner = str(control_owner.get("nick_name") or control_owner.get("ip") or "unknown")
        reasons.append(f"control owned by {owner}")
    if blocked:
        reasons.append("robot blocked")
    if slowed:
        reasons.append("robot slowed")
    if has_alarm:
        reasons.append("active alarm")

    return SafetyState(
        localized=localized,
        localization_pending_confirmation=pending_confirmation,
        map_loaded=map_loaded,
        blocked=blocked,
        slowed=slowed,
        has_alarm=has_alarm,
        emergency_active=emergency_active,
        charging=charging,
        control_locked=control_locked,
        nav_status=nav_status,
        reason=", ".join(reasons),
    )


def parse_footprint(model: dict[str, Any]) -> Footprint:
    params = _find_chassis_rectangle_params(model)
    if params is None:
        raise ValueError("chassis rectangle footprint not found in robot model")

    width = _param_number(params, "width")
    head = _param_number(params, "head")
    tail = _param_number(params, "tail")
    height = _param_number(params, "height", 0.3)
    if width <= 0.0 or head <= 0.0 or tail <= 0.0:
        raise ValueError(f"invalid chassis rectangle footprint: width={width}, head={head}, tail={tail}")
    return Footprint(width=width, head=head, tail=tail, height=height)


def _find_chassis_rectangle_params(model: dict[str, Any]) -> list[dict[str, Any]] | None:
    for device_type in model.get("deviceTypes", []):
        if not isinstance(device_type, dict) or device_type.get("name") != "chassis":
            continue
        devices = device_type.get("devices", [])
        if not isinstance(devices, list) or not devices:
            continue
        device = devices[0]
        if not isinstance(device, dict):
            continue
        for param in device.get("deviceParams", []):
            if not isinstance(param, dict) or param.get("key") != "shape":
                continue
            combo = param.get("comboParam")
            if not isinstance(combo, dict) or combo.get("childKey") != "rectangle":
                continue
            for child in combo.get("childParams", []):
                if isinstance(child, dict) and child.get("key") == "rectangle":
                    params = child.get("params")
                    if isinstance(params, list):
                        return [item for item in params if isinstance(item, dict)]
    return None


def _param_number(params: list[dict[str, Any]], key: str, default: float | None = None) -> float:
    for param in params:
        if param.get("key") != key:
            continue
        for value_key in ("doubleValue", "floatValue", "uint32Value", "int32Value", "value"):
            value = param.get(value_key)
            if isinstance(value, (int, float)):
                return float(value)
        raise ValueError(f"robot model parameter {key!r} is not numeric")
    if default is not None:
        return default
    raise ValueError(f"robot model parameter {key!r} not found")
