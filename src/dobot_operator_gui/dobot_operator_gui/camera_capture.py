"""SC3000 one-shot capture over Modbus TCP and FTP."""

from dataclasses import dataclass
import os
from pathlib import Path
import socket
import struct
import threading
import time


IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.bmp'}
DEFAULT_FTP_USER = os.environ.get('SC3000_FTP_USER', 'sc3000')
DEFAULT_FTP_PASSWORD = os.environ.get('SC3000_FTP_PASSWORD', '123456')


class CameraCaptureError(RuntimeError):
    """Raised when an SC3000 trigger or image upload fails."""


@dataclass(frozen=True)
class CameraCaptureResult:
    image_path: Path
    trigger_time_unix_ns: int
    received_time_unix_ns: int
    status_register: int


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = []
    received = 0
    while received < size:
        chunk = sock.recv(size - received)
        if not chunk:
            raise CameraCaptureError('SC3000 关闭了 Modbus TCP 连接')
        chunks.append(chunk)
        received += len(chunk)
    return b''.join(chunks)


def _modbus_request(
    sock: socket.socket,
    transaction_id: int,
    unit_id: int,
    function: int,
    payload: bytes,
) -> tuple[int, bytes]:
    mbap = struct.pack('>HHHB', transaction_id, 0, len(payload) + 2, unit_id)
    sock.sendall(mbap + bytes([function]) + payload)
    header = _recv_exact(sock, 7)
    rx_tid, protocol, length, rx_unit = struct.unpack('>HHHB', header)
    body = _recv_exact(sock, length - 1)
    if (
        rx_tid != transaction_id
        or protocol != 0
        or rx_unit != unit_id
    ):
        raise CameraCaptureError('SC3000 返回了不匹配的 Modbus 响应头')
    if not body:
        raise CameraCaptureError('SC3000 返回了空 Modbus 响应')
    rx_function = body[0]
    if rx_function & 0x80:
        code = body[1] if len(body) > 1 else None
        raise CameraCaptureError(
            f'Modbus 异常：function=0x{rx_function:02x}, code={code}'
        )
    return rx_function, body[1:]


def _write_holding(
    sock: socket.socket,
    transaction_id: int,
    unit_id: int,
    address: int,
    value: int,
) -> None:
    payload = struct.pack('>HH', address, value)
    function, data = _modbus_request(
        sock, transaction_id, unit_id, 0x06, payload
    )
    if function != 0x06 or data != payload:
        raise CameraCaptureError('SC3000 返回了异常的寄存器写响应')


def _read_holding(
    sock: socket.socket,
    transaction_id: int,
    unit_id: int,
    address: int,
) -> int:
    payload = struct.pack('>HH', address, 1)
    function, data = _modbus_request(
        sock, transaction_id, unit_id, 0x03, payload
    )
    if function != 0x03 or len(data) != 3 or data[0] != 2:
        raise CameraCaptureError('SC3000 返回了异常的寄存器读响应')
    return struct.unpack('>H', data[1:])[0]


def _image_files(directory: Path) -> list[Path]:
    if not directory.exists():
        return []
    return [
        path
        for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    ]


def _image_snapshot(directory: Path) -> dict[str, tuple[int, int]]:
    snapshot = {}
    for path in _image_files(directory):
        stat = path.stat()
        snapshot[path.name] = (stat.st_mtime_ns, stat.st_size)
    return snapshot


def _wait_for_complete_image(
    directory: Path,
    previous: dict[str, tuple[int, int]],
    timeout: float,
    cancel_event: threading.Event | None = None,
) -> tuple[Path, int]:
    deadline = time.monotonic() + timeout
    last_candidate: Path | None = None
    last_signature: tuple[int, int] | None = None
    stable_samples = 0
    while time.monotonic() < deadline:
        if cancel_event is not None and cancel_event.is_set():
            raise CameraCaptureError('SC3000 实时预览已停止')
        changed = []
        for path in _image_files(directory):
            stat = path.stat()
            signature = (stat.st_mtime_ns, stat.st_size)
            if signature != previous.get(path.name):
                changed.append((stat.st_mtime_ns, path, signature))
        if changed:
            unused_mtime, candidate, signature = max(changed)
            if (
                candidate == last_candidate
                and signature == last_signature
                and signature[1] > 0
            ):
                stable_samples += 1
            else:
                last_candidate = candidate
                last_signature = signature
                stable_samples = 1
            if stable_samples >= 3:
                return candidate, time.time_ns()
        time.sleep(0.02)
    raise CameraCaptureError(
        f'{timeout:g} 秒内未在 {directory} 收到该次触发的新图；'
        '请检查 FTP 目标 IP、2121/30000:30009 端口和相机运行态'
    )


def _port_is_open(port: int) -> bool:
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=0.3):
            return True
    except OSError:
        return False


class Sc3000FtpServer:
    """Lazily start one in-process FTP server for the GUI."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._server = None
        self._thread: threading.Thread | None = None
        self._configuration: tuple[Path, int, str] | None = None

    def ensure(
        self,
        directory: Path,
        port: int,
        user: str,
        password: str,
        passive_from: int = 30000,
        passive_to: int = 30009,
    ) -> bool:
        """Ensure a listener exists; return True only when started here."""
        target = directory.expanduser().resolve()
        target.mkdir(parents=True, exist_ok=True)
        configuration = (target, port, user)
        with self._lock:
            if self._server is not None:
                if self._configuration != configuration:
                    raise CameraCaptureError(
                        '上位机 FTP Server 已使用其他目录、端口或用户启动'
                    )
                return False
            if _port_is_open(port):
                raise CameraCaptureError(
                    f'FTP 端口 {port} 已被其他程序占用。请停止独立运行的 '
                    'SC3000 FTP Server，再由上位机启动内存预览服务'
                )
            try:
                from pyftpdlib.authorizers import DummyAuthorizer
                from pyftpdlib.handlers import FTPHandler
                from pyftpdlib.servers import FTPServer
            except ImportError as exc:
                raise CameraCaptureError(
                    '缺少 pyftpdlib；请执行：'
                    '/usr/bin/python3 -m pip install --user pyftpdlib'
                ) from exc

            authorizer = DummyAuthorizer()
            authorizer.add_user(
                user, password, str(target), perm='elradfmwMT'
            )

            class Sc3000Handler(FTPHandler):
                pass

            Sc3000Handler.authorizer = authorizer
            Sc3000Handler.passive_ports = range(passive_from, passive_to + 1)
            try:
                server = FTPServer(('0.0.0.0', port), Sc3000Handler)
            except OSError as exc:
                raise CameraCaptureError(
                    f'无法监听 FTP 端口 {port}：{exc}'
                ) from exc
            thread = threading.Thread(
                target=server.serve_forever,
                name='sc3000_gui_ftp_server',
                daemon=True,
            )
            thread.start()
            self._server = server
            self._thread = thread
            self._configuration = configuration
            return True

    def close(self) -> None:
        with self._lock:
            server = self._server
            thread = self._thread
            self._server = None
            self._thread = None
            self._configuration = None
        if server is not None:
            server.close_all()
        if thread is not None:
            thread.join(timeout=2.0)


class Sc3000CameraCapture:
    """Trigger the configured SC3000 task and return its uploaded image."""

    def __init__(self, ftp_server: Sc3000FtpServer | None = None) -> None:
        self.ftp_server = ftp_server or Sc3000FtpServer()
        self._capture_lock = threading.Lock()

    def capture(
        self,
        host: str,
        image_dir: Path,
        *,
        timeout: float = 12.0,
        modbus_port: int = 502,
        unit_id: int = 1,
        ftp_port: int = 2121,
        ftp_user: str = DEFAULT_FTP_USER,
        ftp_password: str = DEFAULT_FTP_PASSWORD,
        ftp_startup_settle: float = 2.0,
        cancel_event: threading.Event | None = None,
    ) -> CameraCaptureResult:
        if not host.strip():
            raise CameraCaptureError('请输入 SC3000 IP 地址')
        if timeout <= 0:
            raise CameraCaptureError('相机超时时间必须大于 0')
        inbox = Path(image_dir).expanduser().resolve()

        with self._capture_lock:
            if cancel_event is not None and cancel_event.is_set():
                raise CameraCaptureError('SC3000 实时预览已停止')
            started = self.ftp_server.ensure(
                inbox, ftp_port, ftp_user, ftp_password
            )
            if started and ftp_startup_settle > 0:
                if (
                    cancel_event is not None
                    and cancel_event.wait(ftp_startup_settle)
                ):
                    raise CameraCaptureError('SC3000 实时预览已停止')
                if cancel_event is None:
                    time.sleep(ftp_startup_settle)
            previous = _image_snapshot(inbox)
            trigger_time = time.time_ns()
            status = self._trigger_modbus(
                host.strip(), modbus_port, unit_id, timeout, cancel_event
            )
            image_path, received_time = _wait_for_complete_image(
                inbox, previous, timeout, cancel_event
            )
            return CameraCaptureResult(
                image_path=image_path,
                trigger_time_unix_ns=trigger_time,
                received_time_unix_ns=received_time,
                status_register=status,
            )

    @staticmethod
    def _trigger_modbus(
        host: str,
        port: int,
        unit_id: int,
        timeout: float,
        cancel_event: threading.Event | None = None,
    ) -> int:
        transaction_id = 1
        triggered = False
        if cancel_event is not None and cancel_event.is_set():
            raise CameraCaptureError('SC3000 实时预览已停止')
        try:
            sock = socket.create_connection((host, port), timeout=3.0)
        except OSError as exc:
            raise CameraCaptureError(
                f'无法连接 SC3000 Modbus {host}:{port}：{exc}'
            ) from exc
        with sock:
            sock.settimeout(3.0)
            _write_holding(sock, transaction_id, unit_id, 0, 0x0001)
            transaction_id += 1

            deadline = time.monotonic() + timeout
            status = 0
            while time.monotonic() < deadline:
                if cancel_event is not None and cancel_event.is_set():
                    raise CameraCaptureError('SC3000 实时预览已停止')
                status = _read_holding(sock, transaction_id, unit_id, 1)
                transaction_id += 1
                if status & 0x8000:
                    raise CameraCaptureError(
                        f'SC3000 报告 General Fault，REG1=0x{status:04x}'
                    )
                if status & 0x0001:
                    break
                time.sleep(0.02)
            else:
                raise CameraCaptureError(
                    f'{timeout:g} 秒内未等到 Trigger Ready；'
                    '请确认相机已进入循环运行态'
                )

            _write_holding(sock, transaction_id, unit_id, 0, 0x0003)
            transaction_id += 1
            triggered = True
            result_status = 0
            try:
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    if cancel_event is not None and cancel_event.is_set():
                        raise CameraCaptureError('SC3000 实时预览已停止')
                    result_status = _read_holding(
                        sock, transaction_id, unit_id, 1
                    )
                    transaction_id += 1
                    if result_status & 0x8000:
                        raise CameraCaptureError(
                            'SC3000 报告 General Fault，'
                            f'REG1=0x{result_status:04x}'
                        )
                    if result_status & 0x0200:
                        raise CameraCaptureError(
                            'SC3000 任务执行超时（Results Timeout）'
                        )
                    if result_status & 0x0100:
                        return result_status
                    time.sleep(0.02)
                raise CameraCaptureError(
                    f'{timeout:g} 秒内未等到 Results Available'
                )
            finally:
                if triggered:
                    _write_holding(
                        sock, transaction_id, unit_id, 0, 0x0005
                    )
                    transaction_id += 1
                    time.sleep(0.02)
                    _write_holding(
                        sock, transaction_id, unit_id, 0, 0x0001
                    )

    def close(self) -> None:
        self.ftp_server.close()
