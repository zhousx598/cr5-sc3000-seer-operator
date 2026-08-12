#!/usr/bin/env python3
"""Move a Dobot robot between named joint points.

The program intentionally does not enable the robot, clear alarms, change
collision settings, or leave drag mode. Those state changes must be explicit.
"""

import argparse
import math
from pathlib import Path
import re
import sys
import time
from typing import Iterable, Sequence

import rclpy
from rclpy.node import Node

from dobot_msgs_v3.srv import GetAngle
from dobot_msgs_v3.srv import GetErrorID
from dobot_msgs_v3.srv import JointMovJ
from dobot_msgs_v3.srv import RobotMode
from dobot_msgs_v3.srv import SpeedFactor


POINT_NAME_PATTERN = re.compile(r'^[A-Za-z0-9_.-]+$')
ANGLE_RESPONSE_PATTERN = re.compile(
    r"angle\s*=\s*['\"]\{([^{}]+)\}['\"]"
)
BRACED_VALUES_PATTERN = re.compile(r'\{([^{}]+)\}')
JOINT_COUNT = 6
MOTION_START_DELTA_DEG = 0.05
MOTION_SAMPLE_DELTA_DEG = 0.01
MOTION_POLL_PERIOD_SEC = 0.5
TARGET_STABLE_SAMPLES = 2
IDLE_AWAY_SAMPLES = 3
ROBOT_MODE_DESCRIPTIONS = {
    '1': '初始化',
    '2': '抱闸松开',
    '3': '未上电',
    '4': '未使能',
    '5': '已使能且空闲',
    '6': '拖动模式',
    '7': '运行中',
    '8': '单次运动/Jog',
    '9': '报警',
    '10': '暂停',
    '11': '碰撞',
}


class PointFileError(ValueError):
    """Raised when a saved point file cannot be parsed safely."""


def parse_joint_values(text: str) -> tuple[float, ...]:
    """Extract exactly six finite joint angles from saved GetAngle output."""
    match = ANGLE_RESPONSE_PATTERN.search(text)
    candidates = [match.group(1)] if match else []
    if not candidates:
        candidates = BRACED_VALUES_PATTERN.findall(text)

    for candidate in candidates:
        fields = [field.strip() for field in candidate.split(',')]
        if len(fields) != JOINT_COUNT:
            continue
        try:
            values = tuple(float(field) for field in fields)
        except ValueError:
            continue
        if all(math.isfinite(value) for value in values):
            return values

    raise PointFileError('未找到包含 6 个有限数值的关节角记录')


def load_named_point(points_dir: Path, point_name: str) -> tuple[float, ...]:
    """Load ``<point_name>_joint.txt`` without allowing path traversal."""
    if not POINT_NAME_PATTERN.fullmatch(point_name):
        raise PointFileError(
            f'非法点名 {point_name!r}；只允许字母、数字、点、下划线和连字符'
        )

    point_file = points_dir / f'{point_name}_joint.txt'
    try:
        text = point_file.read_text(encoding='utf-8')
    except FileNotFoundError as exc:
        raise PointFileError(f'点位文件不存在：{point_file}') from exc
    except OSError as exc:
        raise PointFileError(f'无法读取点位文件 {point_file}：{exc}') from exc

    try:
        return parse_joint_values(text)
    except PointFileError as exc:
        raise PointFileError(f'{point_file}：{exc}') from exc


def angular_error_deg(actual: float, target: float) -> float:
    """Return the smallest signed angular difference in degrees."""
    return (actual - target + 180.0) % 360.0 - 180.0


def joint_errors_deg(
    actual: Sequence[float], target: Sequence[float]
) -> tuple[float, ...]:
    return tuple(
        angular_error_deg(actual_value, target_value)
        for actual_value, target_value in zip(actual, target)
    )


def max_joint_error_deg(
    actual: Sequence[float], target: Sequence[float]
) -> float:
    """Return the largest absolute wrapped joint error in degrees."""
    return max(abs(error) for error in joint_errors_deg(actual, target))


def robot_mode_text(mode: str) -> str:
    description = ROBOT_MODE_DESCRIPTIONS.get(mode, '未知状态')
    return f'{mode}（{description}）'


def format_joints(values: Iterable[float]) -> str:
    return '[' + ', '.join(f'{value:.6f}' for value in values) + ']'


def parse_arguments(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            '读取 <点名>_joint.txt，并在确认当前机械臂位于起点后，'
            '使用 JointMovJ 低速运动到终点。默认只预览，不运动。'
        )
    )
    parser.add_argument('--from', dest='from_point', required=True, help='起点名')
    parser.add_argument('--to', dest='to_point', required=True, help='终点名')
    parser.add_argument(
        '--points-dir',
        type=Path,
        default=Path('~/dobot_ws/points').expanduser(),
        help='点位目录（默认：~/dobot_ws/points）',
    )
    parser.add_argument(
        '--tolerance-deg',
        type=float,
        default=0.5,
        help='起点及终点的逐关节允许误差，单位度（默认：0.5）',
    )
    parser.add_argument(
        '--speed-factor',
        type=int,
        default=5,
        help='全局速度百分比 1~100（默认：5）',
    )
    parser.add_argument(
        '--speed-j',
        type=int,
        default=10,
        help='JointMovJ 的 SpeedJ 百分比 1~100（默认：10）',
    )
    parser.add_argument(
        '--acc-j',
        type=int,
        default=10,
        help='JointMovJ 的 AccJ 百分比 1~100（默认：10）',
    )
    parser.add_argument(
        '--service-timeout',
        type=float,
        default=5.0,
        help='普通服务超时秒数（默认：5）',
    )
    parser.add_argument(
        '--motion-timeout',
        type=float,
        default=300.0,
        help='状态轮询等待运动完成的总超时秒数（默认：300）',
    )
    parser.add_argument(
        '--start-timeout',
        type=float,
        default=10.0,
        help='运动命令发出后等待机械臂开始运动的秒数（默认：10）',
    )
    parser.add_argument(
        '--execute',
        action='store_true',
        help='实际执行运动；不带此参数时只解析并预览点位',
    )
    args = parser.parse_args(argv)

    if args.from_point == args.to_point:
        parser.error('--from 和 --to 不能相同')
    if not 0.0 < args.tolerance_deg <= 10.0:
        parser.error('--tolerance-deg 必须在 (0, 10] 范围内')
    for name in ('speed_factor', 'speed_j', 'acc_j'):
        value = getattr(args, name)
        if not 1 <= value <= 100:
            parser.error(f'--{name.replace("_", "-")} 必须在 1~100 范围内')
    if (
        args.service_timeout <= 0.0
        or args.motion_timeout <= 0.0
        or args.start_timeout <= 0.0
    ):
        parser.error('超时时间必须大于 0')
    if args.start_timeout >= args.motion_timeout:
        parser.error('--start-timeout 必须小于 --motion-timeout')

    args.points_dir = args.points_dir.expanduser().resolve()
    return args


class PointMover(Node):
    """Service client with explicit state, position, and progress checks."""

    SERVICE_PREFIX = '/dobot_bringup_v3/srv'

    def __init__(
        self,
        service_timeout: float,
        motion_timeout: float,
        start_timeout: float,
    ) -> None:
        super().__init__('dobot_point_mover')
        self.service_timeout = service_timeout
        self.motion_timeout = motion_timeout
        self.start_timeout = start_timeout
        self._monotonic = time.monotonic
        self._sleep = time.sleep
        self.robot_mode_client = self.create_client(
            RobotMode, f'{self.SERVICE_PREFIX}/RobotMode'
        )
        self.get_angle_client = self.create_client(
            GetAngle, f'{self.SERVICE_PREFIX}/GetAngle'
        )
        self.get_error_id_client = self.create_client(
            GetErrorID, f'{self.SERVICE_PREFIX}/GetErrorID'
        )
        self.speed_factor_client = self.create_client(
            SpeedFactor, f'{self.SERVICE_PREFIX}/SpeedFactor'
        )
        self.joint_move_client = self.create_client(
            JointMovJ, f'{self.SERVICE_PREFIX}/JointMovJ'
        )

    def _call(self, client, request, timeout: float, service_name: str):
        if not client.wait_for_service(timeout_sec=self.service_timeout):
            raise RuntimeError(f'ROS 服务不可用：{service_name}')

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            raise RuntimeError(f'调用 {service_name} 超时（{timeout:.1f}s）')
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f'调用 {service_name} 失败：{exception}')
        return future.result()

    def get_robot_mode(self) -> str:
        response = self._call(
            self.robot_mode_client,
            RobotMode.Request(),
            self.service_timeout,
            'RobotMode',
        )
        if response.res != 0:
            raise RuntimeError(f'RobotMode 返回 res={response.res}')
        return response.mode

    def require_idle_enabled(self) -> None:
        mode = self.get_robot_mode()
        if mode != '5':
            raise RuntimeError(
                f'当前 RobotMode={robot_mode_text(mode)}，'
                '必须为 5（已使能且空闲）；'
                '本工具不会自动使能、清警或退出拖动模式'
            )

    def get_angles(self) -> tuple[float, ...]:
        response = self._call(
            self.get_angle_client,
            GetAngle.Request(),
            self.service_timeout,
            'GetAngle',
        )
        if response.res != 0:
            raise RuntimeError(f'GetAngle 返回 res={response.res}')
        try:
            return parse_joint_values(response.angle)
        except PointFileError as exc:
            raise RuntimeError(f'无法解析 GetAngle 响应 {response.angle!r}：{exc}') from exc

    def get_error_ids_for_diagnostic(self) -> str:
        """Return controller alarm payload without hiding the original failure."""
        try:
            response = self._call(
                self.get_error_id_client,
                GetErrorID.Request(),
                self.service_timeout,
                'GetErrorID',
            )
            if response.res != 0:
                return f'GetErrorID 返回 res={response.res}'
            return response.error_id.strip() or '控制器未返回报警码'
        except RuntimeError as exc:
            return f'读取报警码失败：{exc}'

    def set_speed_factor(self, ratio: int) -> None:
        request = SpeedFactor.Request()
        request.ratio = ratio
        response = self._call(
            self.speed_factor_client,
            request,
            self.service_timeout,
            'SpeedFactor',
        )
        if response.res != 0:
            raise RuntimeError(f'SpeedFactor 返回 res={response.res}')

    def move_joint(self, target: Sequence[float], speed_j: int, acc_j: int) -> None:
        request = JointMovJ.Request()
        (
            request.j1,
            request.j2,
            request.j3,
            request.j4,
            request.j5,
            request.j6,
        ) = target
        request.param_value = [f'SpeedJ={speed_j}', f'AccJ={acc_j}']
        response = self._call(
            self.joint_move_client,
            request,
            self.service_timeout,
            'JointMovJ',
        )
        if response.res != 0:
            raise RuntimeError(f'JointMovJ 返回 res={response.res}')

    def wait_until_arrived(
        self,
        start: Sequence[float],
        target: Sequence[float],
        tolerance_deg: float,
    ) -> tuple[float, ...]:
        """Poll position and mode without blocking the bringup node on Sync."""
        started_at = self._monotonic()
        deadline = started_at + self.motion_timeout
        next_report_at = started_at
        motion_started = False
        target_stable_samples = 0
        idle_away_samples = 0
        actual = tuple(start)
        previous_actual = tuple(start)
        mode = '5'

        while True:
            now = self._monotonic()
            if now >= deadline:
                target_error = max_joint_error_deg(actual, target)
                raise RuntimeError(
                    f'等待运动完成超时（{self.motion_timeout:.1f}s）：'
                    f'RobotMode={robot_mode_text(mode)}，'
                    f'最大关节误差={target_error:.3f}°，'
                    f'最后角度={format_joints(actual)}°'
                )

            actual = self.get_angles()
            mode = self.get_robot_mode()
            now = self._monotonic()
            elapsed = now - started_at
            target_error = max_joint_error_deg(actual, target)
            start_delta = max_joint_error_deg(actual, start)
            sample_delta = max_joint_error_deg(actual, previous_actual)

            if mode == '7' or start_delta >= MOTION_START_DELTA_DEG:
                motion_started = True

            if mode not in {'5', '7'}:
                error_ids = self.get_error_ids_for_diagnostic()
                raise RuntimeError(
                    f'运动期间 RobotMode={robot_mode_text(mode)}，运动未完成。\n'
                    f'  当前角度：{format_joints(actual)}°\n'
                    f'  最大关节误差：{target_error:.3f}°\n'
                    f'  报警信息：{error_ids}\n'
                    '本工具不会自动清警、恢复暂停或解除碰撞状态。'
                )

            if target_error <= tolerance_deg and mode == '5':
                target_stable_samples += 1
                if target_stable_samples >= TARGET_STABLE_SAMPLES:
                    return actual
            else:
                target_stable_samples = 0

            if (
                motion_started
                and mode == '5'
                and target_error > tolerance_deg
                and sample_delta < MOTION_SAMPLE_DELTA_DEG
            ):
                idle_away_samples += 1
                if idle_away_samples >= IDLE_AWAY_SAMPLES:
                    raise RuntimeError(
                        '机械臂已停止，但没有到达目标点：\n'
                        f'  当前角度：{format_joints(actual)}°\n'
                        f'  最大关节误差：{target_error:.3f}°'
                    )
            else:
                idle_away_samples = 0

            if not motion_started and elapsed >= self.start_timeout:
                error_ids = self.get_error_ids_for_diagnostic()
                raise RuntimeError(
                    f'JointMovJ 已被控制器接受，但 {self.start_timeout:.1f}s 内'
                    '机械臂没有开始运动。运动队列可能处于暂停/堵塞状态，'
                    '也可能刚触发报警；未继续发送任何运动指令。\n'
                    f'  RobotMode={robot_mode_text(mode)}\n'
                    f'  当前角度：{format_joints(actual)}°\n'
                    f'  最大关节误差：{target_error:.3f}°\n'
                    f'  报警信息：{error_ids}'
                )

            if now >= next_report_at:
                print(
                    f'等待到位：{elapsed:.1f}s，'
                    f'RobotMode={robot_mode_text(mode)}，'
                    f'最大关节误差={target_error:.3f}°',
                    flush=True,
                )
                next_report_at = now + 5.0

            previous_actual = actual
            self._sleep(MOTION_POLL_PERIOD_SEC)


def ensure_within_tolerance(
    actual: Sequence[float],
    target: Sequence[float],
    tolerance_deg: float,
    description: str,
) -> None:
    errors = joint_errors_deg(actual, target)
    if any(abs(error) > tolerance_deg for error in errors):
        raise RuntimeError(
            f'{description}不满足逐关节 ±{tolerance_deg:.3f}°：\n'
            f'  实际角度：{format_joints(actual)}\n'
            f'  目标角度：{format_joints(target)}\n'
            f'  角度误差：{format_joints(errors)}'
        )


def run(args: argparse.Namespace) -> int:
    try:
        from_joints = load_named_point(args.points_dir, args.from_point)
        to_joints = load_named_point(args.points_dir, args.to_point)
    except PointFileError as exc:
        print(f'错误：{exc}', file=sys.stderr)
        return 2

    delta = tuple(
        target - start for start, target in zip(from_joints, to_joints)
    )
    print(f'点位目录：{args.points_dir}')
    print(f'起点 {args.from_point}：{format_joints(from_joints)}°')
    print(f'终点 {args.to_point}：{format_joints(to_joints)}°')
    print(f'变化量：{format_joints(delta)}°')
    print(
        f'参数：SpeedFactor={args.speed_factor}%，'
        f'SpeedJ={args.speed_j}%，AccJ={args.acc_j}%，'
        f'容差={args.tolerance_deg}°'
    )

    if not args.execute:
        print('预览完成；未发送任何 ROS 服务请求。实际运动请显式添加 --execute。')
        return 0

    # Do not let rclpy parse this tool's argparse options as ROS arguments.
    rclpy.init(args=[])
    node = PointMover(
        args.service_timeout,
        args.motion_timeout,
        args.start_timeout,
    )
    try:
        node.require_idle_enabled()
        current = node.get_angles()
        ensure_within_tolerance(
            current,
            from_joints,
            args.tolerance_deg,
            f'当前位置不是起点 {args.from_point}',
        )
        print(f'当前位置已通过起点 {args.from_point} 校验。')

        node.set_speed_factor(args.speed_factor)
        print(f'开始运动：{args.from_point} -> {args.to_point}')
        node.move_joint(to_joints, args.speed_j, args.acc_j)
        final = node.wait_until_arrived(
            current,
            to_joints,
            args.tolerance_deg,
        )
        ensure_within_tolerance(
            final,
            to_joints,
            args.tolerance_deg,
            f'未到达终点 {args.to_point}',
        )
        print(f'运动完成，已到达 {args.to_point}：{format_joints(final)}°')
        return 0
    except KeyboardInterrupt:
        print(
            '收到 Ctrl+C。注意：终端中断不保证控制器中的运动队列停止，'
            '必要时使用实体急停。',
            file=sys.stderr,
        )
        return 130
    except RuntimeError as exc:
        print(f'错误：{exc}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv: Sequence[str] | None = None) -> None:
    args = parse_arguments(argv)
    raise SystemExit(run(args))


if __name__ == '__main__':
    main()
