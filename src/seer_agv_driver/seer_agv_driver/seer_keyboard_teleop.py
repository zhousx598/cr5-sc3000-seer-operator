import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """SEER keyboard teleop

Arrow keys:
  Up/Down    forward/backward
  Left/Right rotate left/right
  Space      stop
  q          quit

Parameters:
  linear_speed  default 0.03 m/s
  forward_speed optional, defaults to linear_speed
  backward_speed optional, defaults to linear_speed
  angular_speed default 0.10 rad/s
  publish_rate  default 10 Hz
  key_timeout   default 0.15 s
"""


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("seer_keyboard_teleop")
        self.declare_parameter("cmd_vel_topic", "/seer_agv/cmd_vel")
        self.declare_parameter("linear_speed", 0.03)
        self.declare_parameter("forward_speed", -1.0)
        self.declare_parameter("backward_speed", -1.0)
        self.declare_parameter("angular_speed", 0.10)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("key_timeout", 0.15)

        topic = str(self.get_parameter("cmd_vel_topic").value)
        self.linear_speed = abs(float(self.get_parameter("linear_speed").value))
        forward_speed = float(self.get_parameter("forward_speed").value)
        backward_speed = float(self.get_parameter("backward_speed").value)
        self.forward_speed = abs(forward_speed) if forward_speed >= 0.0 else self.linear_speed
        self.backward_speed = abs(backward_speed) if backward_speed >= 0.0 else self.linear_speed
        self.angular_speed = abs(float(self.get_parameter("angular_speed").value))
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.key_timeout = max(0.05, float(self.get_parameter("key_timeout").value))

        self.pub = self.create_publisher(Twist, topic, 10)
        self.current = Twist()
        self.last_key_time = 0.0
        self.timer = self.create_timer(1.0 / max(1.0, publish_rate), self._publish)
        self.get_logger().info(
            f"publishing {topic}: forward={self.forward_speed:.3f} m/s, "
            f"backward={self.backward_speed:.3f} m/s, "
            f"angular={self.angular_speed:.3f} rad/s, key_timeout={self.key_timeout:.2f} s"
        )

    def handle_key(self, key: str) -> bool:
        msg = Twist()
        if key == "\x1b[A":
            msg.linear.x = self.forward_speed
        elif key == "\x1b[B":
            msg.linear.x = -self.backward_speed
        elif key == "\x1b[C":
            msg.angular.z = -self.angular_speed
        elif key == "\x1b[D":
            msg.angular.z = self.angular_speed
        elif key == " ":
            pass
        elif key.lower() == "q":
            self.current = Twist()
            self._publish()
            return False
        else:
            return True
        self.current = msg
        self.last_key_time = time.monotonic()
        self._publish()
        return True

    def _publish(self) -> None:
        if self.last_key_time and time.monotonic() - self.last_key_time > self.key_timeout:
            self.current = Twist()
            self.last_key_time = 0.0
        self.pub.publish(self.current)


def read_key(timeout: float = 0.1) -> str:
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return ""
    first = sys.stdin.read(1)
    if first == "\x1b":
        rest = sys.stdin.read(2)
        return first + rest
    return first


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleop()
    old_settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key(0.05)
            if key and not node.handle_key(key):
                break
    finally:
        node.current = Twist()
        node._publish()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
