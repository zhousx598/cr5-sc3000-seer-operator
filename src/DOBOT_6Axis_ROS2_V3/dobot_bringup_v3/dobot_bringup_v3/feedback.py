#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      
from dobot_msgs_v3.msg import ToolVectorActual
from sensor_msgs.msg import JointState  
import socket
import numpy as np
import os
import time

MyType = np.dtype([('len',np.int64,), ('digital_input_bits',np.uint64,), ('digital_output_bits',
    np.uint64,), ('robot_mode',np.uint64,), ('time_stamp',np.uint64,), ( 'time_stamp_reserve_bit', np.uint64,),
    ('test_value',np.uint64,), ('test_value_keep_bit', np.float64,), ('speed_scaling',np.float64,), ('linear_momentum_norm',np.float64,),
    ( 'v_main',np.float64,), ('v_robot',np.float64,), ('i_robot',np.float64,), ('i_robot_keep_bit1',np.float64,), ( 'i_robot_keep_bit2',np.float64,),
    ('tool_accelerometer_values', np.float64, (3, )),
    ('elbow_position', np.float64, (3, )),
    ('elbow_velocity', np.float64, (3, )),
    ('q_target', np.float64, (6, )),
    ('qd_target', np.float64, (6, )),
    ('qdd_target', np.float64, (6, )),
    ('i_target', np.float64, (6, )),
    ('m_target', np.float64, (6, )),
    ('q_actual', np.float64, (6, )),
    ('qd_actual', np.float64, (6, )),
    ('i_actual', np.float64, (6, )),
    ('actual_TCP_force', np.float64, (6, )),
    ('tool_vector_actual', np.float64, (6, )),
    ('TCP_speed_actual', np.float64, (6, )),
    ('TCP_force', np.float64, (6, )),
    ('Tool_vector_target', np.float64, (6, )),
    ('TCP_speed_target', np.float64, (6, )),
    ('motor_temperatures', np.float64, (6, )),
    ('joint_modes', np.float64, (6, )),
    ('v_actual', np.float64, (6, )),
    ('hand_type', np.byte, (4,)),
    ('user', np.byte,),
    ('tool', np.byte,),
    ('run_queued_cmd', np.byte,),
    ('pause_cmd_flag', np.byte,),
    ('velocity_ratio', np.int8,),
    ('acceleration_ratio', np.int8,),
    ('jerk_ratio', np.int8,),
    ('xyz_velocity_ratio', np.int8,),
    ('r_velocity_ratio', np.int8,),
    ('xyz_acceleration_ratio', np.int8,),
    ('r_acceleration_ratio', np.int8,),
    ('xyz_jerk_ratio', np.int8,),
    ('r_jerk_ratio', np.int8,),
    ('brake_status', np.int8,),
    ('enable_status', np.int8,),
    ('drag_status', np.int8,),
    ('running_status', np.int8,),
    ('error_status',np.int8,),
    ('jog_status', np.int8,),
    ('robot_type', np.int8,),
    ('drag_button_signal', np.int8,),
    ('enable_button_signal', np.int8,),
    ('record_button_signal', np.int8,),
    ('reappear_button_signal', np.int8,),
    ('jaw_button_signal', np.int8,),
    ('six_force_online', np.int8,),
    ('reserve2', np.int8, (82,)),
    ('m_actual', np.float64, (6,)),
    ('load', np.float64,),
    ('center_x', np.float64,),
    ('center_y', np.float64,),
    ('center_z', np.float64,),
    ('user1', np.float64, (6,)),
    ('Tool1', np.float64, (6,)),
    ('trace_index', np.float64,),
    ('six_force_value', np.float64, (6,)),
    ('target_quaternion', np.float64, (4,)),
    ('actual_quaternion', np.float64, (4,)),
    ('reserve3',np.int8, (24,))
     ])

FEEDBACK_PACKET_SIZE = MyType.itemsize
FEEDBACK_PERIOD_SEC = 0.01
RECONNECT_INTERVAL_SEC = 2.0
DEFAULT_FEEDBACK_PORT = 30005
ALLOWED_FEEDBACK_PORTS = (30004, 30005)


def resolve_feedback_port(value=None):
    """Return the validated CR5 feedback port from the environment."""
    raw_value = os.getenv("DOBOT_FEEDBACK_PORT") if value is None else value
    if raw_value is None:
        return DEFAULT_FEEDBACK_PORT

    try:
        port = int(str(raw_value).strip())
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "DOBOT_FEEDBACK_PORT must be numeric and one of 30004 or 30005; "
            f"got {raw_value!r}"
        ) from exc

    if port not in ALLOWED_FEEDBACK_PORTS:
        raise ValueError(
            "DOBOT_FEEDBACK_PORT must be 30004 or 30005; "
            f"got {port}"
        )
    return port




class fankuis():
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.socket_feedback = None
        self._buffer = bytearray()

        if self.port not in ALLOWED_FEEDBACK_PORTS:
            raise ValueError(
                f"Feedback server requires port 30004 or 30005, got {self.port}"
            )

        feedback_socket = socket.socket()
        feedback_socket.settimeout(1.0)
        try:
            feedback_socket.connect((self.ip, self.port))
        except OSError:
            feedback_socket.close()
            raise
        self.socket_feedback = feedback_socket

    def close(self):
        feedback_socket = self.socket_feedback
        self.socket_feedback = None
        if feedback_socket is not None:
            try:
                feedback_socket.close()
            except OSError:
                pass

    def feed(self):
        if self.socket_feedback is None:
            raise ConnectionError("Feedback socket is not connected")

        while len(self._buffer) < FEEDBACK_PACKET_SIZE:
            chunk = self.socket_feedback.recv(
                FEEDBACK_PACKET_SIZE - len(self._buffer)
            )
            if not chunk:
                raise ConnectionError("Feedback connection closed by robot")
            self._buffer.extend(chunk)

        data = bytes(self._buffer[:FEEDBACK_PACKET_SIZE])
        del self._buffer[:FEEDBACK_PACKET_SIZE]
        feedback = np.frombuffer(data, dtype=MyType, count=1)
        if hex(feedback['test_value'][0]) != '0x123456789abcdef':
            raise ValueError("Invalid feedback frame marker")

        tool_v = feedback['tool_vector_actual'][0].copy()
        tool_j = feedback['q_actual'][0].copy()
        return tool_v, tool_j

"""
创建一个发布者节点
"""
class PublisherNode(Node):
    
    def __init__(self, name):
        super().__init__(name)                                    
        # self.declare_parameter('IP', '192.168.5.1')  # 默认值
        # self.IP = self.get_parameter('IP').get_parameter_value().string_value
        self.IP = os.getenv("IP_address")
        if not self.IP:
            raise RuntimeError("Environment variable IP_address is not set")
        self.feedback_port = resolve_feedback_port()
        self.feed_v = None
        self._next_reconnect_at = 0.0
        self.pub = self.create_publisher(ToolVectorActual, "dobot_msgs_v3/msg/ToolVectorActual", 10)
        self.pub2 = self.create_publisher(JointState, "joint_states_robot", 10)
        self.connect()
        self.timer = self.create_timer(FEEDBACK_PERIOD_SEC, self.timer_callback)

    def connect(self):
        if self.feed_v is not None:
            return True

        self.get_logger().info(f"connection:{self.IP}:{self.feedback_port}")
        try:
            feed_v = fankuis(self.IP, self.feedback_port)
        except (OSError, ValueError) as exc:
            self._next_reconnect_at = time.monotonic() + RECONNECT_INTERVAL_SEC
            self.get_logger().warning(
                f"连接 {self.IP}:{self.feedback_port} 失败："
                f"{type(exc).__name__}: {exc}；"
                f"{RECONNECT_INTERVAL_SEC:.0f} 秒后重试"
            )
            return False

        self.feed_v = feed_v
        self._next_reconnect_at = 0.0
        self.get_logger().info(
            f"connection succeeded:{self.IP}:{self.feedback_port}"
        )
        return True

    def disconnect(self, reason=None):
        feed_v = self.feed_v
        self.feed_v = None
        if feed_v is not None:
            feed_v.close()
        self._next_reconnect_at = time.monotonic() + RECONNECT_INTERVAL_SEC
        if reason is not None:
            self.get_logger().warning(
                f"反馈连接 {self.IP}:{self.feedback_port} 已断开："
                f"{type(reason).__name__}: {reason}；"
                f"{RECONNECT_INTERVAL_SEC:.0f} 秒后重试"
            )

    def timer_callback(self):                                     
        if self.feed_v is None:
            if time.monotonic() >= self._next_reconnect_at:
                self.connect()
            return

        msg = ToolVectorActual()                                           
        try:
            actual = self.feed_v.feed()
        except (OSError, ValueError) as exc:
            self.disconnect(exc)
            return

        msg2 = JointState()
        msg2.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        msg2.header.stamp = self.get_clock().now().to_msg()
        msg2.header.frame_id = 'joint_states'
        q_target = actual[1]
        msg2.position = [float(angle * np.pi / 180.0) for angle in q_target]
        msg.x = actual[0][0]
        msg.y = actual[0][1]
        msg.z = actual[0][2]
        msg.rx = actual[0][3]
        msg.ry = actual[0][4]
        msg.rz = actual[0][5]
        self.pub.publish(msg)
        self.pub2.publish(msg2)
        
def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode("dobot_feedback")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.disconnect()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
