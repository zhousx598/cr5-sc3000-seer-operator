"""Application entry point."""

import os
import signal
import sys
import threading

from PyQt5.QtCore import QLibraryInfo
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

from .main_window import DobotOperatorWindow
from .ros_client import DobotRosClient


def restore_pyqt_plugin_path() -> str:
    """Undo the incompatible Qt plugin path injected by OpenCV wheels.

    The pip OpenCV wheel bundles its own Qt build and unconditionally points
    ``QT_QPA_PLATFORM_PLUGIN_PATH`` at that bundle when ``cv2`` is imported.
    This application embeds OpenCV in a system PyQt5 GUI, so Qt must instead
    load the platform plugins belonging to the active PyQt5 installation.
    """
    plugin_path = QLibraryInfo.location(QLibraryInfo.PluginsPath)
    if not plugin_path:
        raise RuntimeError('PyQt5没有返回Qt插件目录')
    os.environ.pop('QT_PLUGIN_PATH', None)
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = plugin_path
    # The OpenCV wheel also injects its private font directory.  System Qt
    # should use the desktop's normal font configuration.
    os.environ.pop('QT_QPA_FONTDIR', None)
    return plugin_path


def main(args=None) -> int:
    restore_pyqt_plugin_path()
    ros_arguments = sys.argv if args is None else args
    # Qt owns the main thread and must coordinate shutdown.  Letting rclpy's
    # default SIGINT handler invalidate the context first leaves active Qt
    # timers calling ROS clients after shutdown.
    rclpy.init(
        args=ros_arguments,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    qt_arguments = remove_ros_args(args=ros_arguments)
    application = QApplication(qt_arguments)
    application.setApplicationName('Dobot CR5 上位机')

    node = DobotRosClient()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=executor.spin, name='dobot_gui_ros_executor', daemon=True
    )
    spin_thread.start()
    window = DobotOperatorWindow(node)
    window.show()

    def request_shutdown(unused_signum, unused_frame) -> None:
        application.quit()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    signal_timer = QTimer()
    signal_timer.setInterval(200)
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start()

    exit_code = application.exec_()
    signal_timer.stop()
    window.shutdown_workers()
    executor.shutdown(timeout_sec=3.0)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    spin_thread.join(timeout=3.0)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
