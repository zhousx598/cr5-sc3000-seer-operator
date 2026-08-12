"""PyQt5 main window for the Dobot CR5 operator panel."""

from collections import deque
from concurrent.futures import Future
from concurrent.futures import ThreadPoolExecutor
import itertools
import os
from pathlib import Path
import re
import threading
import time
from typing import Callable

import cv2
import numpy as np
from PyQt5.QtCore import QObject
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QAbstractItemView
from PyQt5.QtWidgets import QCheckBox
from PyQt5.QtWidgets import QComboBox
from PyQt5.QtWidgets import QDoubleSpinBox
from PyQt5.QtWidgets import QFileDialog
from PyQt5.QtWidgets import QFormLayout
from PyQt5.QtWidgets import QFrame
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QGroupBox
from PyQt5.QtWidgets import QHeaderView
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QListWidget
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QSlider
from PyQt5.QtWidgets import QSpinBox
from PyQt5.QtWidgets import QTabWidget
from PyQt5.QtWidgets import QTableWidget
from PyQt5.QtWidgets import QTableWidgetItem
from PyQt5.QtWidgets import QTextEdit
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from .core import check_tcp_ports
from .core import format_joints
from .core import list_point_names
from .core import load_point
from .core import max_joint_error_deg
from .core import mode_text
from .core import point_image_path
from .core import save_capture_group
from .core import save_point
from .core import validate_point_name
from .camera_capture import Sc3000CameraCapture
from .apriltag_localization import APRILTAG_DICTIONARIES
from .apriltag_localization import AprilTagLocalizer
from .handeye_transform import HandEyeTransform
from .handeye_transform import HandEyeTransformError
from .ros_client import DobotRosClient
from .agv_map_widget import AgvMapWidget
from .task_queue import load_queue
from .task_queue import QueueCommand
from .task_queue import QueueRunResult
from .task_queue import save_queue
from .task_queue import TaskQueueRunner


class AppSignals(QObject):
    task_result = pyqtSignal(int, object)
    task_error = pyqtSignal(int, str)
    log = pyqtSignal(str)
    queue_step = pyqtSignal(int, str, str)
    queue_worker_ended = pyqtSignal()
    preview_frame = pyqtSignal(bytes, float)
    preview_error = pyqtSignal(str)
    preview_ended = pyqtSignal()
    apriltag_result = pyqtSignal(object)


class DobotOperatorWindow(QMainWindow):
    """Operator UI. Every normal robot command is serialized off the UI thread."""

    def __init__(self, ros_client: DobotRosClient) -> None:
        super().__init__()
        self.ros = ros_client
        self.workspace_dir = Path(
            os.environ.get('DOBOT_WS', str(Path.home() / 'dobot_ws'))
        ).expanduser()
        self.points_dir = self.workspace_dir / 'points'
        self.queues_dir = self.workspace_dir / 'queues'
        self.camera_inbox = Path('/dev/shm/dobot_operator_gui_sc3000')
        self.camera = Sc3000CameraCapture()
        self.apriltag_localizer = None
        self._apriltag_localizer_lock = threading.Lock()
        self.handeye_transform = None
        self.handeye_error = ''
        try:
            self.handeye_transform = HandEyeTransform()
        except HandEyeTransformError as exc:
            self.handeye_error = str(exc)
        self._normal_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix='dobot_gui_command'
        )
        self._safety_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix='dobot_gui_safety'
        )
        self._signals = AppSignals()
        self._signals.task_result.connect(self._task_succeeded)
        self._signals.task_error.connect(self._task_failed)
        self._signals.log.connect(self.append_log)
        self._signals.queue_step.connect(self._show_queue_progress)
        self._signals.queue_worker_ended.connect(self._queue_worker_ended)
        self._signals.preview_frame.connect(self._show_preview_frame)
        self._signals.preview_error.connect(self._show_preview_error)
        self._signals.preview_ended.connect(self._preview_worker_ended)
        self._signals.apriltag_result.connect(self._show_apriltag_result)
        self._task_ids = itertools.count(1)
        self._tasks: dict[int, tuple[str, Callable | None, bool, bool]] = {}
        self._normal_busy = False
        self._closing = False
        self._collision_disabled_by_app = False
        self._safe_skin_disabled_by_app = False
        self.queue_commands: list[QueueCommand] = []
        self.queue_cancel = threading.Event()
        self._queue_is_running = False
        self._preview_stop = threading.Event()
        self._preview_pause = threading.Event()
        self._preview_thread: threading.Thread | None = None
        self._preview_frame_times = deque(maxlen=30)
        self._agv_teleop_command = (0.0, 0.0)
        self._agv_teleop_active = False
        self._agv_pressed_button: QPushButton | None = None
        self._agv_map_generation = -1

        self.setWindowTitle('CR5 · SC3000 · SEER AGV 综合上位机')
        self.resize(1220, 800)
        self.setMinimumSize(1050, 700)
        self._apply_style()
        self._build_ui()
        self.refresh_points()

        self.status_timer = QTimer(self)
        self.status_timer.setInterval(2000)
        self.status_timer.timeout.connect(self._automatic_status_refresh)
        self.status_timer.start()
        self.agv_status_timer = QTimer(self)
        self.agv_status_timer.setInterval(500)
        self.agv_status_timer.timeout.connect(self._refresh_agv_status_ui)
        self.agv_status_timer.start()
        self.agv_teleop_timer = QTimer(self)
        self.agv_teleop_timer.setInterval(100)
        self.agv_teleop_timer.timeout.connect(self._publish_agv_teleop)
        self.agv_teleop_timer.start()
        QTimer.singleShot(500, self._initial_status_check)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow { background: #f3f5f7; }
            QFrame#header { background: #17263b; border-radius: 8px; }
            QLabel#title { color: white; font-size: 23px; font-weight: 600; }
            QLabel#subtitle { color: #b9c7d8; }
            QLabel#badge { padding: 6px 12px; border-radius: 10px;
                           background: #32465f; color: white; font-weight: 600; }
            QGroupBox { background: white; border: 1px solid #d6dce3;
                        border-radius: 7px; margin-top: 12px; padding-top: 11px;
                        font-weight: 600; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px;
                               padding: 0 5px; }
            QPushButton { min-height: 30px; padding: 2px 13px;
                          border: 1px solid #aab6c4; border-radius: 5px;
                          background: #f9fafb; }
            QPushButton:hover { background: #eaf2fb; border-color: #357abd; }
            QPushButton:pressed { background: #dbe9f7; }
            QPushButton#primary { color: white; background: #2368a2;
                                  border-color: #2368a2; font-weight: 600; }
            QPushButton#danger { color: white; background: #c62828;
                                 border-color: #8e0000; font-weight: 700;
                                 min-height: 38px; }
            QPushButton#warning { color: #5b3600; background: #ffd98a;
                                  border-color: #d69e2e; font-weight: 600; }
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
                min-height: 28px; border: 1px solid #b9c2cd;
                border-radius: 4px; padding: 0 6px; background: white;
            }
            QListWidget, QTextEdit { border: 1px solid #c7ced7;
                                     border-radius: 5px; background: white; }
            QTabWidget::pane { border: 1px solid #c7ced7; background: #f7f8fa; }
            QTabBar::tab { padding: 9px 19px; background: #e4e8ed; }
            QTabBar::tab:selected { background: white; color: #174d78;
                                    font-weight: 600; }
            """
        )

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(10)

        header = QFrame()
        header.setObjectName('header')
        header_layout = QHBoxLayout(header)
        title_box = QVBoxLayout()
        title = QLabel('CR5 · SC3000 · SEER AGV 综合上位机')
        title.setObjectName('title')
        subtitle = QLabel(
            'ROS 2 Humble · 机械臂/夹爪 · 视觉定位 · AGV导航与低速遥控'
        )
        subtitle.setObjectName('subtitle')
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header_layout.addLayout(title_box)
        header_layout.addStretch()
        self.driver_badge = QLabel('驱动：检查中')
        self.driver_badge.setObjectName('badge')
        self.mode_badge = QLabel('模式：--')
        self.mode_badge.setObjectName('badge')
        header_layout.addWidget(self.driver_badge)
        header_layout.addWidget(self.mode_badge)
        emergency = QPushButton('机械臂紧急停止')
        emergency.setObjectName('danger')
        emergency.clicked.connect(self.emergency_stop)
        header_layout.addWidget(emergency)
        root.addWidget(header)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_connection_tab(), '连接与状态')
        self.tabs.addTab(self._build_robot_tab(), '机械臂控制')
        self.tabs.addTab(self._build_points_tab(), '取点与点位运动')
        self.tabs.addTab(self._build_gripper_tab(), 'DH-AG95夹爪')
        self.tabs.addTab(self._build_queue_tab(), '任务队列')
        self.tabs.addTab(self._build_agv_tab(), 'SEER AGV')
        self.tabs.addTab(self._build_log_tab(), '运行日志')
        root.addWidget(self.tabs, 1)

        self.operation_status = QLabel('就绪')
        self.operation_status.setStyleSheet(
            'padding: 5px; color: #30465d; font-weight: 600;'
        )
        root.addWidget(self.operation_status)
        self.setCentralWidget(central)

    def _build_connection_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        target_group = QGroupBox('连接目标')
        target_layout = QHBoxLayout(target_group)
        target_layout.addWidget(QLabel('机械臂 IP：'))
        self.ip_edit = QLineEdit(os.getenv('IP_address', '192.168.100.6'))
        self.ip_edit.setMaximumWidth(220)
        target_layout.addWidget(self.ip_edit)
        check_button = QPushButton('检查连通性')
        check_button.setObjectName('primary')
        check_button.clicked.connect(self.check_connectivity)
        refresh_button = QPushButton('读取机械臂状态')
        refresh_button.clicked.connect(self.refresh_status)
        target_layout.addWidget(check_button)
        target_layout.addWidget(refresh_button)
        target_layout.addStretch()
        layout.addWidget(target_group)

        status_group = QGroupBox('实时状态')
        grid = QGridLayout(status_group)
        self.network_label = QLabel('尚未检查')
        self.ros_label = QLabel('尚未检查')
        self.mode_label = QLabel('--')
        self.joints_label = QLabel('--')
        self.pose_label = QLabel('--')
        self.joints_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.pose_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        labels = [
            ('控制器端口', self.network_label),
            ('ROS 2 驱动', self.ros_label),
            ('RobotMode', self.mode_label),
            ('关节角 J1~J6', self.joints_label),
            ('末端 X,Y,Z,Rx,Ry,Rz', self.pose_label),
        ]
        for row, (name, value) in enumerate(labels):
            name_label = QLabel(name)
            name_label.setStyleSheet('font-weight: 600; color: #455a70;')
            grid.addWidget(name_label, row, 0, alignment=Qt.AlignTop)
            grid.addWidget(value, row, 1)
        grid.setColumnStretch(1, 1)
        map_group = QGroupBox('实时地图与AGV位置（ROS数据）')
        map_view_layout = QVBoxLayout(map_group)
        self.agv_map_widget = AgvMapWidget()
        self.agv_map_widget.relocationPointSelected.connect(
            self._agv_map_point_selected
        )
        reset_map_view_button = QPushButton('地图视图自动适配')
        reset_map_view_button.clicked.connect(self.agv_map_widget.reset_view)
        map_view_layout.addWidget(self.agv_map_widget, 1)
        map_view_layout.addWidget(reset_map_view_button, alignment=Qt.AlignRight)

        overview_layout = QHBoxLayout()
        overview_layout.addWidget(map_group, 3)
        overview_layout.addWidget(status_group, 2)
        layout.addLayout(overview_layout, 1)

        map_control_group = QGroupBox('地图加载与重定位')
        map_control_layout = QGridLayout(map_control_group)
        self.agv_map_combo = QComboBox()
        self.agv_load_map_button = QPushButton('加载所选地图（2022）')
        self.agv_load_map_button.setObjectName('warning')
        self.agv_load_map_button.setEnabled(False)
        self.agv_load_map_button.clicked.connect(self.load_agv_map)
        refresh_map_button = QPushButton('刷新地图数据（4011）')
        refresh_map_button.clicked.connect(self.refresh_agv_map)
        self.agv_reloc_x_spin = QDoubleSpinBox()
        self.agv_reloc_y_spin = QDoubleSpinBox()
        for spin in (self.agv_reloc_x_spin, self.agv_reloc_y_spin):
            spin.setRange(-10000.0, 10000.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.05)
            spin.setSuffix(' m')
            spin.valueChanged.connect(self._agv_relocation_fields_changed)
        self.agv_reloc_yaw_spin = QDoubleSpinBox()
        self.agv_reloc_yaw_spin.setRange(-180.0, 180.0)
        self.agv_reloc_yaw_spin.setDecimals(1)
        self.agv_reloc_yaw_spin.setSingleStep(5.0)
        self.agv_reloc_yaw_spin.setSuffix('°')
        self.agv_reloc_radius_spin = QDoubleSpinBox()
        self.agv_reloc_radius_spin.setRange(0.05, 5.0)
        self.agv_reloc_radius_spin.setDecimals(2)
        self.agv_reloc_radius_spin.setValue(0.50)
        self.agv_reloc_radius_spin.setSuffix(' m')
        use_current_pose_button = QPushButton('填入当前显示位姿')
        use_current_pose_button.clicked.connect(self._fill_current_agv_pose)
        self.agv_relocalize_button = QPushButton('按指定坐标开始重定位（2002）')
        self.agv_relocalize_button.setObjectName('warning')
        self.agv_relocalize_button.setEnabled(False)
        self.agv_relocalize_button.clicked.connect(self.relocalize_agv)
        self.agv_cancel_relocalization_button = QPushButton('取消正在进行的重定位（2004）')
        self.agv_cancel_relocalization_button.setEnabled(False)
        self.agv_cancel_relocalization_button.clicked.connect(
            self.cancel_agv_relocalization
        )
        map_control_layout.addWidget(QLabel('控制器地图'), 0, 0)
        map_control_layout.addWidget(self.agv_map_combo, 0, 1, 1, 3)
        map_control_layout.addWidget(refresh_map_button, 0, 4)
        map_control_layout.addWidget(self.agv_load_map_button, 0, 5)
        map_control_layout.addWidget(QLabel('重定位 X'), 1, 0)
        map_control_layout.addWidget(self.agv_reloc_x_spin, 1, 1)
        map_control_layout.addWidget(QLabel('Y'), 1, 2)
        map_control_layout.addWidget(self.agv_reloc_y_spin, 1, 3)
        map_control_layout.addWidget(QLabel('车头角'), 1, 4)
        map_control_layout.addWidget(self.agv_reloc_yaw_spin, 1, 5)
        map_control_layout.addWidget(QLabel('搜索半径'), 2, 0)
        map_control_layout.addWidget(self.agv_reloc_radius_spin, 2, 1)
        map_control_layout.addWidget(use_current_pose_button, 2, 2)
        map_control_layout.addWidget(self.agv_relocalize_button, 2, 3, 1, 2)
        map_control_layout.addWidget(self.agv_cancel_relocalization_button, 2, 5)
        layout.addWidget(map_control_group)

        note = QLabel(
            '说明：ROS驱动可用时，上位机不会再主动建立29999/30003/30004探测连接，'
            '以免干扰正在运行的控制会话。'
        )
        note.setWordWrap(True)
        note.setStyleSheet('color: #64748b; padding: 8px;')
        layout.addWidget(note)
        layout.addStretch()
        return page

    def _build_robot_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        enable_group = QGroupBox('使能和速度')
        enable_layout = QGridLayout(enable_group)
        self.load_spin = QDoubleSpinBox()
        self.load_spin.setRange(0.0, 20.0)
        self.load_spin.setDecimals(2)
        self.load_spin.setSuffix(' kg')
        self.load_spin.setValue(1.0)
        self.global_speed_spin = QSpinBox()
        self.global_speed_spin.setRange(1, 100)
        self.global_speed_spin.setSuffix(' %')
        self.global_speed_spin.setValue(5)
        enable_button = QPushButton('按当前负载使能')
        enable_button.setObjectName('primary')
        enable_button.clicked.connect(self.enable_robot)
        disable_button = QPushButton('下使能')
        disable_button.clicked.connect(self.disable_robot)
        clear_button = QPushButton('读取并清除报警')
        clear_button.setObjectName('warning')
        clear_button.clicked.connect(self.clear_error)
        continue_button = QPushButton('安全解除模式10暂停')
        continue_button.setObjectName('warning')
        continue_button.clicked.connect(self.resume_from_pause)
        enable_layout.addWidget(QLabel('末端总负载'), 0, 0)
        enable_layout.addWidget(self.load_spin, 0, 1)
        enable_layout.addWidget(QLabel('全局速度'), 0, 2)
        enable_layout.addWidget(self.global_speed_spin, 0, 3)
        enable_layout.addWidget(enable_button, 1, 0)
        enable_layout.addWidget(disable_button, 1, 1)
        enable_layout.addWidget(clear_button, 1, 2)
        enable_layout.addWidget(continue_button, 1, 3)
        layout.addWidget(enable_group)

        drag_group = QGroupBox('拖动模式')
        drag_layout = QGridLayout(drag_group)
        self.disable_collision_check = QCheckBox('临时关闭本体碰撞检测')
        self.disable_collision_check.setChecked(True)
        self.disable_safe_skin_check = QCheckBox(
            '临时关闭电子皮肤（SafeSkin）'
        )
        self.disable_safe_skin_check.setChecked(False)
        self.disable_safe_skin_check.setToolTip(
            '仅在电子皮肤误触发且现场有人监护时临时使用；默认保持开启'
        )
        self.restore_collision_spin = QSpinBox()
        self.restore_collision_spin.setRange(1, 5)
        self.restore_collision_spin.setValue(3)
        start_drag = QPushButton('进入拖动模式')
        start_drag.setObjectName('warning')
        start_drag.clicked.connect(self.start_drag)
        stop_drag = QPushButton('退出拖动并恢复两项保护')
        stop_drag.setObjectName('primary')
        stop_drag.clicked.connect(self.stop_drag)
        restore_protections = QPushButton('手动恢复两项保护')
        restore_protections.clicked.connect(self.restore_drag_protections)
        drag_layout.addWidget(self.disable_collision_check, 0, 0, 1, 2)
        drag_layout.addWidget(self.disable_safe_skin_check, 0, 2, 1, 2)
        drag_layout.addWidget(QLabel('退出后碰撞等级'), 1, 0)
        drag_layout.addWidget(self.restore_collision_spin, 1, 1)
        drag_layout.addWidget(start_drag, 2, 0)
        drag_layout.addWidget(stop_drag, 2, 1, 1, 2)
        drag_layout.addWidget(restore_protections, 2, 3)
        warning = QLabel(
            '本体碰撞检测与电子皮肤是两套独立保护。关闭电子皮肤后将失去非接触防护；'
            '只允许在工作区清空、有人监护且准备拖动取点时临时关闭。'
        )
        warning.setWordWrap(True)
        warning.setStyleSheet(
            'background:#fff2cc; color:#704b00; padding:10px; border-radius:5px;'
        )
        drag_layout.addWidget(warning, 3, 0, 1, 4)
        layout.addWidget(drag_group)
        layout.addStretch()
        return page

    def _build_points_tab(self) -> QWidget:
        page = QWidget()
        layout = QHBoxLayout(page)

        left = QVBoxLayout()
        directory_group = QGroupBox('点位目录')
        directory_layout = QHBoxLayout(directory_group)
        self.points_dir_edit = QLineEdit(str(self.points_dir))
        browse_button = QPushButton('选择')
        browse_button.clicked.connect(self.choose_points_dir)
        refresh_button = QPushButton('刷新')
        refresh_button.clicked.connect(self.refresh_points)
        directory_layout.addWidget(self.points_dir_edit, 1)
        directory_layout.addWidget(browse_button)
        directory_layout.addWidget(refresh_button)
        left.addWidget(directory_group)

        list_group = QGroupBox('已保存点位')
        list_layout = QVBoxLayout(list_group)
        self.point_list = QListWidget()
        self.point_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.point_list.currentTextChanged.connect(self.show_selected_point)
        list_layout.addWidget(self.point_list)
        left.addWidget(list_group, 1)

        preview_group = QGroupBox('SC3000 实时画面（目标 5 FPS）')
        preview_layout = QVBoxLayout(preview_group)
        self.preview_label = QLabel('点击“开始预览”')
        self.preview_label.setAlignment(Qt.AlignCenter)
        self.preview_label.setMinimumSize(400, 290)
        self.preview_label.setStyleSheet(
            'background:#111820; color:#c8d2dc; border:1px solid #596675;'
        )
        self.preview_status_label = QLabel('状态：已停止；预览帧不写入磁盘')
        apriltag_controls = QGridLayout()
        self.apriltag_enable_check = QCheckBox('AprilTag实时定位')
        self.apriltag_enable_check.setChecked(True)
        self.apriltag_size_spin = QDoubleSpinBox()
        self.apriltag_size_spin.setRange(0.1, 1000.0)
        self.apriltag_size_spin.setDecimals(3)
        self.apriltag_size_spin.setValue(58.5)
        self.apriltag_size_spin.setSuffix(' mm')
        self.apriltag_family_combo = QComboBox()
        self.apriltag_family_combo.addItem('自动识别', 'auto')
        for family in APRILTAG_DICTIONARIES:
            self.apriltag_family_combo.addItem(family, family)
        self.apriltag_family_combo.setCurrentIndex(
            self.apriltag_family_combo.findData('tag36h11')
        )
        self.apriltag_id_spin = QSpinBox()
        self.apriltag_id_spin.setRange(-1, 999999)
        self.apriltag_id_spin.setValue(-1)
        self.apriltag_id_spin.setSpecialValueText('全部ID')
        apriltag_controls.addWidget(self.apriltag_enable_check, 0, 0)
        apriltag_controls.addWidget(QLabel('黑框边长'), 0, 1)
        apriltag_controls.addWidget(self.apriltag_size_spin, 0, 2)
        apriltag_controls.addWidget(QLabel('标签族'), 1, 0)
        apriltag_controls.addWidget(self.apriltag_family_combo, 1, 1)
        apriltag_controls.addWidget(self.apriltag_id_spin, 1, 2)
        if self.handeye_transform is None:
            initial_apriltag_text = (
                'AprilTag：等待预览\n'
                f'手眼变换不可用：{self.handeye_error}'
            )
        else:
            initial_apriltag_text = (
                'AprilTag：等待预览\n'
                '相机坐标：X右 / Y下 / Z向前；基座坐标：CR5 User0'
            )
        self.apriltag_pose_label = QLabel(initial_apriltag_text)
        self.apriltag_pose_label.setWordWrap(True)
        self.apriltag_pose_label.setTextInteractionFlags(
            Qt.TextSelectableByMouse
        )
        self.apriltag_pose_label.setStyleSheet(
            'background:#eef5fb; color:#17364f; padding:5px;'
        )
        preview_buttons = QHBoxLayout()
        self.preview_start_button = QPushButton('开始预览')
        self.preview_stop_button = QPushButton('停止预览')
        self.preview_stop_button.setEnabled(False)
        self.preview_start_button.clicked.connect(self.start_camera_preview)
        self.preview_stop_button.clicked.connect(self.stop_camera_preview)
        preview_buttons.addWidget(self.preview_start_button)
        preview_buttons.addWidget(self.preview_stop_button)
        preview_layout.addWidget(self.preview_label, 1)
        preview_layout.addWidget(self.preview_status_label)
        preview_layout.addLayout(apriltag_controls)
        preview_layout.addWidget(self.apriltag_pose_label)
        preview_layout.addLayout(preview_buttons)
        left.addWidget(preview_group, 2)
        layout.addLayout(left, 2)

        right = QVBoxLayout()
        capture_group = QGroupBox('记录当前点（坐标 + SC3000 图像）')
        capture_layout = QGridLayout(capture_group)
        self.point_name_edit = QLineEdit()
        self.point_name_edit.setPlaceholderText('例如 P3')
        self.camera_ip_edit = QLineEdit('192.168.192.11')
        self.camera_timeout_spin = QDoubleSpinBox()
        self.camera_timeout_spin.setRange(3.0, 60.0)
        self.camera_timeout_spin.setDecimals(1)
        self.camera_timeout_spin.setValue(12.0)
        self.camera_timeout_spin.setSuffix(' 秒')
        coordinate_button = QPushButton('仅保存关节角和位姿')
        coordinate_button.clicked.connect(self.capture_point)
        capture_button = QPushButton('同步拍照并保存整组')
        capture_button.setObjectName('primary')
        capture_button.clicked.connect(self.capture_point_with_image)
        capture_layout.addWidget(QLabel('点名'), 0, 0)
        capture_layout.addWidget(self.point_name_edit, 0, 1, 1, 3)
        capture_layout.addWidget(QLabel('SC3000 IP'), 1, 0)
        capture_layout.addWidget(self.camera_ip_edit, 1, 1)
        capture_layout.addWidget(QLabel('超时'), 1, 2)
        capture_layout.addWidget(self.camera_timeout_spin, 1, 3)
        capture_layout.addWidget(coordinate_button, 2, 0, 1, 2)
        capture_layout.addWidget(capture_button, 2, 2, 1, 2)
        capture_note = QLabel(
            '整组保存到“点位目录/点名/”。程序在拍照前后检查机械臂静止，'
            '预览临时帧位于 /dev/shm，显示后立即删除。'
        )
        capture_note.setWordWrap(True)
        capture_note.setStyleSheet('color:#5d6b78; padding:4px;')
        capture_layout.addWidget(capture_note, 3, 0, 1, 4)
        right.addWidget(capture_group)

        detail_group = QGroupBox('点位详情')
        detail_layout = QFormLayout(detail_group)
        self.selected_joint_label = QLabel('--')
        self.selected_pose_label = QLabel('--')
        self.selected_image_label = QLabel('--')
        self.selected_joint_label.setWordWrap(True)
        self.selected_pose_label.setWordWrap(True)
        self.selected_image_label.setWordWrap(True)
        self.selected_joint_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.selected_pose_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.selected_image_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        detail_layout.addRow('关节角', self.selected_joint_label)
        detail_layout.addRow('笛卡尔位姿', self.selected_pose_label)
        detail_layout.addRow('采集图像', self.selected_image_label)
        right.addWidget(detail_group)

        move_group = QGroupBox('低速移动到选中点')
        move_layout = QGridLayout(move_group)
        self.move_speed_factor_spin = QSpinBox()
        self.move_speed_factor_spin.setRange(1, 100)
        self.move_speed_factor_spin.setValue(5)
        self.move_speed_j_spin = QSpinBox()
        self.move_speed_j_spin.setRange(1, 100)
        self.move_speed_j_spin.setValue(10)
        self.move_acc_j_spin = QSpinBox()
        self.move_acc_j_spin.setRange(1, 100)
        self.move_acc_j_spin.setValue(10)
        self.tolerance_spin = QDoubleSpinBox()
        self.tolerance_spin.setRange(0.05, 5.0)
        self.tolerance_spin.setDecimals(2)
        self.tolerance_spin.setValue(0.5)
        self.tolerance_spin.setSuffix(' °')
        for column, (label, widget) in enumerate(
            [
                ('全局速度 %', self.move_speed_factor_spin),
                ('SpeedJ %', self.move_speed_j_spin),
                ('AccJ %', self.move_acc_j_spin),
                ('到位容差', self.tolerance_spin),
            ]
        ):
            move_layout.addWidget(QLabel(label), 0, column)
            move_layout.addWidget(widget, 1, column)
        move_button = QPushButton('移动到选中点')
        move_button.setObjectName('warning')
        move_button.clicked.connect(self.move_to_selected_point)
        move_layout.addWidget(move_button, 2, 0, 1, 4)
        note = QLabel(
            '运动要求 RobotMode=5。程序不调用 Sync，而是持续读取关节角和模式，'
            '到位且恢复空闲后才判定完成。'
        )
        note.setWordWrap(True)
        note.setStyleSheet('color:#5d6b78; padding:6px;')
        move_layout.addWidget(note, 3, 0, 1, 4)
        right.addWidget(move_group)
        right.addStretch()
        layout.addLayout(right, 3)
        return page

    def _build_gripper_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        connection_group = QGroupBox('末端 RS485 / Modbus RTU')
        grid = QGridLayout(connection_group)
        self.modbus_ip_edit = QLineEdit('127.0.0.1')
        self.modbus_port_spin = QSpinBox()
        self.modbus_port_spin.setRange(1, 65535)
        self.modbus_port_spin.setValue(60000)
        self.slave_id_spin = QSpinBox()
        self.slave_id_spin.setRange(1, 247)
        self.slave_id_spin.setValue(1)
        connect_button = QPushButton('创建通道')
        connect_button.setObjectName('primary')
        connect_button.clicked.connect(self.connect_gripper)
        disconnect_button = QPushButton('关闭通道')
        disconnect_button.clicked.connect(self.disconnect_gripper)
        init_button = QPushButton('初始化夹爪')
        init_button.clicked.connect(self.initialize_gripper)
        read_button = QPushButton('读取状态')
        read_button.clicked.connect(self.read_gripper_status)
        grid.addWidget(QLabel('控制器内部地址'), 0, 0)
        grid.addWidget(self.modbus_ip_edit, 0, 1)
        grid.addWidget(QLabel('端口'), 0, 2)
        grid.addWidget(self.modbus_port_spin, 0, 3)
        grid.addWidget(QLabel('从站ID'), 0, 4)
        grid.addWidget(self.slave_id_spin, 0, 5)
        grid.addWidget(connect_button, 1, 0, 1, 2)
        grid.addWidget(disconnect_button, 1, 2)
        grid.addWidget(init_button, 1, 3)
        grid.addWidget(read_button, 1, 4, 1, 2)
        self.gripper_channel_label = QLabel('通道：未创建')
        self.gripper_channel_label.setStyleSheet('font-weight:600; color:#455a70;')
        grid.addWidget(self.gripper_channel_label, 2, 0, 1, 6)
        layout.addWidget(connection_group)

        control_group = QGroupBox('夹爪控制')
        control_layout = QGridLayout(control_group)
        self.force_spin = QSpinBox()
        self.force_spin.setRange(20, 100)
        self.force_spin.setValue(20)
        self.force_spin.setSuffix(' %')
        apply_force = QPushButton('应用夹持力')
        apply_force.clicked.connect(self.apply_gripper_force)
        self.position_slider = QSlider(Qt.Horizontal)
        self.position_slider.setRange(0, 1000)
        self.position_slider.setValue(1000)
        self.position_spin = QSpinBox()
        self.position_spin.setRange(0, 1000)
        self.position_spin.setValue(1000)
        self.position_slider.valueChanged.connect(self.position_spin.setValue)
        self.position_spin.valueChanged.connect(self.position_slider.setValue)
        move_button = QPushButton('移动到设定开度')
        move_button.clicked.connect(self.move_gripper)
        open_button = QPushButton('完全打开（1000）')
        open_button.setObjectName('primary')
        open_button.clicked.connect(self.open_gripper)
        close_button = QPushButton('低力闭合（0）')
        close_button.setObjectName('warning')
        close_button.clicked.connect(self.close_gripper)
        control_layout.addWidget(QLabel('夹持力'), 0, 0)
        control_layout.addWidget(self.force_spin, 0, 1)
        control_layout.addWidget(apply_force, 0, 2)
        control_layout.addWidget(QLabel('目标开度（0闭合，1000打开）'), 1, 0)
        control_layout.addWidget(self.position_slider, 1, 1, 1, 3)
        control_layout.addWidget(self.position_spin, 1, 4)
        control_layout.addWidget(move_button, 2, 0, 1, 2)
        control_layout.addWidget(open_button, 2, 2)
        control_layout.addWidget(close_button, 2, 3, 1, 2)
        layout.addWidget(control_group)

        state_group = QGroupBox('夹爪反馈寄存器 0x0200~0x0202')
        state_layout = QFormLayout(state_group)
        self.gripper_init_label = QLabel('--')
        self.gripper_grip_label = QLabel('--')
        self.gripper_position_label = QLabel('--')
        state_layout.addRow('初始化状态', self.gripper_init_label)
        state_layout.addRow('夹持状态', self.gripper_grip_label)
        state_layout.addRow('当前位置', self.gripper_position_label)
        layout.addWidget(state_group)

        note = QLabel(
            '夹爪默认按 DH-AG95：站号1、115200、8N1。VX500不得并接占用同一末端RS485 A/B线。'
        )
        note.setWordWrap(True)
        note.setStyleSheet(
            'background:#e9f3ff; color:#244c6a; padding:10px; border-radius:5px;'
        )
        layout.addWidget(note)
        layout.addStretch()
        return page

    def _build_queue_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        builder_group = QGroupBox('添加队列指令')
        builder = QGridLayout(builder_group)
        self.queue_kind_combo = QComboBox()
        for text, kind in (
            ('移动到点位', 'move_point'),
            ('夹爪闭合比例', 'gripper_close_percent'),
            ('夹爪目标位置', 'gripper_position'),
            ('设置夹持力', 'gripper_force'),
            ('初始化夹爪', 'gripper_initialize'),
            ('等待', 'wait'),
        ):
            self.queue_kind_combo.addItem(text, kind)
        self.queue_kind_combo.currentIndexChanged.connect(
            self._queue_kind_changed
        )
        self.queue_point_combo = QComboBox()
        self.queue_value_label = QLabel('数值')
        self.queue_value_spin = QDoubleSpinBox()

        self.queue_speed_factor_spin = QSpinBox()
        self.queue_speed_factor_spin.setRange(1, 100)
        self.queue_speed_factor_spin.setValue(5)
        self.queue_speed_j_spin = QSpinBox()
        self.queue_speed_j_spin.setRange(1, 100)
        self.queue_speed_j_spin.setValue(10)
        self.queue_acc_j_spin = QSpinBox()
        self.queue_acc_j_spin.setRange(1, 100)
        self.queue_acc_j_spin.setValue(10)
        self.queue_tolerance_spin = QDoubleSpinBox()
        self.queue_tolerance_spin.setRange(0.05, 5.0)
        self.queue_tolerance_spin.setDecimals(2)
        self.queue_tolerance_spin.setValue(0.5)
        self.queue_tolerance_spin.setSuffix(' °')
        add_button = QPushButton('添加到队尾')
        add_button.setObjectName('primary')
        add_button.clicked.connect(self.add_queue_command)

        builder.addWidget(QLabel('指令类型'), 0, 0)
        builder.addWidget(self.queue_kind_combo, 1, 0)
        builder.addWidget(QLabel('目标点位'), 0, 1)
        builder.addWidget(self.queue_point_combo, 1, 1)
        builder.addWidget(self.queue_value_label, 0, 2)
        builder.addWidget(self.queue_value_spin, 1, 2)
        builder.addWidget(QLabel('全局速度 %'), 0, 3)
        builder.addWidget(self.queue_speed_factor_spin, 1, 3)
        builder.addWidget(QLabel('SpeedJ %'), 0, 4)
        builder.addWidget(self.queue_speed_j_spin, 1, 4)
        builder.addWidget(QLabel('AccJ %'), 0, 5)
        builder.addWidget(self.queue_acc_j_spin, 1, 5)
        builder.addWidget(QLabel('到位容差'), 0, 6)
        builder.addWidget(self.queue_tolerance_spin, 1, 6)
        builder.addWidget(add_button, 1, 7)
        layout.addWidget(builder_group)

        queue_group = QGroupBox('待执行指令（严格按表格顺序逐条完成）')
        queue_layout = QVBoxLayout(queue_group)
        self.queue_table = QTableWidget(0, 4)
        self.queue_table.setHorizontalHeaderLabels(
            ['序号', '指令', '参数', '状态']
        )
        self.queue_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.queue_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.queue_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.queue_table.verticalHeader().setVisible(False)
        header = self.queue_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.Stretch)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        queue_layout.addWidget(self.queue_table, 1)

        edit_buttons = QHBoxLayout()
        for text, callback in (
            ('上移', self.move_queue_command_up),
            ('下移', self.move_queue_command_down),
            ('删除', self.remove_queue_command),
            ('清空', self.clear_queue),
            ('保存队列', self.save_queue_file),
            ('加载队列', self.load_queue_file),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            edit_buttons.addWidget(button)
        edit_buttons.addStretch()
        self.queue_stop_button = QPushButton('当前步骤后停止')
        self.queue_stop_button.setObjectName('warning')
        self.queue_stop_button.setEnabled(False)
        self.queue_stop_button.clicked.connect(self.stop_queue)
        self.queue_execute_button = QPushButton('确认并执行队列')
        self.queue_execute_button.setObjectName('primary')
        self.queue_execute_button.clicked.connect(self.execute_queue)
        edit_buttons.addWidget(self.queue_stop_button)
        edit_buttons.addWidget(self.queue_execute_button)
        queue_layout.addLayout(edit_buttons)
        layout.addWidget(queue_group, 1)

        note = QLabel(
            'DH-AG95 是直线夹爪，不使用角度指令：“闭合比例”0%表示完全打开，'
            '100%表示完全闭合。执行前需先在夹爪页创建Modbus通道并完成初始化。'
            '“当前步骤后停止”不会中途截断机械臂动作；需要立即停机请使用'
            '红色急停或实体急停。'
        )
        note.setWordWrap(True)
        note.setStyleSheet(
            'background:#fff2cc; color:#704b00; padding:10px; border-radius:5px;'
        )
        layout.addWidget(note)
        self._queue_kind_changed()
        return page

    def _build_agv_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        status_group = QGroupBox('AGV实时状态（来自ROS驱动，不建立额外TCP连接）')
        status_layout = QGridLayout(status_group)
        self.agv_driver_label = QLabel('驱动：等待状态')
        self.agv_safety_label = QLabel('安全门：--')
        self.agv_pose_label = QLabel('X=--  Y=--  Yaw=--')
        self.agv_motion_label = QLabel('导航=--  速度=--')
        self.agv_battery_label = QLabel('电池=--')
        self.agv_owner_label = QLabel('急停/充电/控制权=--')
        self.agv_safety_label.setWordWrap(True)
        status_rows = [
            ('ROS驱动', self.agv_driver_label),
            ('运动安全门', self.agv_safety_label),
            ('地图位姿', self.agv_pose_label),
            ('导航与速度', self.agv_motion_label),
            ('电池', self.agv_battery_label),
            ('附加安全状态', self.agv_owner_label),
        ]
        for row, (name, value) in enumerate(status_rows):
            label = QLabel(name)
            label.setStyleSheet('font-weight:600; color:#455a70;')
            status_layout.addWidget(label, row, 0, alignment=Qt.AlignTop)
            status_layout.addWidget(value, row, 1)
        self.agv_confirm_localization_button = QPushButton(
            '核对地图位姿后，确认定位正确（API 2003）'
        )
        self.agv_confirm_localization_button.setObjectName('warning')
        self.agv_confirm_localization_button.setEnabled(False)
        self.agv_confirm_localization_button.clicked.connect(
            self.confirm_agv_localization
        )
        status_layout.addWidget(
            self.agv_confirm_localization_button, len(status_rows), 0, 1, 2
        )
        status_layout.setColumnStretch(1, 1)
        layout.addWidget(status_group)

        navigation_group = QGroupBox('低速站点导航')
        navigation_layout = QGridLayout(navigation_group)
        self.agv_station_combo = QComboBox()
        self.agv_nav_speed_spin = QDoubleSpinBox()
        self.agv_nav_speed_spin.setDecimals(2)
        self.agv_nav_speed_spin.setRange(0.01, 0.08)
        self.agv_nav_speed_spin.setSingleStep(0.01)
        self.agv_nav_speed_spin.setSuffix(' m/s')
        self.agv_nav_speed_spin.setValue(0.08)
        self.agv_navigate_button = QPushButton('导航到所选站点')
        self.agv_navigate_button.setObjectName('primary')
        self.agv_navigate_button.clicked.connect(self.navigate_agv)
        self.agv_cancel_button = QPushButton('取消当前导航')
        self.agv_cancel_button.clicked.connect(self.cancel_agv_navigation)
        navigation_layout.addWidget(QLabel('目标站点'), 0, 0)
        navigation_layout.addWidget(self.agv_station_combo, 0, 1)
        navigation_layout.addWidget(QLabel('最大速度'), 0, 2)
        navigation_layout.addWidget(self.agv_nav_speed_spin, 0, 3)
        navigation_layout.addWidget(self.agv_navigate_button, 1, 0, 1, 2)
        navigation_layout.addWidget(self.agv_cancel_button, 1, 2)
        navigation_layout.addWidget(QLabel('地图和站点由上方地图区域统一刷新'), 1, 3)
        layout.addWidget(navigation_group)

        teleop_group = QGroupBox('低速点动（差速底盘，仅前后和旋转）')
        teleop_layout = QGridLayout(teleop_group)
        self.agv_teleop_enable = QCheckBox('人工确认现场安全后启用点动')
        self.agv_teleop_enable.toggled.connect(self._toggle_agv_teleop)
        self.agv_forward_speed_spin = QDoubleSpinBox()
        self.agv_forward_speed_spin.setRange(0.01, 0.05)
        self.agv_forward_speed_spin.setDecimals(2)
        self.agv_forward_speed_spin.setValue(0.03)
        self.agv_forward_speed_spin.setSuffix(' m/s')
        self.agv_backward_speed_spin = QDoubleSpinBox()
        self.agv_backward_speed_spin.setRange(0.01, 0.05)
        self.agv_backward_speed_spin.setDecimals(2)
        self.agv_backward_speed_spin.setValue(0.03)
        self.agv_backward_speed_spin.setSuffix(' m/s')
        self.agv_turn_speed_spin = QDoubleSpinBox()
        self.agv_turn_speed_spin.setRange(0.02, 0.15)
        self.agv_turn_speed_spin.setDecimals(2)
        self.agv_turn_speed_spin.setValue(0.10)
        self.agv_turn_speed_spin.setSuffix(' rad/s')
        teleop_layout.addWidget(self.agv_teleop_enable, 0, 0, 1, 4)
        teleop_layout.addWidget(QLabel('前进速度'), 1, 0)
        teleop_layout.addWidget(self.agv_forward_speed_spin, 1, 1)
        teleop_layout.addWidget(QLabel('后退速度'), 1, 2)
        teleop_layout.addWidget(self.agv_backward_speed_spin, 1, 3)
        teleop_layout.addWidget(QLabel('旋转速度'), 2, 0)
        teleop_layout.addWidget(self.agv_turn_speed_spin, 2, 1)

        left_button = QPushButton('按住左转')
        forward_button = QPushButton('按住前进')
        right_button = QPushButton('按住右转')
        backward_button = QPushButton('按住后退')
        for button, command in (
            (left_button, 'left'),
            (forward_button, 'forward'),
            (right_button, 'right'),
            (backward_button, 'backward'),
        ):
            button.pressed.connect(
                lambda direction=command, pressed_button=button:
                self._start_agv_teleop(direction, pressed_button)
            )
            button.released.connect(self._stop_agv_teleop)
        self.agv_teleop_buttons = {
            'left': left_button,
            'forward': forward_button,
            'backward': backward_button,
            'right': right_button,
        }
        teleop_layout.addWidget(left_button, 3, 0)
        teleop_layout.addWidget(forward_button, 3, 1)
        teleop_layout.addWidget(backward_button, 3, 2)
        teleop_layout.addWidget(right_button, 3, 3)
        self.agv_teleop_feedback_label = QLabel('点动指令：待命')
        self.agv_teleop_feedback_label.setWordWrap(True)
        teleop_layout.addWidget(self.agv_teleop_feedback_label, 4, 0, 1, 4)

        agv_stop_button = QPushButton('AGV立即停止（2000）')
        agv_stop_button.setObjectName('danger')
        agv_stop_button.clicked.connect(self.stop_agv)
        teleop_layout.addWidget(agv_stop_button, 5, 0, 1, 4)
        layout.addWidget(teleop_group)

        note = QLabel(
            '导航与点动互斥；导航运行时点动安全门自动关闭。按钮松开会发布零速度，'
            '驱动看门狗超时也会发送2000停止。当前Robokit 3.4.4.6只有定位状态1才放行；'
            '状态3时必须现场核对地图位置和车头方向，并使用上方受保护的API 2003按钮确认。'
        )
        note.setWordWrap(True)
        note.setStyleSheet(
            'background:#fff2cc; color:#704b00; padding:10px; border-radius:5px;'
        )
        layout.addWidget(note)
        layout.addStretch()
        return page

    def _build_log_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.document().setMaximumBlockCount(1000)
        clear_button = QPushButton('清空界面日志')
        clear_button.clicked.connect(self.log_edit.clear)
        layout.addWidget(self.log_edit, 1)
        layout.addWidget(clear_button, alignment=Qt.AlignRight)
        return page

    def append_log(self, text: str) -> None:
        self.log_edit.append(text)
        self.operation_status.setText(text)

    def _submit(
        self,
        label: str,
        function: Callable,
        on_success: Callable | None = None,
        *,
        safety: bool = False,
        quiet: bool = False,
    ) -> bool:
        if self._closing:
            return False
        if not safety and self._normal_busy:
            if not quiet:
                self.append_log('已有操作正在执行，请等待完成或使用紧急停止')
            return False
        task_id = next(self._task_ids)
        self._tasks[task_id] = (label, on_success, quiet, safety)
        if not safety:
            self._normal_busy = True
        if not quiet:
            self.append_log(f'▶ {label}')
        executor = self._safety_executor if safety else self._normal_executor
        future = executor.submit(function)
        future.add_done_callback(
            lambda result_future, tid=task_id: self._future_done(tid, result_future)
        )
        return True

    def _future_done(self, task_id: int, future: Future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self._signals.task_error.emit(task_id, f'{type(exc).__name__}: {exc}')
        else:
            self._signals.task_result.emit(task_id, result)

    def _task_succeeded(self, task_id: int, result: object) -> None:
        task = self._tasks.pop(task_id, None)
        if task is None:
            return
        label, callback, quiet, safety = task
        if not safety:
            self._normal_busy = False
        if not quiet:
            self.append_log(f'✓ {label}完成')
        if callback is not None:
            callback(result)
        if not quiet:
            QTimer.singleShot(150, lambda: self.refresh_status(quiet=True))

    def _task_failed(self, task_id: int, error: str) -> None:
        task = self._tasks.pop(task_id, None)
        if task is None:
            return
        label, unused_callback, quiet, safety = task
        if not safety:
            self._normal_busy = False
        if quiet:
            self.driver_badge.setText('驱动：状态读取失败')
            self.driver_badge.setStyleSheet('background:#a33a3a;')
            return
        self.append_log(f'✗ {label}失败：{error}')
        if not quiet and not self._closing:
            QMessageBox.critical(self, f'{label}失败', error)

    def _automatic_status_refresh(self) -> None:
        if not self._normal_busy and self.ros.driver_ready():
            self.refresh_status(quiet=True)

    def _initial_status_check(self) -> None:
        if self.ros.driver_ready():
            self.refresh_status(quiet=True)
        else:
            self.driver_badge.setText('驱动：不可用')
            self.driver_badge.setStyleSheet('background:#a33a3a;')
            self.ros_label.setText('未发现 dobot_bringup_v3；可点击“检查连通性”')

    def check_connectivity(self) -> None:
        host = self.ip_edit.text().strip()
        if not host:
            QMessageBox.warning(self, '输入错误', '请输入机械臂 IP 地址')
            return

        def operation():
            ready, total = self.ros.available_services()
            if self.ros.driver_ready():
                try:
                    status = self.ros.read_status()
                    status_error = None
                except Exception as exc:
                    status = None
                    status_error = f'{type(exc).__name__}: {exc}'
                return {
                    'driver_ready': True,
                    'services': (ready, total),
                    'ports': None,
                    'status': status,
                    'status_error': status_error,
                }
            return {
                'driver_ready': False,
                'services': (ready, total),
                'ports': check_tcp_ports(host),
                'status': None,
                'status_error': None,
            }

        self._submit('检查连通性', operation, self._show_connectivity)

    def _show_connectivity(self, result: dict) -> None:
        ready, total = result['services']
        if result['driver_ready']:
            self.network_label.setText('由ROS服务连接验证；已跳过额外TCP探测')
            if result['status'] is not None:
                self.driver_badge.setText('驱动：已连接')
                self.driver_badge.setStyleSheet('background:#1b7f50;')
                self.ros_label.setText(
                    f'可用（发现 {ready}/{total} 个所需服务）'
                )
                self._show_status(result['status'])
            else:
                self.driver_badge.setText('驱动：通信异常')
                self.driver_badge.setStyleSheet('background:#a33a3a;')
                self.ros_label.setText(
                    f'服务存在但控制器状态读取失败：{result["status_error"]}'
                )
            return
        self.driver_badge.setText('驱动：不可用')
        self.driver_badge.setStyleSheet('background:#a33a3a;')
        self.ros_label.setText(f'不可用（发现 {ready}/{total} 个所需服务）')
        ports = result['ports'] or {}
        self.network_label.setText(
            '；'.join(f'{port}: {state}' for port, state in ports.items())
        )

    def refresh_status(self, quiet: bool = False) -> None:
        self._submit(
            '读取机械臂状态',
            self.ros.read_status,
            self._show_status,
            quiet=quiet,
        )

    def _show_status(self, status: dict) -> None:
        mode = str(status['mode'])
        self.mode_label.setText(mode_text(mode))
        self.mode_badge.setText(f'模式：{mode}')
        if mode == '5':
            self.mode_badge.setStyleSheet('background:#1b7f50;')
        elif mode in {'9', '11'}:
            self.mode_badge.setStyleSheet('background:#a33a3a;')
        elif mode == '6':
            self.mode_badge.setStyleSheet('background:#9b6800;')
        else:
            self.mode_badge.setStyleSheet('background:#32465f;')
        self.joints_label.setText(format_joints(status['angles']))
        self.pose_label.setText(
            ', '.join(f'{value:.3f}' for value in status['pose'])
        )
        self.driver_badge.setText('驱动：已连接')
        self.driver_badge.setStyleSheet('background:#1b7f50;')

    @staticmethod
    def _agv_number(value, decimals: int = 3) -> str:
        try:
            return f'{float(value):.{decimals}f}'
        except (TypeError, ValueError):
            return '--'

    def _refresh_agv_status_ui(self) -> None:
        status = self.ros.agv_status()
        ready = self.ros.agv_driver_ready()
        connected = bool(status.get('connected')) and ready
        if connected:
            self.agv_driver_label.setText('驱动：已连接（单一TCP所有者）')
            self.agv_driver_label.setStyleSheet('color:#1b7f50; font-weight:600;')
        else:
            self.agv_driver_label.setText('驱动：不可用或状态过期')
            self.agv_driver_label.setStyleSheet('color:#a33a3a; font-weight:600;')

        safe = bool(status.get('safe_to_move'))
        teleop_safe = bool(status.get('safe_for_teleop'))
        nav_safe = bool(status.get('safe_to_start_navigation'))
        reason = str(status.get('safety_reason') or '允许运动')
        teleop_reason = str(status.get('teleop_reason') or '允许低速点动')
        self.agv_safety_label.setText(
            ('允许运动' if safe else f'禁止运动：{reason}')
            + f'；点动：{teleop_reason}'
        )
        self.agv_safety_label.setStyleSheet(
            'color:#1b7f50; font-weight:600;'
            if safe
            else 'color:#a33a3a; font-weight:600;'
        )

        pose = status.get('pose') if isinstance(status.get('pose'), dict) else {}
        yaw = pose.get('angle')
        if yaw is None:
            yaw = pose.get('yaw')
        self.agv_pose_label.setText(
            f"X={self._agv_number(pose.get('x'))} m  "
            f"Y={self._agv_number(pose.get('y'))} m  "
            f"Yaw={self._agv_number(yaw)} rad  "
            f"站点={pose.get('current_station') or '--'}"
        )

        nav_status = status.get('nav_status')
        nav_text = {
            0: '无任务',
            1: '等待',
            2: '运行中',
            3: '暂停',
            4: '完成',
            5: '失败',
            6: '取消',
        }.get(nav_status, str(nav_status if nav_status is not None else '--'))
        speed = (
            status.get('speed') if isinstance(status.get('speed'), dict) else {}
        )
        command = (
            status.get('command')
            if isinstance(status.get('command'), dict)
            else {}
        )
        self.agv_motion_label.setText(
            f"导航={nav_text}；vx={self._agv_number(speed.get('vx'), 4)} m/s；"
            f"w={self._agv_number(speed.get('w'), 4)} rad/s；"
            f"指令vx={self._agv_number(command.get('sent_vx'), 3)}；"
            f"指令w={self._agv_number(command.get('sent_w'), 3)}"
        )
        self.agv_map_widget.set_pose(pose)
        generation, map_data = self.ros.agv_map_data(self._agv_map_generation)
        if map_data is not None:
            self._agv_map_generation = generation
            self.agv_map_widget.set_map_data(map_data)
            maps = map_data.get('maps', [])
            if not isinstance(maps, list):
                maps = []
            map_names = [str(name) for name in maps]
            existing_maps = [
                self.agv_map_combo.itemText(index)
                for index in range(self.agv_map_combo.count())
            ]
            current_map = str(map_data.get('current_map') or '')
            if map_names != existing_maps:
                self.agv_map_combo.clear()
                self.agv_map_combo.addItems(map_names)
            current_index = self.agv_map_combo.findText(current_map)
            if current_index >= 0:
                self.agv_map_combo.setCurrentIndex(current_index)

        battery = (
            status.get('battery')
            if isinstance(status.get('battery'), dict)
            else {}
        )
        level = battery.get('battery_level')
        try:
            level_text = f'{float(level) * 100.0:.1f}%'
        except (TypeError, ValueError):
            level_text = '--'
        self.agv_battery_label.setText(
            f"电量={level_text}；电压={self._agv_number(battery.get('voltage'), 2)} V；"
            f"温度={self._agv_number(battery.get('battery_temp'), 1)} ℃"
        )
        owner = (
            status.get('control_owner')
            if isinstance(status.get('control_owner'), dict)
            else {}
        )
        owner_text = owner.get('nick_name') or owner.get('ip') or '无抢占'
        self.agv_owner_label.setText(
            f"急停={'激活' if status.get('emergency_active') else '正常'}；"
            f"充电={'是' if status.get('charging') else '否'}；"
            f"控制权={owner_text}"
        )

        stations = self.ros.agv_stations()
        station_ids = [str(item['id']) for item in stations]
        current_station = self.agv_station_combo.currentText()
        existing = [
            self.agv_station_combo.itemText(index)
            for index in range(self.agv_station_combo.count())
        ]
        if station_ids != existing:
            self.agv_station_combo.clear()
            self.agv_station_combo.addItems(station_ids)
            index = self.agv_station_combo.findText(current_station)
            if index >= 0:
                self.agv_station_combo.setCurrentIndex(index)

        self.agv_navigate_button.setEnabled(connected and nav_safe)
        self.agv_cancel_button.setEnabled(
            connected and nav_status in {1, 2, 3}
        )
        self.agv_teleop_enable.setEnabled(connected and teleop_safe)
        self.agv_load_map_button.setEnabled(connected)
        self.agv_relocalize_button.setEnabled(
            connected and bool(status.get('map_loaded'))
        )
        self.agv_cancel_relocalization_button.setEnabled(
            connected
            and isinstance(status.get('loc_status'), dict)
            and status['loc_status'].get('reloc_status') == 2
        )
        confirm_ready = (
            connected
            and bool(status.get('localization_pending_confirmation'))
            and bool(status.get('map_loaded'))
            and not bool(status.get('emergency_active'))
            and not bool(status.get('charging'))
            and not bool(status.get('control_locked'))
            and not bool(status.get('blocked'))
            and not bool(status.get('slowed'))
            and not bool(status.get('has_alarm'))
            and nav_status not in {1, 2, 3}
        )
        self.agv_confirm_localization_button.setEnabled(confirm_ready)
        if self.agv_teleop_enable.isChecked() and not teleop_safe:
            if status.get('slowed') and self._agv_teleop_command[0] > 0.0:
                self.agv_teleop_feedback_label.setText(
                    '前进已由AGV前向安全减速区拦截并停止；请清除前方障碍物，'
                    '不要绕过安全传感器。'
                )
            self.agv_teleop_enable.blockSignals(True)
            self.agv_teleop_enable.setChecked(False)
            self.agv_teleop_enable.blockSignals(False)
            self._stop_agv_teleop()

    def _toggle_agv_teleop(self, checked: bool) -> None:
        if not checked:
            self._stop_agv_teleop()
            self.append_log('AGV低速点动已关闭')
            return
        status = self.ros.agv_status()
        if not status.get('safe_for_teleop'):
            QMessageBox.warning(
                self,
                'AGV点动被安全门拒绝',
                str(status.get('teleop_reason') or 'AGV状态不可用'),
            )
            self.agv_teleop_enable.blockSignals(True)
            self.agv_teleop_enable.setChecked(False)
            self.agv_teleop_enable.blockSignals(False)
            return
        if QMessageBox.warning(
            self,
            '确认启用AGV点动',
            '确认AGV四周和预计运动方向内没有人员、线缆及障碍物，并可随时按下实体急停。\n\n'
            '导航和点动不能同时进行。是否启用按住运动？',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        ) != QMessageBox.Yes:
            self.agv_teleop_enable.blockSignals(True)
            self.agv_teleop_enable.setChecked(False)
            self.agv_teleop_enable.blockSignals(False)
            return
        self.append_log('AGV低速点动已启用；仅在按住按钮期间发送速度')

    def _start_agv_teleop(
        self, direction: str, pressed_button: QPushButton
    ) -> None:
        if not self.agv_teleop_enable.isChecked():
            self.append_log('请先勾选并确认启用AGV低速点动')
            return
        status = self.ros.agv_status()
        if not status.get('safe_for_teleop'):
            self.append_log(
                f"AGV点动被拒绝：{status.get('teleop_reason', '状态不可用')}"
            )
            self._stop_agv_teleop()
            return
        commands = {
            'forward': (self.agv_forward_speed_spin.value(), 0.0),
            'backward': (-self.agv_backward_speed_spin.value(), 0.0),
            'left': (0.0, self.agv_turn_speed_spin.value()),
            'right': (0.0, -self.agv_turn_speed_spin.value()),
        }
        self._agv_teleop_command = commands[direction]
        self._agv_teleop_active = True
        self._agv_pressed_button = pressed_button
        vx, angular_z = self._agv_teleop_command
        self.agv_teleop_feedback_label.setText(
            f'正在发送{direction}：vx={vx:.3f} m/s，w={angular_z:.3f} rad/s'
        )
        self._publish_agv_teleop()

    def _publish_agv_teleop(self) -> None:
        if (
            not self._agv_teleop_active
            or not self.agv_teleop_enable.isChecked()
            or self._closing
        ):
            return
        if self._agv_pressed_button is None or not self._agv_pressed_button.isDown():
            self._stop_agv_teleop()
            return
        vx, angular_z = self._agv_teleop_command
        self.ros.agv_publish_velocity(vx, angular_z)

    def _stop_agv_teleop(self) -> None:
        self._agv_teleop_active = False
        self._agv_teleop_command = (0.0, 0.0)
        self._agv_pressed_button = None
        if hasattr(self, 'agv_teleop_feedback_label'):
            self.agv_teleop_feedback_label.setText('点动指令：已停止（已发布零速度）')
        self.ros.agv_publish_velocity(0.0, 0.0)

    def _agv_map_point_selected(self, x: float, y: float) -> None:
        self.agv_reloc_x_spin.setValue(x)
        self.agv_reloc_y_spin.setValue(y)
        self.append_log(f'已从地图选择重定位坐标：X={x:.3f} m，Y={y:.3f} m')

    def _agv_relocation_fields_changed(self) -> None:
        self.agv_map_widget.set_relocation_selection(
            self.agv_reloc_x_spin.value(), self.agv_reloc_y_spin.value()
        )

    def _fill_current_agv_pose(self) -> None:
        status = self.ros.agv_status()
        pose = status.get('pose') if isinstance(status.get('pose'), dict) else {}
        yaw = pose.get('angle')
        if yaw is None:
            yaw = pose.get('yaw')
        try:
            x = float(pose.get('x'))
            y = float(pose.get('y'))
            yaw = float(yaw)
        except (TypeError, ValueError):
            QMessageBox.warning(self, '位姿不可用', '尚未收到有效的AGV地图位姿')
            return
        self.agv_reloc_x_spin.setValue(x)
        self.agv_reloc_y_spin.setValue(y)
        self.agv_reloc_yaw_spin.setValue(float(np.degrees(yaw)))

    def load_agv_map(self) -> None:
        status = self.ros.agv_status()
        if not status.get('connected'):
            QMessageBox.warning(self, 'AGV状态不可用', '无法在状态过期时加载地图')
            return
        map_name = self.agv_map_combo.currentText().strip()
        if not map_name:
            QMessageBox.warning(self, '未选择地图', '请先刷新并选择控制器中的地图')
            return
        current_map = str(status.get('current_map') or '')
        if QMessageBox.warning(
            self,
            '确认加载AGV地图',
            f'当前地图：{current_map or "--"}\n目标地图：{map_name}\n\n'
            '切换地图会使当前定位失效，并关闭导航与点动安全门。加载完成后必须在新地图上'
            '重新定位、核对车体位置和车头方向，再确认定位正确。\n\n是否继续？',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        ) != QMessageBox.Yes:
            return
        self._stop_agv_teleop()
        self._submit(
            f'加载AGV地图 {map_name}',
            lambda: self.ros.agv_load_map(map_name),
            lambda result: self.append_log(f'AGV地图加载完成：{result}'),
        )

    def relocalize_agv(self) -> None:
        status = self.ros.agv_status()
        if not status.get('connected') or not status.get('map_loaded'):
            QMessageBox.warning(
                self, '无法重定位', '需要AGV状态新鲜且当前地图已经加载'
            )
            return
        x = self.agv_reloc_x_spin.value()
        y = self.agv_reloc_y_spin.value()
        yaw_degrees = self.agv_reloc_yaw_spin.value()
        yaw = float(np.radians(yaw_degrees))
        radius = self.agv_reloc_radius_spin.value()
        if QMessageBox.warning(
            self,
            '核对AGV重定位初值',
            f'地图：{status.get("current_map") or "--"}\n'
            f'X={x:.3f} m，Y={y:.3f} m\n'
            f'车头角={yaw_degrees:.1f}°，搜索半径={radius:.2f} m\n\n'
            '请确认该坐标和车头方向接近AGV在现场的真实位置。错误初值可能得到错误定位。',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        ) != QMessageBox.Yes:
            return
        if QMessageBox.question(
            self,
            '最后确认开始重定位',
            '我已现场核对AGV实际位置和车头方向，确认与输入初值相符。\n'
            '是否发送API 2002？重定位完成后仍需再次确认定位正确。',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        ) != QMessageBox.Yes:
            return
        self._stop_agv_teleop()
        self._submit(
            'AGV重定位（API 2002）',
            lambda: self.ros.agv_relocalize(x, y, yaw, radius),
            lambda result: self.append_log(f'AGV重定位已受理：{result}'),
        )

    def cancel_agv_relocalization(self) -> None:
        if QMessageBox.question(
            self,
            '确认取消重定位',
            '取消当前重定位并恢复到重定位前的位置？',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        ) != QMessageBox.Yes:
            return
        self._submit(
            '取消AGV重定位',
            self.ros.agv_cancel_relocalization,
            safety=True,
        )

    def navigate_agv(self) -> None:
        station_id = self.agv_station_combo.currentText().strip()
        if not station_id:
            QMessageBox.warning(self, '未选择站点', '请先刷新并选择AGV目标站点')
            return
        status = self.ros.agv_status()
        if not status.get('safe_to_start_navigation'):
            QMessageBox.warning(
                self,
                'AGV导航被安全门拒绝',
                str(status.get('teleop_reason') or 'AGV状态不可用'),
            )
            return
        speed = self.agv_nav_speed_spin.value()
        if QMessageBox.warning(
            self,
            '确认AGV站点导航',
            f'目标站点：{station_id}\n最大速度：{speed:.2f} m/s\n\n'
            '确认完整路径及AGV周围没有人员、线缆和障碍物？',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        ) != QMessageBox.Yes:
            return
        if self.agv_teleop_enable.isChecked():
            self.agv_teleop_enable.setChecked(False)
        self._submit(
            f'AGV导航到 {station_id}',
            lambda: self.ros.agv_navigate_to_station(station_id, speed),
            lambda result: self.append_log(
                f'AGV导航任务已接受：task_id={result[0]}；{result[1]}'
            ),
        )

    def confirm_agv_localization(self) -> None:
        status = self.ros.agv_status()
        if not status.get('connected'):
            QMessageBox.warning(self, 'AGV状态不可用', '尚未收到新鲜的AGV状态')
            return
        if not status.get('localization_pending_confirmation'):
            QMessageBox.information(
                self,
                '无需确认',
                '当前定位状态不是“等待操作员确认”（reloc_status不为3）。',
            )
            return
        pose = status.get('pose') if isinstance(status.get('pose'), dict) else {}
        yaw = pose.get('angle')
        if yaw is None:
            yaw = pose.get('yaw')
        try:
            x = float(pose.get('x'))
            y = float(pose.get('y'))
            yaw = float(yaw)
        except (TypeError, ValueError):
            QMessageBox.warning(self, '位姿不可用', '无法读取有效的AGV X/Y/Yaw')
            return
        if not all(np.isfinite(value) for value in (x, y, yaw)):
            QMessageBox.warning(self, '位姿不可用', 'AGV位姿包含非有限数值')
            return

        pose_text = (
            f'X = {x:.3f} m\nY = {y:.3f} m\n'
            f'Yaw = {yaw:.3f} rad（{np.degrees(yaw):.1f}°）'
        )
        if QMessageBox.warning(
            self,
            '核对AGV地图位姿',
            '此操作会把当前地图中的定位结果确认为现场真实位置，并可能随后开放'
            '导航和点动安全门。\n\n'
            f'{pose_text}\n\n'
            '请同时观察现场AGV和地图图标，确认位置及车头方向完全一致。若不一致，'
            '必须取消并重新定位。',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        ) != QMessageBox.Yes:
            return
        if QMessageBox.question(
            self,
            '最后确认定位正确',
            f'{pose_text}\n\n'
            '我已在现场核对AGV实际位置和车头方向，并确认与上述地图位姿一致。\n'
            '是否发送API 2003？',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        ) != QMessageBox.Yes:
            return

        self._submit(
            '确认AGV定位正确（API 2003）',
            lambda: self.ros.agv_confirm_localization(x, y, yaw),
            lambda result: self.append_log(f'AGV定位已确认：{result}'),
        )

    def cancel_agv_navigation(self) -> None:
        if QMessageBox.question(
            self,
            '确认取消AGV导航',
            '取消当前导航并发送2000停止？',
        ) != QMessageBox.Yes:
            return
        self._submit(
            '取消AGV导航并停止',
            self.ros.agv_cancel_navigation,
            safety=True,
        )

    def stop_agv(self) -> None:
        self._stop_agv_teleop()
        self.agv_teleop_enable.blockSignals(True)
        self.agv_teleop_enable.setChecked(False)
        self.agv_teleop_enable.blockSignals(False)
        self._submit('AGV立即停止', self.ros.agv_stop, safety=True)

    def refresh_agv_map(self) -> None:
        self._submit('刷新AGV地图和站点', self.ros.agv_download_map)

    def enable_robot(self) -> None:
        load = self.load_spin.value()
        speed = self.global_speed_spin.value()
        answer = QMessageBox.question(
            self,
            '确认使能',
            f'确认工作区安全，并以末端总负载 {load:.2f} kg、'
            f'全局速度 {speed}% 使能机械臂？',
        )
        if answer != QMessageBox.Yes:
            return
        self._submit(
            '使能机械臂',
            lambda: self.ros.enable_robot(load, speed),
            lambda mode: self.append_log(f'当前模式：{mode_text(mode)}'),
        )

    def disable_robot(self) -> None:
        if QMessageBox.question(
            self, '确认下使能', '确认机械臂已经停止，执行下使能？'
        ) != QMessageBox.Yes:
            return
        self._submit('机械臂下使能', self.ros.disable_robot)

    def clear_error(self) -> None:
        self._submit(
            '读取报警码',
            self.ros.get_error_ids,
            self._confirm_clear_error,
        )

    def _confirm_clear_error(self, error_ids: str) -> None:
        safe_skin_alarm = re.search(r'(?<!\d)-3(?!\d)', error_ids) is not None
        if safe_skin_alarm:
            message = (
                f'检测到电子皮肤碰撞报警 -3：\n{error_ids}\n\n'
                '先让人员、障碍物和线缆离开电子皮肤检测区域，并扶稳机械臂。'
                '如果报警发生在拖动模式，清除后机械臂可能立即回到 Mode 6，'
                '此时关节可再次被拖动。\n\n确认现场安全并清除吗？'
            )
        else:
            message = (
                f'当前报警码：{error_ids}\n\n'
                '只有在报警原因已经排除、机械臂工作区安全时才能清除。继续吗？'
            )
        if QMessageBox.warning(
            self,
            '确认清除报警',
            message,
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        ) != QMessageBox.Yes:
            return
        self._submit(
            '清除报警',
            self.ros.clear_error,
            self._error_cleared,
        )

    def _error_cleared(self, result: tuple[str, str]) -> None:
        before, mode = result
        self.append_log(
            f'清除前报警码：{before}；清除后模式：{mode_text(mode)}'
        )
        if mode == '6':
            QMessageBox.warning(
                self,
                '机械臂已回到拖动模式',
                '清除报警后机械臂处于 Mode 6，关节现在可以被拖动。'
                '请继续扶稳机械臂；结束取点后点击“退出拖动并恢复两项保护”。',
            )

    def resume_from_pause(self) -> None:
        if QMessageBox.warning(
            self,
            '确认解除模式10暂停',
            'Continue 可能立即恢复控制器中遗留的运动队列。\n\n'
            '请先在示教器确认没有未知任务，并确认机械臂整条路径内没有人员、'
            '线缆和障碍物。上位机会先检查 RobotMode=10 且报警码为空。\n\n'
            '确认继续吗？',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        ) != QMessageBox.Yes:
            return

        def succeeded(mode):
            if mode == '7':
                QMessageBox.warning(
                    self,
                    '历史队列正在运行',
                    'Continue 后机械臂进入模式7，说明控制器恢复了运动队列。'
                    '请立即观察机械臂，必要时使用实体急停。',
                )
            self.append_log(f'解除暂停后模式：{mode_text(mode)}')

        self._submit(
            '安全解除模式10暂停',
            self.ros.resume_from_pause,
            succeeded,
        )

    def start_drag(self) -> None:
        disable_collision = self.disable_collision_check.isChecked()
        disable_safe_skin = self.disable_safe_skin_check.isChecked()
        restore_level = self.restore_collision_spin.value()
        warning = '确认机械臂已使能且静止，工作区已经清空。'
        if disable_collision:
            warning += '\n\n本操作将临时关闭本体软件碰撞检测。'
        if disable_safe_skin:
            warning += (
                '\n\n本操作还将临时关闭电子皮肤，机械臂将失去非接触防护。'
                '必须有人全程监护并可立即按下实体急停。'
            )
        if QMessageBox.warning(
            self,
            '确认进入拖动模式',
            warning,
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        ) != QMessageBox.Yes:
            return

        def succeeded(mode):
            self._collision_disabled_by_app = disable_collision
            self._safe_skin_disabled_by_app = disable_safe_skin
            self.append_log(f'当前模式：{mode_text(mode)}；可以拖动取点')

        scheduled = self._submit(
            '进入拖动模式',
            lambda: self.ros.start_drag(
                disable_collision,
                disable_safe_skin,
                restore_level,
            ),
            succeeded,
        )
        if scheduled:
            # 从任务排入队列起保守记录。若命令失败但回滚状态无法确认，关闭程序时
            # 仍会再次尝试恢复，而不会错误地认为保护已经开启。
            self._collision_disabled_by_app = disable_collision
            self._safe_skin_disabled_by_app = disable_safe_skin

    def stop_drag(self) -> None:
        restore_level = self.restore_collision_spin.value()
        self._submit(
            '退出拖动并恢复两项保护',
            lambda: self.ros.stop_drag(restore_level, True),
            self._drag_stopped,
        )

    def _drag_stopped(self, mode: str) -> None:
        self._collision_disabled_by_app = False
        self._safe_skin_disabled_by_app = False
        self.append_log(
            f'当前模式：{mode_text(mode)}；电子皮肤已开启，碰撞等级已恢复为 '
            f'{self.restore_collision_spin.value()}'
        )

    def restore_drag_protections(self) -> None:
        level = self.restore_collision_spin.value()

        def restored(unused_result):
            self._collision_disabled_by_app = False
            self._safe_skin_disabled_by_app = False
            self.append_log(f'电子皮肤已开启；碰撞等级已恢复为 {level}')

        self._submit(
            '手动恢复两项保护',
            lambda: self.ros.restore_drag_protections(level, True),
            restored,
        )

    def choose_points_dir(self) -> None:
        chosen = QFileDialog.getExistingDirectory(
            self, '选择点位目录', self.points_dir_edit.text()
        )
        if chosen:
            self.points_dir_edit.setText(chosen)
            self.refresh_points()

    def _clear_preview_files(self) -> None:
        self.camera_inbox.mkdir(parents=True, exist_ok=True)
        for path in self.camera_inbox.iterdir():
            if (
                path.is_file()
                and path.suffix.lower() in {'.jpg', '.jpeg', '.png', '.bmp'}
            ):
                try:
                    path.unlink()
                except OSError:
                    pass

    def start_camera_preview(self) -> None:
        if self._preview_thread is not None and self._preview_thread.is_alive():
            return
        camera_host = self.camera_ip_edit.text().strip()
        if not camera_host:
            QMessageBox.warning(self, '相机地址错误', '请输入 SC3000 IP 地址')
            return
        camera_timeout = self.camera_timeout_spin.value()
        apriltag_config = {
            'enabled': self.apriltag_enable_check.isChecked(),
            'tag_size_mm': self.apriltag_size_spin.value(),
            'family': self.apriltag_family_combo.currentData(),
            'tag_id': (
                None
                if self.apriltag_id_spin.value() < 0
                else self.apriltag_id_spin.value()
            ),
        }
        self._clear_preview_files()
        self._preview_stop.clear()
        self._preview_pause.clear()
        self._preview_frame_times.clear()
        self.preview_start_button.setEnabled(False)
        self.preview_stop_button.setEnabled(True)
        self.preview_status_label.setText('状态：正在连接 SC3000……')
        self._preview_thread = threading.Thread(
            target=self._camera_preview_worker,
            args=(camera_host, camera_timeout, apriltag_config),
            name='sc3000_gui_preview',
            daemon=True,
        )
        self._preview_thread.start()

    def stop_camera_preview(self) -> None:
        if self._preview_thread is None:
            return
        self._preview_stop.set()
        self.preview_stop_button.setEnabled(False)
        self.preview_status_label.setText('状态：正在停止预览……')

    def _camera_preview_worker(
        self,
        camera_host: str,
        camera_timeout: float,
        apriltag_config: dict,
    ) -> None:
        target_period = 0.2
        previous_error = ''
        previous_tag_error = ''
        try:
            while not self._preview_stop.is_set():
                if self._preview_pause.is_set():
                    self._preview_stop.wait(0.02)
                    continue
                frame_started = time.monotonic()
                image_path = None
                try:
                    result = self.camera.capture(
                        camera_host,
                        self.camera_inbox,
                        timeout=camera_timeout,
                        cancel_event=self._preview_stop,
                    )
                    image_path = result.image_path
                    image_data = image_path.read_bytes()
                    if not image_data:
                        raise RuntimeError('SC3000 返回了空图像')
                    if apriltag_config['enabled']:
                        try:
                            image_array = cv2.imdecode(
                                np.frombuffer(image_data, dtype=np.uint8),
                                cv2.IMREAD_COLOR,
                            )
                            localizer = self._get_apriltag_localizer()
                            detections = localizer.detect(
                                image_array,
                                apriltag_config['tag_size_mm'],
                                apriltag_config['family'],
                                apriltag_config['tag_id'],
                            )
                            detection_dicts = [
                                detection.as_dict()
                                for detection in detections
                            ]
                            base_error = ''
                            tool_pose = None
                            if detection_dicts:
                                try:
                                    tool_pose = self.ros.get_pose(
                                        user=0, tool=0, timeout=1.0
                                    )
                                    self._add_base_poses(
                                        detection_dicts, tool_pose
                                    )
                                except Exception as exc:
                                    base_error = (
                                        f'{type(exc).__name__}: {exc}'
                                    )
                            self._signals.apriltag_result.emit(
                                {
                                    'detections': detection_dicts,
                                    'error': '',
                                    'base_error': base_error,
                                    'tool_pose_user0_tool0_mm_deg': (
                                        list(tool_pose)
                                        if tool_pose is not None
                                        else None
                                    ),
                                }
                            )
                            previous_tag_error = ''
                            if detections:
                                annotated = localizer.annotate(
                                    image_array, detections
                                )
                                encoded, buffer = cv2.imencode(
                                    '.jpg',
                                    annotated,
                                    [cv2.IMWRITE_JPEG_QUALITY, 90],
                                )
                                if encoded:
                                    image_data = buffer.tobytes()
                        except Exception as exc:
                            tag_error = f'{type(exc).__name__}: {exc}'
                            if tag_error != previous_tag_error:
                                self._signals.apriltag_result.emit(
                                    {'detections': [], 'error': tag_error}
                                )
                                previous_tag_error = tag_error
                    previous_error = ''
                    self._signals.preview_frame.emit(
                        image_data, time.monotonic()
                    )
                except Exception as exc:
                    if self._preview_stop.is_set():
                        break
                    message = f'{type(exc).__name__}: {exc}'
                    if message != previous_error:
                        self._signals.preview_error.emit(message)
                        previous_error = message
                    self._preview_stop.wait(1.0)
                finally:
                    if image_path is not None:
                        try:
                            image_path.unlink()
                        except OSError:
                            pass

                remaining = target_period - (time.monotonic() - frame_started)
                if remaining > 0:
                    self._preview_stop.wait(remaining)
        finally:
            self._signals.preview_ended.emit()

    def _show_preview_frame(self, image_data: bytes, received_at: float) -> None:
        pixmap = QPixmap()
        if not pixmap.loadFromData(image_data):
            self._show_preview_error('无法解码 SC3000 图像')
            return
        self.preview_label.setPixmap(
            pixmap.scaled(
                self.preview_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )
        self._preview_frame_times.append(received_at)
        if len(self._preview_frame_times) >= 2:
            duration = (
                self._preview_frame_times[-1] - self._preview_frame_times[0]
            )
            fps = (
                (len(self._preview_frame_times) - 1) / duration
                if duration > 0
                else 0.0
            )
            self.preview_status_label.setText(
                f'状态：实时预览中；{fps:.1f} FPS；帧仅存于内存'
            )
        else:
            self.preview_status_label.setText('状态：实时预览中；帧仅存于内存')

    def _show_preview_error(self, message: str) -> None:
        self.preview_status_label.setText(f'状态：预览异常；{message}')
        self.append_log(f'SC3000 预览异常：{message}')

    def _get_apriltag_localizer(self) -> AprilTagLocalizer:
        with self._apriltag_localizer_lock:
            if self.apriltag_localizer is None:
                self.apriltag_localizer = AprilTagLocalizer()
            return self.apriltag_localizer

    def _add_base_poses(
        self, detections: list[dict], tool_pose
    ) -> list[dict]:
        if self.handeye_transform is None:
            raise HandEyeTransformError(
                self.handeye_error or '手眼标定变换尚未加载'
            )
        for detection in detections:
            detection['base_pose'] = (
                self.handeye_transform.transform_detection(
                    tool_pose, detection
                )
            )
        return detections

    def _show_apriltag_result(self, result: dict) -> None:
        error = result.get('error', '')
        if error:
            self.apriltag_pose_label.setText(f'AprilTag定位异常：{error}')
            return
        base_error = result.get('base_error', '')
        detections = result.get('detections', [])
        if not detections:
            self.apriltag_pose_label.setText(
                'AprilTag：未检测到完整标签\n'
                '相机坐标：X右 / Y下 / Z向前；基座坐标：CR5 User0'
            )
            return
        lines = []
        for detection in detections:
            x, y, z = detection['xyz_mm']
            roll, pitch, yaw = detection['rpy_degrees']
            lines.append(
                f"{detection['family']} ID={detection['id']}  "
                f"误差={detection['reprojection_rms_px']:.3f}px\n"
                f'  相机：X={x:.3f}  Y={y:.3f}  Z={z:.3f} mm  '
                f'RPY=[{roll:.2f}, {pitch:.2f}, {yaw:.2f}]°  '
            )
            base_pose = detection.get('base_pose')
            if base_pose is not None:
                bx, by, bz = base_pose['xyz_mm']
                br, bp, byaw = base_pose['rpy_degrees']
                lines.append(
                    f'  基座(User0)：X={bx:.3f}  Y={by:.3f}  '
                    f'Z={bz:.3f} mm  '
                    f'RPY=[{br:.2f}, {bp:.2f}, {byaw:.2f}]°'
                )
            else:
                lines.append(
                    f'  基座(User0)：不可用：'
                    f'{base_error or "本帧没有Tool0位姿"}'
                )
        self.apriltag_pose_label.setText('\n'.join(lines))

    def _preview_worker_ended(self) -> None:
        self._preview_thread = None
        self.preview_start_button.setEnabled(not self._closing)
        self.preview_stop_button.setEnabled(False)
        if not self._closing:
            self.preview_status_label.setText('状态：已停止；预览帧不写入磁盘')
        self._clear_preview_files()

    def _current_points_dir(self) -> Path:
        return Path(self.points_dir_edit.text()).expanduser().resolve()

    def refresh_points(self) -> None:
        try:
            directory = self._current_points_dir()
            names = list_point_names(directory)
        except OSError as exc:
            QMessageBox.critical(self, '点位目录错误', str(exc))
            return
        current_item = self.point_list.currentItem()
        selected = current_item.text() if current_item else ''
        self.point_list.clear()
        self.point_list.addItems(names)
        matches = self.point_list.findItems(selected, Qt.MatchExactly)
        if matches:
            self.point_list.setCurrentItem(matches[0])
        elif self.point_list.count():
            self.point_list.setCurrentRow(0)
        self._refresh_queue_point_combo(names)

    def _refresh_queue_point_combo(self, names: list[str]) -> None:
        selected = self.queue_point_combo.currentText()
        self.queue_point_combo.clear()
        self.queue_point_combo.addItems(names)
        index = self.queue_point_combo.findText(selected, Qt.MatchExactly)
        if index >= 0:
            self.queue_point_combo.setCurrentIndex(index)

    def show_selected_point(self, name: str) -> None:
        if not name:
            self.selected_joint_label.setText('--')
            self.selected_pose_label.setText('--')
            self.selected_image_label.setText('--')
            return
        try:
            point = load_point(self._current_points_dir(), name)
        except Exception as exc:
            self.selected_joint_label.setText(f'读取失败：{exc}')
            self.selected_pose_label.setText('--')
            self.selected_image_label.setText('--')
            return
        self.selected_joint_label.setText(format_joints(point.joints))
        self.selected_pose_label.setText(
            '--' if point.pose is None else ', '.join(
                f'{value:.3f}' for value in point.pose
            )
        )
        image = point_image_path(self._current_points_dir(), name)
        self.selected_image_label.setText(
            '--' if image is None else str(image)
        )

    def capture_point(self) -> None:
        try:
            name = validate_point_name(self.point_name_edit.text())
        except Exception as exc:
            QMessageBox.warning(self, '点名错误', str(exc))
            return
        directory = self._current_points_dir()
        if (directory / name).is_dir():
            QMessageBox.warning(
                self,
                '点位组已存在',
                f'{name} 已经是包含图像的点位组。为避免坐标与旧图错配，'
                '请使用“同步拍照并保存整组”覆盖，或输入新的点名。',
            )
            return
        point_file = directory / f'{name}_joint.txt'
        if point_file.exists() and QMessageBox.question(
            self, '覆盖点位', f'点位 {name} 已存在，确认覆盖？'
        ) != QMessageBox.Yes:
            return

        def operation():
            joints, pose, _ = self.ros.capture_stable_state()
            return save_point(directory, name, joints, pose)

        def succeeded(point):
            self.append_log(
                f'点位 {point.name} 已保存：[{format_joints(point.joints)}]'
            )
            self.refresh_points()
            matches = self.point_list.findItems(point.name, Qt.MatchExactly)
            if matches:
                self.point_list.setCurrentItem(matches[0])

        self._submit(f'保存点位 {name}', operation, succeeded)

    def capture_point_with_image(self) -> None:
        try:
            name = validate_point_name(self.point_name_edit.text())
        except Exception as exc:
            QMessageBox.warning(self, '点名错误', str(exc))
            return
        directory = self._current_points_dir()
        group_directory = directory / name
        legacy_file = directory / f'{name}_joint.txt'
        overwrite = group_directory.exists()
        if (overwrite or legacy_file.exists()) and QMessageBox.question(
            self,
            '覆盖点位',
            f'点位 {name} 已存在。确认保存新的坐标和图像组？\n\n'
            '已有同名文件夹将被整组替换；旧版平铺点位文件会保留。',
        ) != QMessageBox.Yes:
            return

        camera_host = self.camera_ip_edit.text().strip()
        camera_timeout = self.camera_timeout_spin.value()
        apriltag_config = {
            'enabled': self.apriltag_enable_check.isChecked(),
            'tag_size_mm': self.apriltag_size_spin.value(),
            'family': self.apriltag_family_combo.currentData(),
            'tag_id': (
                None
                if self.apriltag_id_spin.value() < 0
                else self.apriltag_id_spin.value()
            ),
        }
        if self._normal_busy:
            self.append_log('已有操作正在执行，请等待完成后再采集点位图像组')
            return
        self._preview_pause.set()

        def operation():
            camera_result = None
            try:
                before_time = time.time_ns()
                before_joints, before_pose, before_mode = (
                    self.ros.capture_stable_state()
                )
                camera_result = self.camera.capture(
                    camera_host,
                    self.camera_inbox,
                    timeout=camera_timeout,
                )
                tag_detections = []
                tag_error = ''
                if apriltag_config['enabled']:
                    try:
                        captured_image = cv2.imread(
                            str(camera_result.image_path), cv2.IMREAD_COLOR
                        )
                        tag_detections = [
                            detection.as_dict()
                            for detection in self._get_apriltag_localizer().detect(
                                captured_image,
                                apriltag_config['tag_size_mm'],
                                apriltag_config['family'],
                                apriltag_config['tag_id'],
                            )
                        ]
                    except Exception as exc:
                        tag_error = f'{type(exc).__name__}: {exc}'
                after_joints, after_pose, after_mode = (
                    self.ros.capture_stable_state()
                )
                after_time = time.time_ns()
                drift = max_joint_error_deg(before_joints, after_joints)
                if drift > 0.05:
                    raise RuntimeError(
                        f'拍照期间机械臂位置变化 {drift:.3f}°，'
                        '为避免图像与点位错配，本次整组未保存'
                    )
                base_transform_error = ''
                if tag_detections:
                    try:
                        self._add_base_poses(tag_detections, after_pose)
                    except Exception as exc:
                        base_transform_error = (
                            f'{type(exc).__name__}: {exc}'
                        )
                metadata = {
                    'association': (
                        'image trigger bracketed by two stable robot samples'
                    ),
                    'camera': {
                        'model': 'SC3000',
                        'ip': camera_host,
                        'modbus_port': 502,
                        'ftp_port': 2121,
                        'modbus_status_register': (
                            camera_result.status_register
                        ),
                    },
                    'timing_unix_ns': {
                        'robot_sample_before': before_time,
                        'camera_trigger': (
                            camera_result.trigger_time_unix_ns
                        ),
                        'image_received': (
                            camera_result.received_time_unix_ns
                        ),
                        'robot_sample_after': after_time,
                    },
                    'robot_mode_before': before_mode,
                    'robot_mode_after': after_mode,
                    'joint_angles_before_deg': list(before_joints),
                    'tool_pose_before': list(before_pose),
                    'maximum_joint_drift_deg': drift,
                    'ftp_source_filename': camera_result.image_path.name,
                    'apriltag': {
                        'enabled': apriltag_config['enabled'],
                        'configured_family': apriltag_config['family'],
                        'configured_id': apriltag_config['tag_id'],
                        'tag_size_mm': apriltag_config['tag_size_mm'],
                        'coordinate_frame': (
                            'opencv_camera_x_right_y_down_z_forward'
                        ),
                        'camera_coordinate_frame': (
                            'opencv_camera_x_right_y_down_z_forward'
                        ),
                        'base_coordinate_frame': 'dobot_user0_base',
                        'tool_pose_source': (
                            'robot_sample_after; GetPose(User=0, Tool=0)'
                        ),
                        'handeye_calibration': (
                            self.handeye_transform.calibration_summary()
                            if self.handeye_transform is not None
                            else None
                        ),
                        'detections': tag_detections,
                        'error': tag_error,
                        'base_transform_error': base_transform_error,
                    },
                }
                capture = save_capture_group(
                    directory,
                    name,
                    after_joints,
                    after_pose,
                    camera_result.image_path,
                    metadata,
                    overwrite=overwrite,
                )
                return capture, tag_detections
            finally:
                if camera_result is not None:
                    try:
                        camera_result.image_path.unlink()
                    except OSError:
                        pass
                self._preview_pause.clear()

        def succeeded(result):
            capture, tag_detections = result
            self.append_log(
                f'点位图像组 {capture.point.name} 已保存：'
                f'{capture.directory}'
            )
            for detection in tag_detections:
                x, y, z = detection['xyz_mm']
                self.append_log(
                    f"AprilTag {detection['family']} ID={detection['id']}："
                    f'相机 X={x:.3f}, Y={y:.3f}, Z={z:.3f} mm'
                )
                base_pose = detection.get('base_pose')
                if base_pose is not None:
                    bx, by, bz = base_pose['xyz_mm']
                    self.append_log(
                        f"AprilTag {detection['family']} "
                        f"ID={detection['id']}：基座(User0) "
                        f'X={bx:.3f}, Y={by:.3f}, Z={bz:.3f} mm'
                    )
            self.refresh_points()
            matches = self.point_list.findItems(
                capture.point.name, Qt.MatchExactly
            )
            if matches:
                self.point_list.setCurrentItem(matches[0])

        self._submit(f'拍照并保存点位组 {name}', operation, succeeded)

    def move_to_selected_point(self) -> None:
        item = self.point_list.currentItem()
        if item is None:
            QMessageBox.warning(self, '未选择点位', '请先选择目标点位')
            return
        try:
            point = load_point(self._current_points_dir(), item.text())
        except Exception as exc:
            QMessageBox.critical(self, '点位错误', str(exc))
            return
        if QMessageBox.warning(
            self,
            '确认点位运动',
            f'目标：{point.name}\n关节角：{format_joints(point.joints)}\n\n'
            '确认机械臂路径无人员、线缆和障碍物，并执行低速运动？',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        ) != QMessageBox.Yes:
            return

        def progress(text):
            self._signals.log.emit(text)

        speed_factor = self.move_speed_factor_spin.value()
        speed_j = self.move_speed_j_spin.value()
        acc_j = self.move_acc_j_spin.value()
        tolerance = self.tolerance_spin.value()
        self._submit(
            f'移动到点位 {point.name}',
            lambda: self.ros.move_to_joints(
                point.joints,
                speed_factor,
                speed_j,
                acc_j,
                tolerance_deg=tolerance,
                progress=progress,
            ),
            lambda actual: self.append_log(
                f'已到达 {point.name}：[{format_joints(actual)}]'
            ),
        )

    def _queue_kind_changed(self) -> None:
        kind = self.queue_kind_combo.currentData()
        is_move = kind == 'move_point'
        self.queue_point_combo.setEnabled(is_move)
        for widget in (
            self.queue_speed_factor_spin,
            self.queue_speed_j_spin,
            self.queue_acc_j_spin,
            self.queue_tolerance_spin,
        ):
            widget.setEnabled(is_move)

        configurations = {
            'gripper_close_percent': ('闭合比例', 0.0, 100.0, 0, ' %'),
            'gripper_position': ('协议位置', 0.0, 1000.0, 0, ''),
            'gripper_force': ('夹持力', 20.0, 100.0, 0, ' %'),
            'wait': ('等待时间', 0.1, 3600.0, 2, ' 秒'),
        }
        enabled = kind in configurations
        self.queue_value_label.setEnabled(enabled)
        self.queue_value_spin.setEnabled(enabled)
        if enabled:
            label, minimum, maximum, decimals, suffix = configurations[kind]
            self.queue_value_label.setText(label)
            self.queue_value_spin.setDecimals(decimals)
            self.queue_value_spin.setRange(minimum, maximum)
            self.queue_value_spin.setSuffix(suffix)
            if kind == 'wait' and self.queue_value_spin.value() < 1.0:
                self.queue_value_spin.setValue(1.0)
        else:
            self.queue_value_label.setText('无需数值' if not is_move else '数值')

    def _queue_editing_allowed(self) -> bool:
        if self._queue_is_running:
            QMessageBox.information(
                self,
                '队列正在执行',
                '队列执行期间不能修改指令；请等待结束或请求停止后再操作。',
            )
            return False
        return True

    def add_queue_command(self) -> None:
        if not self._queue_editing_allowed():
            return
        kind = self.queue_kind_combo.currentData()
        value = self.queue_value_spin.value()
        if kind == 'move_point':
            point = self.queue_point_combo.currentText()
            if not point:
                QMessageBox.warning(self, '没有点位', '请先在取点页面保存点位')
                return
            params = {
                'point': point,
                'speed_factor': self.queue_speed_factor_spin.value(),
                'speed_j': self.queue_speed_j_spin.value(),
                'acc_j': self.queue_acc_j_spin.value(),
                'tolerance_deg': self.queue_tolerance_spin.value(),
            }
        elif kind == 'gripper_close_percent':
            params = {'percent': round(value)}
        elif kind == 'gripper_position':
            params = {'position': round(value)}
        elif kind == 'gripper_force':
            params = {'force_percent': round(value)}
        elif kind == 'wait':
            params = {'seconds': value}
        else:
            params = {}
        try:
            command = QueueCommand(kind, params)
        except Exception as exc:
            QMessageBox.warning(self, '指令参数错误', str(exc))
            return
        self.queue_commands.append(command)
        self._render_queue_table()
        self.queue_table.selectRow(len(self.queue_commands) - 1)

    def _render_queue_table(self) -> None:
        self.queue_table.setRowCount(len(self.queue_commands))
        for row, command in enumerate(self.queue_commands):
            values = [str(row + 1), command.title, command.description, '待执行']
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                if column in {0, 3}:
                    item.setTextAlignment(Qt.AlignCenter)
                self.queue_table.setItem(row, column, item)

    def _selected_queue_row(self) -> int:
        rows = self.queue_table.selectionModel().selectedRows()
        return -1 if not rows else rows[0].row()

    def move_queue_command_up(self) -> None:
        if not self._queue_editing_allowed():
            return
        row = self._selected_queue_row()
        if row <= 0:
            return
        self.queue_commands[row - 1], self.queue_commands[row] = (
            self.queue_commands[row], self.queue_commands[row - 1]
        )
        self._render_queue_table()
        self.queue_table.selectRow(row - 1)

    def move_queue_command_down(self) -> None:
        if not self._queue_editing_allowed():
            return
        row = self._selected_queue_row()
        if row < 0 or row >= len(self.queue_commands) - 1:
            return
        self.queue_commands[row + 1], self.queue_commands[row] = (
            self.queue_commands[row], self.queue_commands[row + 1]
        )
        self._render_queue_table()
        self.queue_table.selectRow(row + 1)

    def remove_queue_command(self) -> None:
        if not self._queue_editing_allowed():
            return
        row = self._selected_queue_row()
        if row < 0:
            return
        del self.queue_commands[row]
        self._render_queue_table()
        if self.queue_commands:
            self.queue_table.selectRow(min(row, len(self.queue_commands) - 1))

    def clear_queue(self) -> None:
        if not self._queue_editing_allowed() or not self.queue_commands:
            return
        if QMessageBox.question(
            self, '确认清空', '确认删除当前队列中的全部指令？'
        ) != QMessageBox.Yes:
            return
        self.queue_commands.clear()
        self._render_queue_table()

    def save_queue_file(self) -> None:
        if not self.queue_commands:
            QMessageBox.warning(self, '空队列', '当前没有可保存的指令')
            return
        self.queues_dir.mkdir(parents=True, exist_ok=True)
        path, unused_filter = QFileDialog.getSaveFileName(
            self,
            '保存任务队列',
            str(self.queues_dir / 'task_queue.json'),
            'JSON 队列 (*.json)',
        )
        if not path:
            return
        destination = Path(path)
        if destination.suffix.lower() != '.json':
            destination = destination.with_suffix('.json')
        try:
            save_queue(destination, self.queue_commands)
        except Exception as exc:
            QMessageBox.critical(self, '保存失败', str(exc))
            return
        self.append_log(f'队列已保存：{destination}')

    def load_queue_file(self) -> None:
        if not self._queue_editing_allowed():
            return
        self.queues_dir.mkdir(parents=True, exist_ok=True)
        path, unused_filter = QFileDialog.getOpenFileName(
            self,
            '加载任务队列',
            str(self.queues_dir),
            'JSON 队列 (*.json)',
        )
        if not path:
            return
        try:
            commands = load_queue(Path(path))
        except Exception as exc:
            QMessageBox.critical(self, '加载失败', str(exc))
            return
        self.queue_commands = commands
        self._render_queue_table()
        self.append_log(f'已加载 {len(commands)} 条指令：{path}')

    def execute_queue(self) -> None:
        if self._queue_is_running:
            return
        if self._normal_busy:
            QMessageBox.information(
                self, '操作繁忙', '已有机械臂或夹爪操作正在执行，请等待完成。'
            )
            return
        if not self.queue_commands:
            QMessageBox.warning(self, '空队列', '请先添加至少一条指令')
            return
        if not self.ros.driver_ready():
            QMessageBox.warning(self, '驱动不可用', '请先启动并连接ROS 2驱动')
            return
        requires_gripper = any(
            command.kind.startswith('gripper_')
            for command in self.queue_commands
        )
        if requires_gripper and self.ros.gripper_index is None:
            QMessageBox.warning(
                self,
                '夹爪通道未创建',
                '队列包含夹爪指令。请先在“DH-AG95夹爪”页面创建Modbus通道。',
            )
            return

        points_dir = self._current_points_dir()
        try:
            for command in self.queue_commands:
                if command.kind == 'move_point':
                    load_point(points_dir, command.params['point'])
        except Exception as exc:
            QMessageBox.critical(
                self,
                '队列点位检查失败',
                f'尚未执行任何指令。请修正点位文件后重试：\n{exc}',
            )
            return

        preview_lines = [
            f'{index}. {command.title}：{command.description}'
            for index, command in enumerate(self.queue_commands[:15], 1)
        ]
        if len(self.queue_commands) > 15:
            preview_lines.append(f'……另有 {len(self.queue_commands) - 15} 条')
        preview = '\n'.join(preview_lines)
        if QMessageBox.warning(
            self,
            '确认执行任务队列',
            f'{preview}\n\n确认整条机械臂路径安全、夹爪周围无人，并按此顺序执行？',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        ) != QMessageBox.Yes:
            return

        commands = list(self.queue_commands)
        self.queue_cancel.clear()
        self._queue_is_running = True
        self.queue_execute_button.setEnabled(False)
        self.queue_stop_button.setEnabled(True)
        self._render_queue_table()
        runner = TaskQueueRunner(self.ros)

        def progress(index: int, state: str, message: str) -> None:
            self._signals.queue_step.emit(index, state, message)

        def operation():
            try:
                return runner.run(
                    commands, points_dir, self.queue_cancel, progress
                )
            finally:
                self._signals.queue_worker_ended.emit()

        self._submit('执行任务队列', operation, self._queue_finished)

    def stop_queue(self) -> None:
        if not self._queue_is_running:
            return
        self.queue_cancel.set()
        self.queue_stop_button.setEnabled(False)
        self.append_log('已请求在当前步骤安全完成后停止后续队列指令')

    def _show_queue_progress(
        self, index: int, state: str, message: str
    ) -> None:
        if not 0 <= index < self.queue_table.rowCount():
            return
        labels = {
            'running': '执行中',
            'done': '完成',
            'error': '失败',
            'cancelled': '已停止',
            'skipped': '未执行',
        }
        item = self.queue_table.item(index, 3)
        item.setText(labels.get(state, state))
        item.setToolTip(message)
        self.operation_status.setText(f'队列第 {index + 1} 步：{message}')
        if state in {'done', 'error', 'cancelled'}:
            self.append_log(f'队列第 {index + 1} 步 {labels[state]}：{message}')

    def _queue_finished(self, result: QueueRunResult) -> None:
        if result.cancelled:
            self.append_log(
                f'任务队列已停止：完成 {result.completed}/{result.total} 步'
            )
        else:
            self.append_log(f'任务队列全部完成：{result.total} 步')

    def _queue_worker_ended(self) -> None:
        self._queue_is_running = False
        self.queue_execute_button.setEnabled(True)
        self.queue_stop_button.setEnabled(False)

    def emergency_stop(self) -> None:
        self.queue_cancel.set()
        self.append_log('‼ 正在请求紧急停止……')
        self._submit(
            '紧急停止',
            self.ros.emergency_stop,
            lambda unused: self.append_log('紧急停止命令已被控制器接受'),
            safety=True,
        )

    def connect_gripper(self) -> None:
        address = self.modbus_ip_edit.text().strip()
        port = self.modbus_port_spin.value()
        slave_id = self.slave_id_spin.value()
        self._submit(
            '创建夹爪Modbus通道',
            lambda: self.ros.connect_gripper(address, port, slave_id),
            lambda index: self.gripper_channel_label.setText(
                f'通道：已连接，index={index}'
            ),
        )

    def disconnect_gripper(self) -> None:
        self._submit(
            '关闭夹爪Modbus通道',
            self.ros.disconnect_gripper,
            lambda unused: self.gripper_channel_label.setText('通道：未创建'),
        )

    def initialize_gripper(self) -> None:
        self._submit('初始化夹爪', self.ros.initialize_gripper)

    def apply_gripper_force(self) -> None:
        force = self.force_spin.value()
        self._submit(
            f'设置夹持力 {force}%',
            lambda: self.ros.set_gripper_force(force),
        )

    def move_gripper(self) -> None:
        position = self.position_spin.value()
        self._submit(
            f'设置夹爪位置 {position}',
            lambda: self.ros.set_gripper_position(position),
        )

    def open_gripper(self) -> None:
        self.position_spin.setValue(1000)
        self._submit(
            '完全打开夹爪', lambda: self.ros.set_gripper_position(1000)
        )

    def close_gripper(self) -> None:
        force = self.force_spin.value()
        if QMessageBox.warning(
            self,
            '确认闭合夹爪',
            f'夹爪将以 {force}% 夹持力闭合。请确认手指和无关物体已离开夹爪。',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        ) != QMessageBox.Yes:
            return

        def operation():
            self.ros.set_gripper_force(force)
            self.ros.set_gripper_position(0)

        self.position_spin.setValue(0)
        self._submit('低力闭合夹爪', operation)

    def read_gripper_status(self) -> None:
        self._submit(
            '读取夹爪状态',
            self.ros.read_gripper_status,
            self._show_gripper_status,
        )

    def _show_gripper_status(self, status: tuple[int, int, int]) -> None:
        initialized, grip_state, position = status
        initialization_text = {
            0: '0（未初始化）',
            1: '1（初始化成功）',
        }.get(initialized, f'{initialized}（请查协议）')
        grip_text = {
            0: '0（运动中）',
            1: '1（到达位置/未夹到物体）',
            2: '2（夹到物体）',
            3: '3（物体脱落）',
        }.get(grip_state, f'{grip_state}（请查协议）')
        self.gripper_init_label.setText(initialization_text)
        self.gripper_grip_label.setText(grip_text)
        self.gripper_position_label.setText(str(position))

    def closeEvent(self, event) -> None:
        if self._collision_disabled_by_app or self._safe_skin_disabled_by_app:
            answer = QMessageBox.warning(
                self,
                '安全保护尚未确认恢复',
                '本程序记录到本体碰撞检测或电子皮肤曾被关闭。关闭窗口后，'
                '程序会先尝试退出拖动并恢复两项保护；如果驱动已经断开，'
                '恢复可能失败，届时必须在示教器确认。\n\n继续关闭吗？',
                QMessageBox.Yes | QMessageBox.Cancel,
                QMessageBox.Cancel,
            )
            if answer != QMessageBox.Yes:
                event.ignore()
                return
        self._closing = True
        self.status_timer.stop()
        self.agv_status_timer.stop()
        self.agv_teleop_timer.stop()
        self._stop_agv_teleop()
        self.queue_cancel.set()
        self.ros.motion_cancel.set()
        self._preview_stop.set()
        event.accept()

    def shutdown_workers(self) -> None:
        self._closing = True
        self.queue_cancel.set()
        self.ros.motion_cancel.set()
        self._preview_stop.set()
        # 先等待已排队的控制指令退出，再恢复保护，避免 StartDrag 工作线程在
        # 恢复命令之后继续发送 SetSafeSkin(0)。
        self._normal_executor.shutdown(wait=True, cancel_futures=True)
        self._safety_executor.shutdown(wait=True, cancel_futures=True)
        if self.ros.agv_driver_ready():
            try:
                self.ros.agv_stop()
            except Exception as exc:
                print(
                    '上位机退出时未能确认AGV停止：'
                    f'{type(exc).__name__}: {exc}',
                    flush=True,
                )
        if self._collision_disabled_by_app or self._safe_skin_disabled_by_app:
            level = self.restore_collision_spin.value()
            try:
                mode = self.ros.get_mode()
                if mode == '6':
                    self.ros.stop_drag(level, True)
                else:
                    self.ros.restore_drag_protections(level, True)
            except Exception as exc:
                print(
                    '上位机退出时未能确认两项保护均已恢复：'
                    f'{type(exc).__name__}: {exc}',
                    flush=True,
                )
            else:
                self._collision_disabled_by_app = False
                self._safe_skin_disabled_by_app = False
        if self._preview_thread is not None:
            self._preview_thread.join(timeout=4.0)
        self.camera.close()
        self._clear_preview_files()
