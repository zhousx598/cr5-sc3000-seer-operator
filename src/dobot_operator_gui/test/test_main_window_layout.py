import os
from pathlib import Path
import threading

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QScrollArea
from PyQt5.QtWidgets import QSplitter

from dobot_operator_gui.agv_map_widget import AgvMapWidget
from dobot_operator_gui.main_window import DobotOperatorWindow


class OfflineRosClient:
    def __init__(self):
        self.motion_cancel = threading.Event()
        self.gripper_index = None


def test_main_tabs_group_related_controls_and_agv_is_scrollable(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv('DOBOT_WS', str(tmp_path))
    app = QApplication.instance() or QApplication([])
    window = DobotOperatorWindow(OfflineRosClient())
    window.status_timer.stop()
    window.agv_status_timer.stop()
    window.agv_teleop_timer.stop()

    try:
        labels = [
            window.tabs.tabText(index)
            for index in range(window.tabs.count())
        ]
        assert labels == [
            '连接、状态与日志',
            '机械臂与DH-AG95',
            '取点与点位运动',
            '任务队列',
            'SEER AGV综合控制',
        ]

        status_tab = window.tabs.widget(0)
        assert status_tab.isAncestorOf(window.ros_label)
        assert status_tab.isAncestorOf(window.log_edit)
        assert len(status_tab.findChildren(QSplitter)) == 1

        robot_tab = window.tabs.widget(1)
        assert robot_tab.isAncestorOf(window.load_spin)
        assert robot_tab.isAncestorOf(window.gripper_channel_label)
        assert len(robot_tab.findChildren(QScrollArea)) == 1

        agv_tab = window.tabs.widget(4)
        assert agv_tab.isAncestorOf(window.agv_driver_label)
        assert agv_tab.isAncestorOf(window.agv_map_widget)
        assert agv_tab.isAncestorOf(window.agv_teleop_enable)
        assert len(agv_tab.findChildren(QScrollArea)) == 1
        assert len(agv_tab.findChildren(AgvMapWidget)) == 1

        waypoint_index = window.agv_map_click_target_combo.findData('waypoint')
        window.agv_map_click_target_combo.setCurrentIndex(waypoint_index)
        window._agv_unified_map_point_selected(1.2, -0.4)
        assert window.agv_waypoint_x_spin.value() == 1.2
        assert window.agv_waypoint_y_spin.value() == -0.4

        relocation_index = window.agv_map_click_target_combo.findData(
            'relocation'
        )
        window.agv_map_click_target_combo.setCurrentIndex(relocation_index)
        window._agv_unified_map_point_selected(-2.0, 3.0)
        assert window.agv_reloc_x_spin.value() == -2.0
        assert window.agv_reloc_y_spin.value() == 3.0
    finally:
        window._normal_executor.shutdown(wait=False, cancel_futures=True)
        window._safety_executor.shutdown(wait=False, cancel_futures=True)
        window.camera.close()
        window.deleteLater()
        app.processEvents()
