import os

import cv2  # noqa: F401 - importing cv2 reproduces its Qt path injection
from PyQt5.QtCore import QLibraryInfo

from dobot_operator_gui.main import restore_pyqt_plugin_path


def test_restore_pyqt_plugin_path_after_opencv_import():
    expected = QLibraryInfo.location(QLibraryInfo.PluginsPath)
    actual = restore_pyqt_plugin_path()

    assert actual == expected
    assert os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] == expected
    assert '/cv2/qt/plugins' not in os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']
    assert 'QT_PLUGIN_PATH' not in os.environ
