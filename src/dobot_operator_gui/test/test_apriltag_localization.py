from pathlib import Path

import cv2
import numpy as np
import pytest

from dobot_operator_gui.apriltag_localization import AprilTagLocalizer
from dobot_operator_gui.apriltag_localization import AprilTagLocalizationError


def _write_calibration(path: Path):
    matrix = np.array(
        [
            [3792.0356339885, 0.0, 775.5997743484],
            [0.0, 3799.1077236463, 593.9024039816],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    distortion = np.array(
        [-0.0860208681, 0.0, 0.0, 0.0, 0.0], dtype=np.float64
    )
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    storage.write('image_width', 1408)
    storage.write('image_height', 1024)
    storage.write('camera_matrix', matrix)
    storage.write('distortion_coefficients', distortion)
    storage.release()


def test_detect_and_localize_synthetic_apriltag(tmp_path: Path):
    calibration = tmp_path / 'intrinsics.yaml'
    _write_calibration(calibration)
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11
    )
    tag = cv2.aruco.generateImageMarker(dictionary, 7, 400)
    image = np.full((1024, 1408), 255, dtype=np.uint8)
    image[312:712, 504:904] = tag
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    localizer = AprilTagLocalizer(calibration)
    detections = localizer.detect(image, 58.5, 'auto')

    assert len(detections) == 1
    detection = detections[0]
    assert detection.family == 'tag36h11'
    assert detection.tag_id == 7
    assert detection.xyz_mm[2] == pytest.approx(554.8, abs=2.0)
    assert detection.reprojection_rms_px < 0.5
    assert localizer.annotate(image, detections).shape == image.shape


def test_reject_wrong_image_resolution(tmp_path: Path):
    calibration = tmp_path / 'intrinsics.yaml'
    _write_calibration(calibration)
    localizer = AprilTagLocalizer(calibration)
    with pytest.raises(AprilTagLocalizationError):
        localizer.detect(np.zeros((480, 640, 3), np.uint8), 58.5)
