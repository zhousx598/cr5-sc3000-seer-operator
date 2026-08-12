"""AprilTag localization using calibrated SC3000 intrinsics."""

from dataclasses import dataclass
import math
import os
from pathlib import Path

import cv2
import numpy as np


WORKSPACE = Path(
    os.environ.get('DOBOT_WS', str(Path.home() / 'dobot_ws'))
).expanduser()
DEFAULT_CALIBRATION = (
    WORKSPACE / 'points/calibration_results/sc3000_intrinsics_recommended.yaml'
)

APRILTAG_DICTIONARIES = {
    'tag16h5': cv2.aruco.DICT_APRILTAG_16h5,
    'tag25h9': cv2.aruco.DICT_APRILTAG_25h9,
    'tag36h10': cv2.aruco.DICT_APRILTAG_36h10,
    'tag36h11': cv2.aruco.DICT_APRILTAG_36h11,
}


class AprilTagLocalizationError(RuntimeError):
    pass


@dataclass(frozen=True)
class AprilTagDetection:
    family: str
    tag_id: int
    tag_size_mm: float
    corners_px: np.ndarray
    xyz_mm: np.ndarray
    rvec: np.ndarray
    rotation_matrix: np.ndarray
    rpy_degrees: tuple[float, float, float]
    reprojection_rms_px: float

    def as_dict(self) -> dict:
        return {
            'family': self.family,
            'id': self.tag_id,
            'tag_size_mm': self.tag_size_mm,
            'corners_px': self.corners_px.tolist(),
            'xyz_mm': self.xyz_mm.tolist(),
            'xyz_m': (self.xyz_mm / 1000.0).tolist(),
            'rvec': self.rvec.reshape(3).tolist(),
            'rotation_matrix': self.rotation_matrix.tolist(),
            'rpy_degrees': list(self.rpy_degrees),
            'reprojection_rms_px': self.reprojection_rms_px,
            'coordinate_frame': 'opencv_camera_x_right_y_down_z_forward',
        }


def _rotation_to_rpy_degrees(
    rotation: np.ndarray,
) -> tuple[float, float, float]:
    sy = math.hypot(rotation[0, 0], rotation[1, 0])
    if sy >= 1e-8:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


class AprilTagLocalizer:
    """Detect AprilTags and estimate their centers in the camera frame."""

    def __init__(self, calibration_path: Path = DEFAULT_CALIBRATION) -> None:
        self.calibration_path = calibration_path.expanduser().resolve()
        storage = cv2.FileStorage(
            str(self.calibration_path), cv2.FILE_STORAGE_READ
        )
        if not storage.isOpened():
            raise AprilTagLocalizationError(
                f'无法打开相机内参：{self.calibration_path}'
            )
        try:
            matrix = storage.getNode('camera_matrix').mat()
            distortion = storage.getNode('distortion_coefficients').mat()
            width = int(storage.getNode('image_width').real())
            height = int(storage.getNode('image_height').real())
        finally:
            storage.release()
        if matrix is None or matrix.shape != (3, 3):
            raise AprilTagLocalizationError('camera_matrix无效')
        if distortion is None or distortion.size < 4:
            raise AprilTagLocalizationError('distortion_coefficients无效')
        self.camera_matrix = matrix.astype(np.float64)
        self.distortion = distortion.astype(np.float64).reshape(-1, 1)
        self.image_size = (width, height)
        self._detectors = {}

    def _detector(self, family: str):
        detector = self._detectors.get(family)
        if detector is not None:
            return detector
        dictionary = cv2.aruco.getPredefinedDictionary(
            APRILTAG_DICTIONARIES[family]
        )
        if hasattr(cv2.aruco, 'DetectorParameters'):
            parameters = cv2.aruco.DetectorParameters()
        else:
            parameters = cv2.aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        else:
            detector = (dictionary, parameters)
        self._detectors[family] = detector
        return detector

    def _detect_markers(self, image: np.ndarray, family: str):
        detector = self._detector(family)
        if hasattr(detector, 'detectMarkers'):
            return detector.detectMarkers(image)
        dictionary, parameters = detector
        return cv2.aruco.detectMarkers(
            image, dictionary, parameters=parameters
        )

    def detect(
        self,
        image: np.ndarray,
        tag_size_mm: float,
        family: str = 'auto',
        tag_id: int | None = None,
    ) -> list[AprilTagDetection]:
        if image is None or image.size == 0:
            raise AprilTagLocalizationError('输入图像为空')
        size = (image.shape[1], image.shape[0])
        if size != self.image_size:
            raise AprilTagLocalizationError(
                f'图像分辨率{size}与内参分辨率{self.image_size}不一致'
            )
        if not math.isfinite(tag_size_mm) or tag_size_mm <= 0:
            raise AprilTagLocalizationError('AprilTag边长必须大于0')
        if family != 'auto' and family not in APRILTAG_DICTIONARIES:
            raise AprilTagLocalizationError(f'不支持的AprilTag族：{family}')

        families = list(APRILTAG_DICTIONARIES) if family == 'auto' else [family]
        detections = []
        for current_family in families:
            corners, ids, unused_rejected = self._detect_markers(
                image, current_family
            )
            if ids is None:
                continue
            for marker_id, marker_corners in zip(ids.reshape(-1), corners):
                marker_id = int(marker_id)
                if tag_id is not None and marker_id != tag_id:
                    continue
                detections.append(
                    self._estimate(
                        current_family,
                        marker_id,
                        marker_corners.reshape(4, 2).astype(np.float64),
                        tag_size_mm,
                    )
                )
        return detections

    def _estimate(
        self,
        family: str,
        tag_id: int,
        corners: np.ndarray,
        tag_size_mm: float,
    ) -> AprilTagDetection:
        half = tag_size_mm / 2.0
        object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )
        result = cv2.solvePnPGeneric(
            object_points,
            corners,
            self.camera_matrix,
            self.distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        solved, rvecs, tvecs = result[:3]
        if not solved:
            raise AprilTagLocalizationError('IPPE未能求解AprilTag位姿')
        candidates = []
        for rvec, tvec in zip(rvecs, tvecs):
            if float(tvec.reshape(3)[2]) <= 0:
                continue
            candidates.append(
                (
                    self._reprojection_rms(
                        object_points, corners, rvec, tvec
                    ),
                    rvec,
                    tvec,
                )
            )
        if not candidates:
            raise AprilTagLocalizationError('IPPE只返回了相机后方的解')
        unused_error, rvec, tvec = min(candidates, key=lambda item: item[0])
        rvec, tvec = cv2.solvePnPRefineLM(
            object_points,
            corners,
            self.camera_matrix,
            self.distortion,
            rvec,
            tvec,
        )
        rotation, unused = cv2.Rodrigues(rvec)
        return AprilTagDetection(
            family=family,
            tag_id=tag_id,
            tag_size_mm=tag_size_mm,
            corners_px=corners.copy(),
            xyz_mm=tvec.reshape(3).copy(),
            rvec=rvec.reshape(3).copy(),
            rotation_matrix=rotation,
            rpy_degrees=_rotation_to_rpy_degrees(rotation),
            reprojection_rms_px=self._reprojection_rms(
                object_points, corners, rvec, tvec
            ),
        )

    def _reprojection_rms(
        self, object_points, image_points, rvec, tvec
    ) -> float:
        projected, unused = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            self.camera_matrix,
            self.distortion,
        )
        residual = image_points.reshape(-1, 2) - projected.reshape(-1, 2)
        return float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))

    def annotate(
        self, image: np.ndarray, detections: list[AprilTagDetection]
    ) -> np.ndarray:
        annotated = image.copy()
        for detection in detections:
            corners = detection.corners_px
            cv2.polylines(
                annotated,
                [corners.astype(np.int32)],
                True,
                (0, 255, 0),
                2,
            )
            cv2.drawFrameAxes(
                annotated,
                self.camera_matrix,
                self.distortion,
                detection.rvec.reshape(3, 1),
                detection.xyz_mm.reshape(3, 1),
                detection.tag_size_mm * 0.5,
                2,
            )
            x, y, z = detection.xyz_mm
            label = (
                f'{detection.family} ID={detection.tag_id} '
                f'X={x:.2f} Y={y:.2f} Z={z:.2f} mm'
            )
            anchor = tuple(corners[0].astype(int))
            cv2.putText(
                annotated,
                label,
                (anchor[0], max(24, anchor[1] - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        return annotated
