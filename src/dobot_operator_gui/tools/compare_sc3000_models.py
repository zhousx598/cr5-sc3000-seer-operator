#!/usr/bin/env python3
"""Compare distortion models for the SC3000 chessboard dataset."""

import argparse
import importlib.util
import json
from pathlib import Path

import cv2
import numpy as np


def load_helpers():
    path = Path(__file__).with_name('calibrate_sc3000.py')
    spec = importlib.util.spec_from_file_location('sc3000_calibration', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def rms_for_view(object_points, image_points, rvec, tvec, matrix, distortion):
    projected, unused = cv2.projectPoints(
        object_points, rvec, tvec, matrix, distortion
    )
    residuals = image_points.reshape(-1, 2) - projected.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))


def calibrate(object_points, image_points, image_size, flags):
    return cv2.calibrateCameraExtended(
        object_points,
        image_points,
        image_size,
        None,
        None,
        flags=flags,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-dir', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    helpers = load_helpers()
    pattern_size = (7, 5)
    square_size = 11.2
    template = np.zeros((35, 3), np.float32)
    template[:, :2] = (
        np.mgrid[0:7, 0:5].T.reshape(-1, 2).astype(np.float32)
        * square_size
    )
    object_points = []
    image_points = []
    names = []
    image_size = None
    for path in helpers.grouped_images(args.input_dir.resolve()):
        image = cv2.imread(str(path))
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, unused_method = helpers.detect_corners(gray, pattern_size)
        if corners is None:
            continue
        image_size = (gray.shape[1], gray.shape[0])
        object_points.append(template.copy())
        image_points.append(corners)
        names.append(path.parent.name)

    models = {
        'standard_5': 0,
        'fix_k3': cv2.CALIB_FIX_K3,
        'fix_k2_k3': cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3,
        'radial_k1_k2': cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3,
        'radial_k1': (
            cv2.CALIB_ZERO_TANGENT_DIST
            | cv2.CALIB_FIX_K2
            | cv2.CALIB_FIX_K3
        ),
        'no_distortion': (
            cv2.CALIB_ZERO_TANGENT_DIST
            | cv2.CALIB_FIX_K1
            | cv2.CALIB_FIX_K2
            | cv2.CALIB_FIX_K3
        ),
    }
    results = {}
    for model_name, flags in models.items():
        output = calibrate(object_points, image_points, image_size, flags)
        rms, matrix, distortion, rvecs, tvecs = output[:5]
        train_errors = [
            rms_for_view(
                object_points[index],
                image_points[index],
                rvecs[index],
                tvecs[index],
                matrix,
                distortion,
            )
            for index in range(len(names))
        ]
        holdout_errors = []
        for held_out in range(len(names)):
            train_objects = [
                value for index, value in enumerate(object_points)
                if index != held_out
            ]
            train_images = [
                value for index, value in enumerate(image_points)
                if index != held_out
            ]
            held_output = calibrate(
                train_objects, train_images, image_size, flags
            )
            held_matrix, held_distortion = held_output[1:3]
            solved, rvec, tvec = cv2.solvePnP(
                object_points[held_out],
                image_points[held_out],
                held_matrix,
                held_distortion,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not solved:
                holdout_errors.append(float('nan'))
                continue
            holdout_errors.append(
                rms_for_view(
                    object_points[held_out],
                    image_points[held_out],
                    rvec,
                    tvec,
                    held_matrix,
                    held_distortion,
                )
            )
        results[model_name] = {
            'flags': flags,
            'training_rms_px': float(rms),
            'mean_per_view_training_rms_px': float(np.mean(train_errors)),
            'leave_one_out_mean_rms_px': float(np.nanmean(holdout_errors)),
            'leave_one_out_max_rms_px': float(np.nanmax(holdout_errors)),
            'camera_matrix': matrix.tolist(),
            'distortion_coefficients': distortion.reshape(-1).tolist(),
            'per_view_training_rms_px': dict(zip(names, train_errors)),
            'per_view_holdout_rms_px': dict(zip(names, holdout_errors)),
        }

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'distortion_model_comparison.json').write_text(
        json.dumps(results, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    print(
        'model training_rms mean_train leave_one_out_mean leave_one_out_max '
        'fx fy cx cy distortion'
    )
    for name, result in results.items():
        matrix = result['camera_matrix']
        print(
            name,
            f"{result['training_rms_px']:.6f}",
            f"{result['mean_per_view_training_rms_px']:.6f}",
            f"{result['leave_one_out_mean_rms_px']:.6f}",
            f"{result['leave_one_out_max_rms_px']:.6f}",
            f"{matrix[0][0]:.3f}",
            f"{matrix[1][1]:.3f}",
            f"{matrix[0][2]:.3f}",
            f"{matrix[1][2]:.3f}",
            result['distortion_coefficients'],
        )


if __name__ == '__main__':
    main()
