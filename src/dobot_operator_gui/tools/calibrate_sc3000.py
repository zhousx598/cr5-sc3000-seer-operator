#!/usr/bin/env python3
"""Calibrate SC3000 intrinsics from grouped chessboard captures."""

import argparse
import json
import math
from pathlib import Path
import re

import cv2
import numpy as np


IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.bmp'}


def natural_key(path: Path):
    return [
        int(token) if token.isdigit() else token.lower()
        for token in re.split(r'(\d+)', str(path))
    ]


def grouped_images(root: Path) -> list[Path]:
    images = []
    for directory in root.iterdir():
        if not directory.is_dir():
            continue
        candidates = [
            path
            for path in directory.iterdir()
            if path.is_file()
            and path.suffix.lower() in IMAGE_SUFFIXES
            and path.stem == f'{directory.name}_image'
        ]
        images.extend(candidates)
    return sorted(images, key=natural_key)


def detect_corners(gray, pattern_size):
    sb_flags = (
        cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_EXHAUSTIVE
        | cv2.CALIB_CB_ACCURACY
    )
    found, corners = cv2.findChessboardCornersSB(
        gray, pattern_size, flags=sb_flags
    )
    if found:
        return corners.astype(np.float32), 'findChessboardCornersSB'

    classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(
        gray, pattern_size, flags=classic_flags
    )
    if not found:
        return None, None
    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        50,
        1e-4,
    )
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1), criteria
    )
    return corners, 'findChessboardCorners+cornerSubPix'


def build_contact_sheet(entries, output_path: Path) -> None:
    if not entries:
        return
    thumb_width = 352
    columns = 4
    tiles = []
    for entry in entries:
        image = cv2.imread(str(entry['path']))
        if image is None:
            continue
        cv2.drawChessboardCorners(
            image,
            entry['pattern_size'],
            entry['corners'],
            True,
        )
        scale = thumb_width / image.shape[1]
        resized = cv2.resize(
            image,
            (thumb_width, int(round(image.shape[0] * scale))),
            interpolation=cv2.INTER_AREA,
        )
        label = f"{entry['path'].parent.name}: {entry['error_px']:.3f} px"
        cv2.rectangle(resized, (0, 0), (thumb_width, 28), (0, 0, 0), -1)
        cv2.putText(
            resized,
            label,
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (80, 255, 100),
            1,
            cv2.LINE_AA,
        )
        tiles.append(resized)
    if not tiles:
        return
    tile_height = max(tile.shape[0] for tile in tiles)
    rows = math.ceil(len(tiles) / columns)
    sheet = np.zeros(
        (rows * tile_height, columns * thumb_width, 3), dtype=np.uint8
    )
    for index, tile in enumerate(tiles):
        row, column = divmod(index, columns)
        sheet[
            row * tile_height:row * tile_height + tile.shape[0],
            column * thumb_width:(column + 1) * thumb_width,
        ] = tile
    cv2.imwrite(str(output_path), sheet)


def write_opencv_yaml(
    output_path: Path,
    camera_matrix,
    distortion,
    image_size,
    rms,
    pattern_size,
    square_size_mm,
    valid_count,
) -> None:
    storage = cv2.FileStorage(str(output_path), cv2.FILE_STORAGE_WRITE)
    if not storage.isOpened():
        raise RuntimeError(f'无法写入：{output_path}')
    storage.write('image_width', image_size[0])
    storage.write('image_height', image_size[1])
    storage.write('board_inner_corners_columns', pattern_size[0])
    storage.write('board_inner_corners_rows', pattern_size[1])
    storage.write('square_size_mm', square_size_mm)
    storage.write('valid_image_count', valid_count)
    storage.write('rms_reprojection_error_px', rms)
    storage.write('camera_matrix', camera_matrix)
    storage.write('distortion_coefficients', distortion)
    storage.release()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--corners-x', type=int, default=7)
    parser.add_argument('--corners-y', type=int, default=5)
    parser.add_argument('--square-size-mm', type=float, default=11.2)
    parser.add_argument(
        '--distortion-model',
        choices=[
            'standard-5',
            'fix-k3',
            'fix-k2-k3',
            'radial-k1-k2',
            'radial-k1',
            'no-distortion',
        ],
        default='standard-5',
    )
    parser.add_argument('--output-prefix', default='sc3000_intrinsics')
    args = parser.parse_args()

    input_dir = args.input_dir.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    pattern_size = (args.corners_x, args.corners_y)

    object_template = np.zeros(
        (pattern_size[0] * pattern_size[1], 3), dtype=np.float32
    )
    object_template[:, :2] = (
        np.mgrid[0:pattern_size[0], 0:pattern_size[1]]
        .T.reshape(-1, 2)
        .astype(np.float32)
        * args.square_size_mm
    )

    paths = grouped_images(input_dir)
    if not paths:
        raise RuntimeError(f'没有找到分组标定图：{input_dir}')

    object_points = []
    image_points = []
    detected = []
    failed = []
    image_size = None
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            failed.append({'image': str(path), 'reason': '无法读取图像'})
            continue
        size = (image.shape[1], image.shape[0])
        if image_size is None:
            image_size = size
        if size != image_size:
            failed.append(
                {
                    'image': str(path),
                    'reason': f'分辨率 {size} 与首张 {image_size} 不一致',
                }
            )
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, method = detect_corners(gray, pattern_size)
        if corners is None:
            failed.append({'image': str(path), 'reason': '未找到7×5内角点'})
            continue
        object_points.append(object_template.copy())
        image_points.append(corners)
        detected.append(
            {
                'path': path,
                'corners': corners,
                'method': method,
                'pattern_size': pattern_size,
            }
        )

    if len(detected) < 8:
        raise RuntimeError(
            f'只有 {len(detected)} 张图片检测成功，少于建议下限 8 张'
        )

    calibration_flags = {
        'standard-5': 0,
        'fix-k3': cv2.CALIB_FIX_K3,
        'fix-k2-k3': cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3,
        'radial-k1-k2': cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3,
        'radial-k1': (
            cv2.CALIB_ZERO_TANGENT_DIST
            | cv2.CALIB_FIX_K2
            | cv2.CALIB_FIX_K3
        ),
        'no-distortion': (
            cv2.CALIB_ZERO_TANGENT_DIST
            | cv2.CALIB_FIX_K1
            | cv2.CALIB_FIX_K2
            | cv2.CALIB_FIX_K3
        ),
    }[args.distortion_model]

    (
        rms,
        camera_matrix,
        distortion,
        rotation_vectors,
        translation_vectors,
        intrinsic_std,
        unused_extrinsic_std,
        unused_per_view,
    ) = cv2.calibrateCameraExtended(
        object_points,
        image_points,
        image_size,
        None,
        None,
        flags=calibration_flags,
    )

    per_view = []
    for index, entry in enumerate(detected):
        projected, unused_jacobian = cv2.projectPoints(
            object_points[index],
            rotation_vectors[index],
            translation_vectors[index],
            camera_matrix,
            distortion,
        )
        residuals = image_points[index].reshape(-1, 2) - projected.reshape(-1, 2)
        error = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))
        entry['error_px'] = error
        per_view.append(
            {
                'image': str(entry['path']),
                'detection_method': entry['method'],
                'reprojection_rms_px': error,
                'rvec': rotation_vectors[index].reshape(-1).tolist(),
                'tvec_mm': translation_vectors[index].reshape(-1).tolist(),
            }
        )

    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    horizontal_fov = math.degrees(2 * math.atan(image_size[0] / (2 * fx)))
    vertical_fov = math.degrees(2 * math.atan(image_size[1] / (2 * fy)))
    errors = np.array([item['reprojection_rms_px'] for item in per_view])
    result = {
        'model': 'SC3000',
        'opencv_version': cv2.__version__,
        'distortion_model': args.distortion_model,
        'calibration_flags': calibration_flags,
        'image_size': {'width': image_size[0], 'height': image_size[1]},
        'board': {
            'squares': [8, 6],
            'inner_corners': [pattern_size[0], pattern_size[1]],
            'square_size_mm': args.square_size_mm,
        },
        'input_image_count': len(paths),
        'valid_image_count': len(detected),
        'failed_images': failed,
        'rms_reprojection_error_px': float(rms),
        'mean_per_view_rms_px': float(errors.mean()),
        'median_per_view_rms_px': float(np.median(errors)),
        'maximum_per_view_rms_px': float(errors.max()),
        'camera_matrix': camera_matrix.tolist(),
        'distortion_coefficients': distortion.reshape(-1).tolist(),
        'intrinsic_standard_deviations': intrinsic_std.reshape(-1).tolist(),
        'field_of_view_degrees': {
            'horizontal': horizontal_fov,
            'vertical': vertical_fov,
        },
        'principal_point': {'cx': float(cx), 'cy': float(cy)},
        'per_view': per_view,
    }

    json_path = output_dir / f'{args.output_prefix}.json'
    json_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    yaml_path = output_dir / f'{args.output_prefix}.yaml'
    write_opencv_yaml(
        yaml_path,
        camera_matrix,
        distortion,
        image_size,
        float(rms),
        pattern_size,
        args.square_size_mm,
        len(detected),
    )
    contact_sheet_path = output_dir / f'{args.output_prefix}_corners.jpg'
    build_contact_sheet(detected, contact_sheet_path)

    ordered = sorted(per_view, key=lambda item: item['reprojection_rms_px'])
    report_lines = [
        '# SC3000 相机内参标定结果',
        '',
        f'- 输入图片：{len(paths)} 张',
        f'- 成功检测：{len(detected)} 张',
        f'- 失败：{len(failed)} 张',
        f'- 分辨率：{image_size[0]} × {image_size[1]}',
        f'- 棋盘内角点：{pattern_size[0]} × {pattern_size[1]}',
        f'- 角点间距：{args.square_size_mm:g} mm',
        f'- 畸变模型：{args.distortion_model}',
        f'- OpenCV 总体 RMS：{rms:.6f} px',
        f'- 逐图误差均值：{errors.mean():.6f} px',
        f'- 逐图误差中位数：{np.median(errors):.6f} px',
        f'- 逐图最大误差：{errors.max():.6f} px',
        '',
        '## 相机矩阵',
        '',
        '```text',
        np.array2string(camera_matrix, precision=10),
        '```',
        '',
        '## 畸变系数',
        '',
        '顺序：`k1, k2, p1, p2, k3`',
        '',
        '```text',
        np.array2string(distortion.reshape(-1), precision=10),
        '```',
        '',
        f'- 水平视场角：{horizontal_fov:.4f}°',
        f'- 垂直视场角：{vertical_fov:.4f}°',
        '',
        '## 每张图片重投影误差',
        '',
        '| 图片 | RMS (px) |',
        '|---|---:|',
    ]
    for item in ordered:
        report_lines.append(
            f"| `{Path(item['image']).parent.name}` | "
            f"{item['reprojection_rms_px']:.6f} |"
        )
    if failed:
        report_lines.extend(['', '## 检测失败', ''])
        for item in failed:
            report_lines.append(f"- `{item['image']}`：{item['reason']}")
    report_lines.extend(
        [
            '',
            '## 文件',
            '',
            f'- `{yaml_path.name}`：OpenCV 可直接读取的内参。',
            f'- `{json_path.name}`：完整参数、逐图位姿和误差。',
            f'- `{contact_sheet_path.name}`：角点检测和逐图误差拼图。',
            '',
        ]
    )
    report_name = (
        'CALIBRATION_REPORT.md'
        if args.output_prefix == 'sc3000_intrinsics'
        else f'{args.output_prefix}_report.md'
    )
    (output_dir / report_name).write_text(
        '\n'.join(report_lines), encoding='utf-8'
    )

    print(f'input={len(paths)} valid={len(detected)} failed={len(failed)}')
    print(f'image_size={image_size[0]}x{image_size[1]}')
    print(f'rms={rms:.6f} px mean={errors.mean():.6f} px max={errors.max():.6f} px')
    print('camera_matrix=')
    print(camera_matrix)
    print('distortion=', distortion.reshape(-1))
    print(f'output={output_dir}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
