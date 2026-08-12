"""Small native Qt 2-D viewer for SEER map snapshots and live AGV pose."""

import math

from PyQt5.QtCore import QPointF, QSize, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPainterPath, QPen, QPolygonF
from PyQt5.QtWidgets import QWidget


class AgvMapWidget(QWidget):
    relocationPointSelected = pyqtSignal(float, float)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(330)
        self.setMouseTracking(True)
        self._map_data = {}
        self._pose = {}
        self._bounds = (-2.0, 2.0, -2.0, 2.0)
        self._scale = 60.0
        self._pan = QPointF(0.0, 0.0)
        self._auto_fit = True
        self._selected_point: tuple[float, float] | None = None
        self._drag_origin: QPointF | None = None
        self._pan_origin = QPointF(0.0, 0.0)

    def minimumSizeHint(self) -> QSize:
        return QSize(520, 330)

    def set_map_data(self, data: dict) -> None:
        self._map_data = data if isinstance(data, dict) else {}
        self._bounds = self._calculate_bounds()
        self._auto_fit = True
        self.update()

    def set_pose(self, pose: dict) -> None:
        self._pose = pose if isinstance(pose, dict) else {}
        self.update()

    def set_relocation_selection(self, x: float, y: float) -> None:
        self._selected_point = (float(x), float(y))
        self.update()

    def reset_view(self) -> None:
        self._auto_fit = True
        self._pan = QPointF(0.0, 0.0)
        self.update()

    def _calculate_bounds(self) -> tuple[float, float, float, float]:
        points: list[tuple[float, float]] = []

        def add_pair(value) -> None:
            if (
                isinstance(value, (list, tuple))
                and len(value) >= 2
                and isinstance(value[0], (int, float))
                and isinstance(value[1], (int, float))
            ):
                points.append((float(value[0]), float(value[1])))

        for point in self._map_data.get('normal_points', []):
            add_pair(point)
        for line in self._map_data.get('feature_lines', []):
            if isinstance(line, list):
                for point in line:
                    add_pair(point)
        for curve in self._map_data.get('curves', []):
            if isinstance(curve, list):
                for point in curve:
                    add_pair(point)
        for key in ('stations', 'advanced_points'):
            for item in self._map_data.get(key, []):
                if isinstance(item, dict):
                    add_pair((item.get('x'), item.get('y')))
        if not points:
            return (-2.0, 2.0, -2.0, 2.0)
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        margin = max(0.5, max(max(xs) - min(xs), max(ys) - min(ys)) * 0.05)
        return (
            min(xs) - margin,
            max(xs) + margin,
            min(ys) - margin,
            max(ys) + margin,
        )

    def _fit_view(self) -> None:
        left, right, bottom, top = self._bounds
        world_width = max(0.1, right - left)
        world_height = max(0.1, top - bottom)
        self._scale = max(
            2.0,
            min(
                500.0,
                (self.width() - 50.0) / world_width,
                (self.height() - 50.0) / world_height,
            ),
        )
        self._pan = QPointF(0.0, 0.0)
        self._auto_fit = False

    def _world_center(self) -> tuple[float, float]:
        left, right, bottom, top = self._bounds
        return (0.5 * (left + right), 0.5 * (bottom + top))

    def world_to_screen(self, x: float, y: float) -> QPointF:
        center_x, center_y = self._world_center()
        return QPointF(
            self.width() * 0.5 + (x - center_x) * self._scale + self._pan.x(),
            self.height() * 0.5 - (y - center_y) * self._scale + self._pan.y(),
        )

    def screen_to_world(self, point: QPointF) -> tuple[float, float]:
        center_x, center_y = self._world_center()
        return (
            center_x
            + (point.x() - self.width() * 0.5 - self._pan.x()) / self._scale,
            center_y
            - (point.y() - self.height() * 0.5 - self._pan.y()) / self._scale,
        )

    def paintEvent(self, unused_event) -> None:
        if self._auto_fit:
            self._fit_view()
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor('#101a28'))
        self._draw_map(painter)
        self._draw_agv(painter)
        self._draw_selection(painter)
        self._draw_overlay(painter)

    def _draw_map(self, painter: QPainter) -> None:
        normal = QPolygonF()
        for item in self._map_data.get('normal_points', []):
            if isinstance(item, list) and len(item) >= 2:
                normal.append(self.world_to_screen(float(item[0]), float(item[1])))
        if normal:
            painter.setPen(QPen(QColor(190, 202, 218, 170), 2.0))
            painter.drawPoints(normal)

        painter.setPen(QPen(QColor('#dce6f3'), 2.0))
        for line in self._map_data.get('feature_lines', []):
            if isinstance(line, list) and len(line) >= 2:
                start, end = line[0], line[1]
                painter.drawLine(
                    self.world_to_screen(float(start[0]), float(start[1])),
                    self.world_to_screen(float(end[0]), float(end[1])),
                )

        painter.setPen(QPen(QColor('#f39c45'), 2.0))
        for curve in self._map_data.get('curves', []):
            if not isinstance(curve, list) or len(curve) < 2:
                continue
            path = QPainterPath()
            first = curve[0]
            path.moveTo(self.world_to_screen(float(first[0]), float(first[1])))
            for point in curve[1:]:
                path.lineTo(self.world_to_screen(float(point[0]), float(point[1])))
            painter.drawPath(path)

        painter.setPen(QPen(QColor('#59e5ad'), 1.5))
        painter.setBrush(QColor(30, 180, 120, 150))
        for item in self._map_data.get('stations', []):
            if not isinstance(item, dict):
                continue
            point = self.world_to_screen(float(item['x']), float(item['y']))
            painter.drawEllipse(point, 5.0, 5.0)
            painter.drawText(point + QPointF(7.0, -7.0), str(item.get('id', '')))

    def _draw_agv(self, painter: QPainter) -> None:
        try:
            x = float(self._pose.get('x'))
            y = float(self._pose.get('y'))
            yaw_value = self._pose.get('angle')
            if yaw_value is None:
                yaw_value = self._pose.get('yaw')
            yaw = float(yaw_value)
        except (TypeError, ValueError):
            return
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return
        footprint = self._map_data.get('footprint', {})
        width = float(footprint.get('width', 0.7))
        head = float(footprint.get('head', 0.52))
        tail = float(footprint.get('tail', 0.48))
        local = [
            (head, width * 0.5),
            (head, -width * 0.5),
            (-tail, -width * 0.5),
            (-tail, width * 0.5),
        ]
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        polygon = QPolygonF()
        for local_x, local_y in local:
            world_x = x + local_x * cosine - local_y * sine
            world_y = y + local_x * sine + local_y * cosine
            polygon.append(self.world_to_screen(world_x, world_y))
        painter.setBrush(QColor(25, 190, 110, 115))
        painter.setPen(QPen(QColor('#32f08d'), 2.5))
        painter.drawPolygon(polygon)
        center = self.world_to_screen(x, y)
        front = self.world_to_screen(
            x + head * math.cos(yaw), y + head * math.sin(yaw)
        )
        painter.setPen(QPen(QColor('#ff3b30'), 3.0))
        painter.drawLine(center, front)
        painter.setBrush(QColor('#ffffff'))
        painter.drawEllipse(center, 3.5, 3.5)

    def _draw_selection(self, painter: QPainter) -> None:
        if self._selected_point is None:
            return
        point = self.world_to_screen(*self._selected_point)
        painter.setPen(QPen(QColor('#ff4fd8'), 2.0))
        painter.drawLine(point + QPointF(-9.0, 0.0), point + QPointF(9.0, 0.0))
        painter.drawLine(point + QPointF(0.0, -9.0), point + QPointF(0.0, 9.0))

    def _draw_overlay(self, painter: QPainter) -> None:
        current_map = str(self._map_data.get('current_map') or '尚未收到地图')
        painter.setPen(QColor('#e7eef8'))
        painter.drawText(QPointF(12.0, 22.0), f'地图：{current_map}')
        painter.setPen(QColor('#9db0c7'))
        painter.drawText(
            QPointF(12.0, self.height() - 12.0),
            '左键选择重定位坐标；右键拖动画布；滚轮缩放',
        )

    def wheelEvent(self, event) -> None:
        before = self.screen_to_world(event.pos())
        factor = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15
        self._scale = max(2.0, min(800.0, self._scale * factor))
        after = self.world_to_screen(*before)
        self._pan += event.pos() - after
        self._auto_fit = False
        self.update()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            x, y = self.screen_to_world(event.pos())
            self._selected_point = (x, y)
            self.relocationPointSelected.emit(x, y)
            self.update()
        elif event.button() == Qt.RightButton:
            self._drag_origin = event.pos()
            self._pan_origin = QPointF(self._pan)

    def mouseMoveEvent(self, event) -> None:
        if self._drag_origin is not None and event.buttons() & Qt.RightButton:
            self._pan = self._pan_origin + event.pos() - self._drag_origin
            self._auto_fit = False
            self.update()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.RightButton:
            self._drag_origin = None

    def resizeEvent(self, event) -> None:
        if self._auto_fit:
            self.update()
        super().resizeEvent(event)
