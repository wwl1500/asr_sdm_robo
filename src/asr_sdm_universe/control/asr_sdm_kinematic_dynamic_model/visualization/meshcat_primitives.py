"""Small, reusable Meshcat line primitives."""

from __future__ import annotations

import numpy as np


def arrow_segments(
    origin: np.ndarray,
    vector: np.ndarray,
    scale: float,
    head_ratio: float = 0.18,
):
    """Return 3xN line-segment endpoints for an arrow."""
    origin = np.asarray(origin, dtype=float)
    vector = np.asarray(vector, dtype=float)
    shaft = scale * vector
    length = np.linalg.norm(shaft)
    if length < 1.0e-12:
        return None

    direction = shaft / length
    helper = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(direction, helper)) > 0.9:
        helper = np.array([0.0, 1.0, 0.0])
    first = np.cross(direction, helper)
    first /= np.linalg.norm(first)
    second = np.cross(direction, first)

    tip = origin + shaft
    base = tip - head_ratio * length * direction
    width = 0.45 * head_ratio * length
    points = np.stack(
        [
            origin,
            tip,
            tip,
            base + width * first,
            tip,
            base - width * first,
            tip,
            base + width * second,
            tip,
            base - width * second,
        ],
        axis=1,
    )
    return points


def set_arrow(viewer, path, origin, vector, scale, color, geometry):
    """Replace one line-segment arrow in a Meshcat scene."""
    points = arrow_segments(origin, vector, scale)
    node = viewer[path]
    if points is None:
        node.delete()
        return
    node.set_object(
        geometry.LineSegments(
            geometry.PointsGeometry(points),
            geometry.LineBasicMaterial(color=color),
        )
    )


def polyline_segments(points: list[np.ndarray]):
    """Return 3xN line-segment endpoints for a point history."""
    if len(points) < 2:
        return None
    return np.stack(
        [point for pair in zip(points[:-1], points[1:]) for point in pair],
        axis=1,
    )


def set_polyline(viewer, path, points, color, geometry):
    """Replace one line-strip history in a Meshcat scene."""
    segments = polyline_segments(points)
    if segments is None:
        return
    viewer[path].set_object(
        geometry.LineSegments(
            geometry.PointsGeometry(segments),
            geometry.LineBasicMaterial(color=color),
        )
    )
