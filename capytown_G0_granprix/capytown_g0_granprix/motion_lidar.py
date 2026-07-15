"""Módulo motion_lidar."""

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class ZoneWindow:
    """Implementa ZoneWindow."""

    lo_deg: float
    hi_deg: float


def _scan_angles(n_points: int, angle_min: float, angle_increment: float) -> np.ndarray:
    """Ejecuta scan angles."""
    idx = np.arange(n_points, dtype=float)
    a = angle_min + idx * angle_increment
    return np.mod(a + math.pi, 2.0 * math.pi) - math.pi


def _robot_frame_angles(scan_angles: np.ndarray, front_offset_rad: float, sign: int) -> np.ndarray:
    """Ejecuta robot frame angles."""
    r = sign * (scan_angles - front_offset_rad)
    return np.mod(r + math.pi, 2.0 * math.pi) - math.pi


def compute_zone_distance(
    ranges: np.ndarray,
    robot_angles: np.ndarray,
    range_min: float,
    range_max: float,
    window: ZoneWindow,
) -> Tuple[float, bool]:
    """Calcula compute zone distance."""
    lo = math.radians(window.lo_deg)
    hi = math.radians(window.hi_deg)

    if lo <= hi:
        in_window = (robot_angles >= lo) & (robot_angles <= hi)
    else:

        in_window = (robot_angles >= lo) | (robot_angles <= hi)

    finite = np.isfinite(ranges)
    in_range = (ranges >= range_min) & (ranges <= range_max)
    mask = in_window & finite & in_range

    if not np.any(mask):
        return float('inf'), False
    return float(np.min(ranges[mask])), True


def compute_robot_frame_angles(
    ranges, angle_min: float, angle_increment: float, front_offset_rad: float, sign: int
) -> np.ndarray:
    """Calcula compute robot frame angles."""
    scan_angles = _scan_angles(len(ranges), angle_min, angle_increment)
    return _robot_frame_angles(scan_angles, front_offset_rad, sign)


def fit_wall_line(
    ranges: np.ndarray,
    robot_angles: np.ndarray,
    range_min: float,
    range_max: float,
    window: ZoneWindow,
    min_points: int = 6,
    max_outlier_iter: int = 3,
    outlier_residual_m: float = 0.03,
) -> Tuple[float, float, bool]:
    """Ejecuta fit wall line."""
    lo = math.radians(window.lo_deg)
    hi = math.radians(window.hi_deg)

    if lo <= hi:
        in_window = (robot_angles >= lo) & (robot_angles <= hi)
    else:
        in_window = (robot_angles >= lo) | (robot_angles <= hi)

    finite = np.isfinite(ranges)
    in_range = (ranges >= range_min) & (ranges <= range_max)
    mask = in_window & finite & in_range

    if int(np.sum(mask)) < min_points:
        return 0.0, 0.0, False

    x = ranges[mask] * np.cos(robot_angles[mask])
    y = ranges[mask] * np.sin(robot_angles[mask])

    for _ in range(max_outlier_iter):
        m, b = np.polyfit(x, y, 1)
        residuals = np.abs(y - (m * x + b)) / math.sqrt(m * m + 1.0)
        inliers = residuals < outlier_residual_m
        if bool(np.all(inliers)) or int(np.sum(inliers)) < min_points:
            break
        x, y = x[inliers], y[inliers]

    angulo = math.atan(m)
    distancia = abs(b) / math.sqrt(m * m + 1.0)

    return angulo, distancia, True
