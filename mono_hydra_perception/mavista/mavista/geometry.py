"""Camera-geometry helpers shared by the ROS node and its tests."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ResizeGeometry:
    x0: int
    y0: int
    crop_width: int
    crop_height: int
    scale_x: float
    scale_y: float


def center_crop_resize_geometry(width: int, height: int, target_width: int = 640, target_height: int = 480) -> ResizeGeometry:
    """Return the centered 4:3 crop and scale factors for a target image."""
    if width <= 0 or height <= 0:
        raise ValueError("input image dimensions must be positive")
    input_ratio = width / height
    target_ratio = target_width / target_height
    if input_ratio > target_ratio:
        crop_height = height
        crop_width = round(height * target_ratio)
        x0, y0 = (width - crop_width) // 2, 0
    else:
        crop_width = width
        crop_height = round(width / target_ratio)
        x0, y0 = 0, (height - crop_height) // 2
    return ResizeGeometry(x0, y0, crop_width, crop_height, target_width / crop_width, target_height / crop_height)


def transform_intrinsics(fx: float, fy: float, cx: float, cy: float, geometry: ResizeGeometry) -> tuple[float, float, float, float]:
    """Transform pixel intrinsics after the crop followed by resize."""
    return (
        fx * geometry.scale_x,
        fy * geometry.scale_y,
        (cx - geometry.x0) * geometry.scale_x,
        (cy - geometry.y0) * geometry.scale_y,
    )
