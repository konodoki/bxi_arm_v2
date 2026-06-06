"""Pico arm workspace calibration helpers."""

import time

import numpy as np
from scipy.optimize import leastsq


class CalibrationStopped(RuntimeError):
    """Raised when calibration is interrupted by the caller."""


def fit_sphere(points: np.ndarray) -> tuple[float, float, float, float]:
    """Fit a sphere to points and return center xyz plus radius."""
    if not isinstance(points, np.ndarray):
        raise TypeError('points must be a numpy.ndarray')
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    if points.shape[0] < 4:
        raise ValueError('at least 4 non-coplanar points are required')

    center_init = points.mean(axis=0)
    radius_init = np.mean(np.linalg.norm(points - center_init, axis=1))
    params_init = np.hstack((center_init, radius_init))

    def residuals(params, sample_points):
        x0, y0, z0, radius = params
        center = np.array([x0, y0, z0])
        distances = np.linalg.norm(sample_points - center, axis=1)
        return distances - radius

    result, _, _, message, ier = leastsq(
        residuals,
        params_init,
        args=(points,),
        full_output=True,
    )
    if ier not in (1, 2, 3, 4):
        raise RuntimeError(f'sphere fit failed: {message}')

    return tuple(float(value) for value in result)


def _raise_if_stopped(stop_event) -> None:
    if stop_event.is_set():
        raise CalibrationStopped()


def _relative_position(hand, head) -> list[float]:
    hand_pos = hand.get_pos()
    head_pos = head.get_pos()
    return [
        hand_pos[0] - head_pos[0],
        hand_pos[1] - head_pos[1],
        hand_pos[2] - head_pos[2],
    ]


def fit_pico_method2(
    left_pico,
    right_pico,
    head_pico,
    CONTROLOR_ARM_LENGTH,
    pico_thread_event,
):
    """Calibrate Pico hand workspaces using sphere fitting."""
    if CONTROLOR_ARM_LENGTH <= 0.0:
        raise ValueError('CONTROLOR_ARM_LENGTH must be positive')

    sample_rate_hz = 1000.0
    sample_period = 1.0 / sample_rate_hz
    left_sample_point = []
    right_sample_point = []

    print('将双手举过头顶挥舞', flush=True)
    while (
        left_pico.get_pos()[2] - 0.4 < head_pico.get_pos()[2]
        or right_pico.get_pos()[2] - 0.4 < head_pico.get_pos()[2]
    ):
        _raise_if_stopped(pico_thread_event)
        time.sleep(sample_period)

    print('开始读取', flush=True)
    start_time = time.monotonic()
    while time.monotonic() - start_time < 10.0:
        _raise_if_stopped(pico_thread_event)
        left_sample_point.append(_relative_position(left_pico, head_pico))
        right_sample_point.append(_relative_position(right_pico, head_pico))
        time.sleep(sample_period)

    left_sample_point = np.asarray(left_sample_point, dtype=float)
    right_sample_point = np.asarray(right_sample_point, dtype=float)
    print(f'共采集{left_sample_point.shape}个点', flush=True)
    print('开始拟合', flush=True)

    x0, y0, z0, r0 = fit_sphere(left_sample_point)
    x1, y1, z1, r1 = fit_sphere(right_sample_point)
    left_scale = r0 / CONTROLOR_ARM_LENGTH
    right_scale = r1 / CONTROLOR_ARM_LENGTH

    print(
        f'左手拟合球心: ({x0:.4f}, {y0:.4f}, {z0:.4f}, '
        f'拟合半径: {r0:.4f})',
        flush=True,
    )
    print(
        f'右手拟合球心: ({x1:.4f}, {y1:.4f}, {z1:.4f}, '
        f'拟合半径: {r1:.4f})',
        flush=True,
    )

    return [x0, y0, z0], [x1, y1, z1], left_scale, right_scale


def main() -> None:
    """Run a small sphere-fitting self-test."""
    np.random.seed(0)
    true_center = np.array([1.0, 2.0, 3.0])
    true_radius = 5.0
    num_points = 100

    phi = np.random.uniform(0, np.pi, num_points)
    theta = np.random.uniform(0, 2 * np.pi, num_points)
    x = true_center[0] + true_radius * np.sin(phi) * np.cos(theta)
    y = true_center[1] + true_radius * np.sin(phi) * np.sin(theta)
    z = true_center[2] + true_radius * np.cos(phi)
    noise = np.random.normal(0, 0.05, size=(num_points, 3))
    points = np.column_stack((x, y, z)) + noise

    x0, y0, z0, radius = fit_sphere(points)
    print(f'拟合球心: ({x0:.4f}, {y0:.4f}, {z0:.4f})')
    print(f'拟合半径: {radius:.4f}')


if __name__ == '__main__':
    main()
