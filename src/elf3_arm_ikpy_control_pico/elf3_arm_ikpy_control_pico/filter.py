"""Small multi-channel filters used by the Pico IK bridge."""

from collections import deque
from typing import Iterable, Optional

import numpy as np
from scipy import signal


def _readings(values: Iterable[float], channels: int) -> np.ndarray:
    readings = np.asarray(list(values), dtype=float)
    if readings.shape != (channels,):
        raise ValueError(f'expected {channels} channels, got {readings.size}')
    return readings


class MultiChannelLowPassFilter:
    """First-order low-pass filter for vector-valued sensor readings."""

    def __init__(self, num_channels: int = 4, alpha: float = 0.3):
        if num_channels <= 0:
            raise ValueError('num_channels must be positive')
        if not 0.0 < alpha <= 1.0:
            raise ValueError('alpha must be in (0, 1]')

        self.num_channels = num_channels
        self.alpha = alpha
        self.last_values = np.zeros(num_channels)
        self.is_initialized = False

    def filter(self, sensor_readings: Iterable[float]) -> np.ndarray:
        readings = _readings(sensor_readings, self.num_channels)
        if not self.is_initialized:
            self.last_values = readings.copy()
            self.is_initialized = True
            return readings

        filtered = self.alpha * readings
        filtered += (1.0 - self.alpha) * self.last_values
        self.last_values = filtered.copy()
        return filtered

    def reset(self) -> None:
        self.last_values = np.zeros(self.num_channels)
        self.is_initialized = False


class MultiChannelMovingAverage:
    """Moving-average filter for vector-valued sensor readings."""

    def __init__(self, num_channels: int = 4, window_size: int = 5):
        if num_channels <= 0:
            raise ValueError('num_channels must be positive')
        if window_size <= 0:
            raise ValueError('window_size must be positive')

        self.num_channels = num_channels
        self.window_size = window_size
        self.buffers = [
            deque(maxlen=window_size)
            for _ in range(num_channels)
        ]

    def filter(self, sensor_readings: Iterable[float]) -> np.ndarray:
        readings = _readings(sensor_readings, self.num_channels)
        for index, reading in enumerate(readings):
            self.buffers[index].append(reading)
        return np.asarray([np.mean(buffer) for buffer in self.buffers])

    def reset(self) -> None:
        for buffer in self.buffers:
            buffer.clear()


class MultiChannelFIRFilter:
    """FIR low-pass filter for vector-valued sensor readings."""

    def __init__(
        self,
        num_channels: int = 4,
        cutoff_freq: float = 10.0,
        sampling_freq: float = 100.0,
        filter_order: int = 20,
        window: str = 'hamming',
    ):
        if num_channels <= 0:
            raise ValueError('num_channels must be positive')
        if cutoff_freq <= 0.0 or sampling_freq <= 0.0:
            raise ValueError('frequencies must be positive')
        if cutoff_freq >= sampling_freq / 2.0:
            raise ValueError('cutoff_freq must be below Nyquist')
        if filter_order <= 0:
            raise ValueError('filter_order must be positive')

        self.num_channels = num_channels
        self.cutoff_freq = cutoff_freq
        self.sampling_freq = sampling_freq
        self.filter_order = filter_order
        self.window = window
        self.taps = self._design_fir_filter()
        self.buffers = np.zeros((num_channels, len(self.taps)))
        self.buffer_index = 0

    def _design_fir_filter(self) -> np.ndarray:
        nyquist = self.sampling_freq / 2.0
        return signal.firwin(
            numtaps=self.filter_order,
            cutoff=self.cutoff_freq / nyquist,
            window=self.window,
            pass_zero=True,
        )

    def filter(self, sensor_readings: Iterable[float]) -> np.ndarray:
        readings = _readings(sensor_readings, self.num_channels)
        self.buffers[:, self.buffer_index] = readings

        filtered = np.zeros(self.num_channels)
        read_indices = (
            self.buffer_index - np.arange(len(self.taps))
        ) % len(self.taps)
        for index in range(self.num_channels):
            filtered[index] = np.dot(
                self.buffers[index, read_indices],
                self.taps,
            )

        self.buffer_index = (self.buffer_index + 1) % len(self.taps)
        return filtered

    def reset(self) -> None:
        self.buffers.fill(0.0)
        self.buffer_index = 0

    def get_filter_coefficients(self) -> np.ndarray:
        return self.taps.copy()

    def get_frequency_response(
        self,
        n_points: int = 512,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        w, h = signal.freqz(self.taps, worN=n_points)
        frequencies = w * self.sampling_freq / (2.0 * np.pi)
        magnitude = 20.0 * np.log10(np.maximum(np.abs(h), 1e-12))
        phase = np.angle(h)
        return frequencies, magnitude, phase


class RealTimeFIRFilter:
    """FIR filter using caller-provided coefficients."""

    def __init__(
        self,
        num_channels: int = 4,
        coefficients: Optional[Iterable[float]] = None,
    ):
        if num_channels <= 0:
            raise ValueError('num_channels must be positive')

        taps = np.ones(5) / 5.0 if coefficients is None else coefficients
        self.taps = np.asarray(list(taps), dtype=float)
        if self.taps.size == 0:
            raise ValueError('coefficients must not be empty')

        self.num_channels = num_channels
        self.filter_order = len(self.taps)
        self.buffers = np.zeros((num_channels, self.filter_order))
        self.write_index = 0
        self.conv_indices = np.arange(self.filter_order)

    def filter(self, sensor_readings: Iterable[float]) -> np.ndarray:
        readings = _readings(sensor_readings, self.num_channels)
        self.buffers[:, self.write_index] = readings

        filtered = np.zeros(self.num_channels)
        read_indices = (
            self.write_index - self.conv_indices
        ) % self.filter_order
        for index in range(self.num_channels):
            filtered[index] = np.dot(
                self.buffers[index, read_indices],
                self.taps,
            )

        self.write_index = (self.write_index + 1) % self.filter_order
        return filtered

    def reset(self) -> None:
        self.buffers.fill(0.0)
        self.write_index = 0


class SimpleFIRFilter:
    """Moving-window FIR filter with configurable coefficients."""

    def __init__(self, num_channels: int = 4, window_size: int = 5):
        if num_channels <= 0:
            raise ValueError('num_channels must be positive')
        if window_size <= 0:
            raise ValueError('window_size must be positive')

        self.num_channels = num_channels
        self.window_size = window_size
        self.coefficients = np.ones(window_size) / float(window_size)
        self.buffers = [
            deque(maxlen=window_size)
            for _ in range(num_channels)
        ]

    def filter(self, sensor_readings: Iterable[float]) -> np.ndarray:
        readings = _readings(sensor_readings, self.num_channels)
        filtered = []

        for index, reading in enumerate(readings):
            self.buffers[index].append(reading)
            buffer = np.asarray(self.buffers[index], dtype=float)
            coeffs = self.coefficients[-len(buffer):]
            coeffs = coeffs / np.sum(coeffs)
            filtered.append(np.dot(buffer, coeffs))

        return np.asarray(filtered)

    def set_coefficients(self, coefficients: Iterable[float]) -> None:
        coefficients = np.asarray(list(coefficients), dtype=float)
        if len(coefficients) != self.window_size:
            raise ValueError(f'coefficients length must be {self.window_size}')
        self.coefficients = coefficients

    def reset(self) -> None:
        for buffer in self.buffers:
            buffer.clear()
