"""Simple altitude sensor model used by TTC guidance."""

from dataclasses import dataclass

import numpy as np

from .config import StrikeConfig


@dataclass(frozen=True)
class BarometerReading:
    """One raw and filtered barometer measurement for the guidance cycle."""

    altitude_m: float
    vertical_velocity_mps: float
    raw_altitude_m: float | None = None


class Barometer:
    """Sample BMP388-like altitude independently of camera frames.

    The model combines per-reading Gaussian noise, a fixed reference bias, and
    an optional seeded random walk for slow field drift.  Its defaults use the
    BMP388 full-bandwidth 1.2 Pa noise figure, approximately 0.10 m in altitude.
    """

    def __init__(self, config: StrikeConfig) -> None:
        self.config = config
        self.period_s = 1 / config.barometer_sample_hz
        self.rng = np.random.default_rng(config.random_seed)
        self.last_time_s: float | None = None
        self.last_altitude_m: float | None = None
        self.filtered_altitude_m: float | None = None
        self.vertical_velocity_mps = 0.0
        self.drift_m = 0.0

    def sample(self, true_altitude_m: float, now_s: float) -> BarometerReading | None:
        """Return one scheduled noisy altitude reading, or ``None`` between ticks."""
        if self.last_time_s is not None and now_s - self.last_time_s < self.period_s:
            return None
        elapsed_s = 0.0 if self.last_time_s is None else now_s - self.last_time_s
        if self.config.barometer_drift_sigma_m_per_sqrt_s:
            self.drift_m += self.rng.normal(0.0, self.config.barometer_drift_sigma_m_per_sqrt_s * elapsed_s**0.5)
        raw_altitude_m = (
            true_altitude_m
            + self.config.barometer_bias_m
            + self.drift_m
            + self.rng.normal(0.0, self.config.barometer_noise_sigma_m)
        )
        if self.filtered_altitude_m is None:
            self.filtered_altitude_m = raw_altitude_m
        else:
            old = self.config.barometer_altitude_old_weight
            self.filtered_altitude_m = old * self.filtered_altitude_m + (1 - old) * raw_altitude_m
        altitude_m = self.filtered_altitude_m
        if self.last_time_s is not None and self.last_altitude_m is not None:
            measured_velocity = (altitude_m - self.last_altitude_m) / (now_s - self.last_time_s)
            old = self.config.barometer_velocity_old_weight
            self.vertical_velocity_mps = old * self.vertical_velocity_mps + (1 - old) * measured_velocity
        self.last_time_s, self.last_altitude_m = now_s, altitude_m
        return BarometerReading(altitude_m, self.vertical_velocity_mps, raw_altitude_m)


@dataclass(frozen=True)
class VerticalImuReading:
    """One noisy world-up linear-acceleration sample for vertical fusion."""

    vertical_acceleration_mps2: float


class VerticalImu:
    """Simulate ICM-42688-P-inspired acceleration noise with a drifting bias.

    The first fusion lesson uses ideal PyBullet attitude, so this sensor emits
    already world-aligned vertical acceleration. Attitude/gyro estimation is a
    later extension.
    """

    def __init__(self, config: StrikeConfig) -> None:
        self.config = config
        self.rng = np.random.default_rng(config.random_seed + 1)
        self.bias_mps2 = self.rng.normal(0.0, config.accelerometer_initial_bias_sigma_mps2)
        self.last_time_s: float | None = None

    def sample(self, true_vertical_acceleration_mps2: float, now_s: float) -> VerticalImuReading:
        """Return one noisy acceleration sample and advance its slow bias drift."""
        elapsed_s = 0.0 if self.last_time_s is None else now_s - self.last_time_s
        if self.config.accelerometer_bias_random_walk_mps2_per_sqrt_s:
            self.bias_mps2 += self.rng.normal(0.0, self.config.accelerometer_bias_random_walk_mps2_per_sqrt_s * elapsed_s**0.5)
        self.last_time_s = now_s
        return VerticalImuReading(true_vertical_acceleration_mps2 + self.bias_mps2 + self.rng.normal(0.0, self.config.accelerometer_noise_sigma_mps2))


@dataclass(frozen=True)
class VerticalEstimate:
    """Fused world-up height and vertical speed supplied to guidance."""

    altitude_m: float
    vertical_velocity_mps: float


class VerticalEstimator:
    """Predict vertical state from IMU acceleration and correct it with barometer altitude."""

    def __init__(self, config: StrikeConfig, initial_altitude_m: float) -> None:
        self.config = config
        self.altitude_m = initial_altitude_m
        self.vertical_velocity_mps = 0.0

    def update(self, imu: VerticalImuReading, dt_s: float, barometer: BarometerReading | None = None) -> VerticalEstimate:
        """Propagate with IMU, then apply an alpha-beta barometer correction when available."""
        self.vertical_velocity_mps += imu.vertical_acceleration_mps2 * dt_s
        self.altitude_m += self.vertical_velocity_mps * dt_s
        if barometer is not None:
            error_m = barometer.altitude_m - self.altitude_m
            self.altitude_m += self.config.vertical_estimator_alpha * error_m
            self.vertical_velocity_mps += self.config.vertical_estimator_beta * error_m / dt_s
        return VerticalEstimate(self.altitude_m, self.vertical_velocity_mps)
