"""Small reusable PID controller for the course flight examples."""

from dataclasses import dataclass
from math import isfinite


@dataclass
class PID:
    """PID with externally measured rate and a bounded integral."""

    kp: float
    ki: float
    kd: float
    integral_limit: float = 0.2
    integral: float = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def update_terms(self, error: float, rate: float, dt: float = 1 / 120) -> tuple[float, float, float]:
        """Advance finite PID inputs with an explicitly positive sample interval."""
        if not all(isfinite(value) for value in (error, rate, dt)) or dt <= 0:
            raise ValueError("PID inputs must be finite and dt positive")
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral + error * dt))
        return self.kp * error, self.ki * self.integral, -self.kd * rate

    def update(self, error: float, rate: float, dt: float = 1 / 120) -> float:
        return sum(self.update_terms(error, rate, dt))
