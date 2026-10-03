"""Small dependency-free linear ADRC/ESO controller."""

from dataclasses import dataclass


@dataclass(frozen=True)
class ADRC:
    """Third-order linear ESO with bounded linear state-error feedback."""

    bandwidth: float
    b0: float
    output_limit: float
    z1: float = 0.0
    z2: float = 0.0
    z3: float = 0.0
    last_output: float = 0.0

    def __post_init__(self) -> None:
        if self.bandwidth <= 0.0 or self.b0 <= 0.0 or self.output_limit <= 0.0:
            raise ValueError("ADRC bandwidth, b0, and output_limit must be positive")

    def reset(self, measurement: float) -> "ADRC":
        """Return a controller initialized at the current measured output."""
        return ADRC(self.bandwidth, self.b0, self.output_limit, measurement, 0.0, 0.0, 0.0)

    def update(self, target: float, measurement: float, dt: float) -> tuple[float, "ADRC"]:
        """Return bounded control and the next observer state."""
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        beta1 = 3.0 * self.bandwidth
        beta2 = 3.0 * self.bandwidth**2
        beta3 = self.bandwidth**3
        kp = self.bandwidth**2
        kd = 2.0 * self.bandwidth
        error = self.z1 - measurement
        z1 = self.z1 + dt * (self.z2 - beta1 * error)
        z2 = self.z2 + dt * (self.z3 + self.b0 * self.last_output - beta2 * error)
        z3 = self.z3 + dt * (-beta3 * error)
        raw = (kp * (target - z1) - kd * z2 - z3) / self.b0
        output = max(-self.output_limit, min(self.output_limit, raw))
        return output, ADRC(self.bandwidth, self.b0, self.output_limit, z1, z2, z3, output)
