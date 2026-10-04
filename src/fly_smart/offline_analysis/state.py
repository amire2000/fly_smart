"""Current-sample state formatting for the offline analyzer."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

from ..simulation.telemetry import FlightLog


@dataclass(frozen=True)
class FlightState:
    """Human-readable state values for the side panel and video overlay."""

    time_s: float
    phase: str
    vertical_mode: str
    vx_mps: float
    command_vx_mps: float
    vz_mps: float
    command_vz_mps: float
    altitude_m: float
    ttc_s: float
    bbox_scale_px: float
    bbox_growth_px_s: float


class FlightStatePresenter:
    """Convert a FlightLog sample into stable display values."""

    def at(self, log: FlightLog, index: int) -> FlightState:
        """Return the state at a clamped telemetry index."""
        index = max(0, min(index, len(log.time_s) - 1))
        return FlightState(
            log.time_s[index],
            log.phase[index],
            log.vertical_control_mode[index].upper(),
            log.vx_mps[index],
            log.command_vx_mps[index],
            log.vz_mps[index],
            log.command_vz_mps[index],
            log.barometer_filtered_altitude_m[index],
            log.ttc_s[index],
            log.bbox_scale_px[index],
            log.bbox_growth_px_s[index],
        )

    def text(self, state: FlightState) -> str:
        """Format a readable fixed state summary."""
        return "\n".join(
            (
                f"time: {state.time_s:.3f} s",
                f"phase: {state.phase}",
                f"vertical control: {state.vertical_mode}",
                f"vx: {self._number(state.vx_mps)} / cmd {self._number(state.command_vx_mps)} m/s",
                f"vz: {self._number(state.vz_mps)} / cmd {self._number(state.command_vz_mps)} m/s",
                f"altitude: {self._number(state.altitude_m)} m",
                f"TTC: {self._number(state.ttc_s)} s",
                f"bbox: {self._number(state.bbox_scale_px)} px; growth {self._number(state.bbox_growth_px_s)} px/s",
            )
        )

    @staticmethod
    def _number(value: float) -> str:
        return f"{value:.3f}" if isfinite(value) else "—"
