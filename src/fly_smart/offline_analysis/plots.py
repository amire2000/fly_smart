"""Graph selection and cursor management for offline review."""

from __future__ import annotations

from dataclasses import dataclass
from math import nan

from ..simulation.config import SceneConfig, StrikeConfig
from ..simulation.telemetry import FlightLog, TelemetryPlot, make_plot, refresh_plot


@dataclass(frozen=True)
class GraphPanel:
    """A user-selectable graph group and its time-series axes."""

    name: str
    axes: tuple[object, ...]
    default_visible: bool


class GraphPanelRegistry:
    """Reuse the simulation plot and control its visible analysis panels."""

    def __init__(self, config: StrikeConfig | None = None) -> None:
        config = config or StrikeConfig()
        self.plot: TelemetryPlot = make_plot(config, SceneConfig())
        self._cursor_lines = tuple(
            axis.axvline(nan, color="#111827", linestyle="-", linewidth=1.0, label="current sample")
            for axis in self.time_axes
        )
        self.panels = (
            GraphPanel("Velocity", (self.plot.velocity_axis,), True),
            GraphPanel("Path", (self.plot.path_axis, self.plot.path_slope_axis), False),
            GraphPanel("Trajectory command", (self.plot.trajectory_axis, self.plot.trajectory_altitude_axis), True),
            GraphPanel("Guidance", (self.plot.guidance_axis, self.plot.pitch_axis), True),
            GraphPanel("BBox growth", (self.plot.growth_axis,), False),
            GraphPanel("TTC", (self.plot.ttc_axis,), True),
            GraphPanel("Alignment", (self.plot.alignment_axis, self.plot.alignment_angle_axis), False),
            GraphPanel("Altitude", (self.plot.barometer_axis,), True),
        )
        self._apply_visibility()

    @property
    def time_axes(self) -> tuple[object, ...]:
        """Return every time-based primary axis, excluding the x-z path."""
        return (
            self.plot.velocity_axis,
            self.plot.trajectory_axis,
            self.plot.guidance_axis,
            self.plot.growth_axis,
            self.plot.ttc_axis,
            self.plot.alignment_axis,
            self.plot.barometer_axis,
        )

    def set_visible(self, name: str, visible: bool) -> None:
        """Show or hide one named graph panel."""
        for panel in self.panels:
            if panel.name == name:
                for axis in panel.axes:
                    axis.set_visible(visible)
                return
        raise KeyError(name)

    def update(self, log: FlightLog, index: int) -> None:
        """Refresh graph data and place the current-time cursor."""
        refresh_plot(self.plot, log, autoscale=True)
        time_s = log.time_s[index]
        for line in self._cursor_lines:
            line.set_xdata([time_s, time_s])
        self.plot.figure.canvas.draw_idle()

    def _apply_visibility(self) -> None:
        for panel in self.panels:
            for axis in panel.axes:
                axis.set_visible(panel.default_visible)
