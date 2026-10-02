"""Small reusable state container for interactive simulation controls."""

from dataclasses import dataclass


@dataclass
class SimulationControls:
    """Mutable button state read by a simulation loop between physics ticks."""

    running: bool = False
    reset_requested: bool = False
    stop_requested: bool = False
    exit_requested: bool = False
    def start(self) -> None:
        """Allow the simulation loop to advance physics."""
        self.running = True

    def stop(self) -> None:
        """Pause the simulation loop without resetting its current state."""
        self.running = False

    def request_reset(self) -> None:
        """Pause the loop and ask its owner to restore its initial state."""
        self.running = False
        self.reset_requested = True

    def consume_reset(self) -> bool:
        """Return and clear one pending reset request."""
        requested, self.reset_requested = self.reset_requested, False
        return requested

    def request_stop(self) -> None:
        """Ask the owner to finalize the current attempt and exit."""
        self.stop_requested = True

    def consume_stop(self) -> bool:
        """Return and clear one pending stop request."""
        requested, self.stop_requested = self.stop_requested, False
        return requested

    def request_exit(self) -> None:
        """Ask the simulation loop to close after its current GUI update."""
        self.exit_requested = True
