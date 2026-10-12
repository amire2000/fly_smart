# Bearing, TTC and image-error diagram plan

Superseded design: [dy-bearing-smooth-descent.md](dy-bearing-smooth-descent.md)
makes vertical bearing an active descent input. The linked SVG now illustrates
that revised proposal; the original scope below records the earlier design.

Create `docs/trajectory_control_inputs.svg` as a companion to the proposed
trajectory-stage drawing. Show a side-view trajectory with aligned control
lanes and a sensor-to-command diagram.

Make the agreed control ownership explicit: horizontal compensated bearing
from bbox centre and IMU drives roll while visual data is fresh; conservative
pitch requests forward motion; confident TTC plus fused altitude drives the
descent target and vertical feedback drives thrust. Raw dx/dy are image-centre
offsets, not world-frame control errors. Vertical bearing is measured but is
not a direct descent-feedback term in this proposal: its quality and target
visibility affect gating and terminal checks. Do not imply that image-centre
alignment guarantees centre contact or that altitude hold stops translation.

Mark the diagram as proposed and schematic, label transition conditions,
include target-loss recovery, and link it from the trajectory document. Check
SVG XML and render/inspect the final layout. No flight behavior changes.

Implementation commit: `690d24e`.

## Implementation update

The vector asset now depicts implemented DY PD, bearing-guarded TTC blending,
shared vertical PI and a braked level finish. XML parsing and PNG rendering
are part of validation; the curve remains schematic, not measured telemetry.
Implementation commit: `690d24e`.
