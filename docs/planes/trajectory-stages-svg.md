# Trajectory stages SVG plan

Superseded design: [dy-bearing-smooth-descent.md](dy-bearing-smooth-descent.md)
replaces initial altitude hold and pause-on-TTC-loss with smooth DY descent.
The linked SVG now illustrates that revised proposal.

Create `docs/trajectory_stages.svg` to illustrate the proposed bearing-guidance
and confidence-gated TTC design, rather than the currently implemented flight
behavior. Show the normal altitude-hold approach, descent activation, terminal
continuation and contact, plus a separate TTC-loss/recovery path. Label sensor
influence, pitch/thrust ownership and the proposed timing gates.

Use schematic distances and intermediate heights. Label the 15 m takeoff and
1 m target-altitude examples, and distinguish commanded altitude hold from
physical motion that lags commands. Include accessible SVG title/description.

Link the drawing from `docs/trajectory_algo.md` with an explicit proposed-design
caption. Validate SVG XML, render and visually inspect it. No controller,
configuration, dependencies or generated flight data changes.

Implementation commit: `pending`.

## Implementation update

The vector asset now depicts implemented DY PD, bearing-guarded TTC blending,
shared vertical PI and a braked level finish. XML parsing and PNG rendering
are part of validation; the curve remains schematic, not measured telemetry.
Implementation commit: `pending`.
