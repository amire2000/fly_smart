# TTC trust gate and bounded descent correction

Long TTC estimates are not trusted as final impact timing. The planner treats
TTC values above 8 seconds like unavailable TTC, keeping the gentle fallback
descent and takeoff-altitude reference until the estimate is short enough to
drive the final impact-altitude trajectory.

The vertical position-error correction is independently limited to 2 m/s
before it is combined with the TTC trajectory velocity. The existing overall
descent/climb limits remain the final safety bounds.
