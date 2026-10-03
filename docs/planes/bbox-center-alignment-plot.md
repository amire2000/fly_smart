# Bbox-center alignment plot

Telemetry records the red-target bbox center error in pixels and displays it
with calibrated horizontal and vertical angular offsets. Pixel coordinates use
image convention (right/down positive). Angles use the configured camera width,
height, and horizontal FOV; vertical FOV is derived from the aspect ratio.

The live UDP sample and CSV include the pixel errors. The static/live plot
derives the angular series from those values, keeping one source of truth for
the recorded detection geometry.
