# Offline flight analyzer

## Intent

Provide a separate desktop tool for inspecting recorded flight attempts without
running the simulator again. The tool selects a run and attempt, synchronizes
the recorded movie with telemetry, supports sample-by-sample playback, and
shows the active vertical-control state and selected graphs.

## Package boundary

All analyzer-specific code lives in `fly_smart.offline_analysis`. It may use
shared core types and simulation telemetry/configuration helpers, but the core
and simulation packages do not import the analyzer.

The package is divided by responsibility: artifact loading, playback timing,
video access, state presentation, graph selection, and Tkinter composition.
Small protocols keep playback and presentation testable with in-memory data
and a no-video source.

## Synchronization and failure behavior

Telemetry timestamps are authoritative. The selected timestamp maps to the
nearest movie frame using the recorded video FPS. Missing movies produce a
graphs-only review; missing or malformed telemetry exits with an actionable
error.

The selector accepts either a run folder containing `attempt-*` directories or
one direct attempt folder. The first version reviews one attempt at a time and does not write notes,
bookmarks, screenshots, or changes to generated run data.
