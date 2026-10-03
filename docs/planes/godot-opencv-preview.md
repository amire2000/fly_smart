# Godot FPV overlay

The Godot FPV texture now receives the Python red-target bbox and live flight
telemetry through the existing pose packet. Godot draws the bbox and compact
telemetry directly over the camera view while PyBullet remains in DIRECT mode.

The Godot VS Code task no longer opens an OpenCV viewer. Validation is CLI
argument parsing, Godot scene startup, and the maintained Python test suite.
