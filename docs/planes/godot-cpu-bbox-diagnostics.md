# Godot CPU and distant-target detection

The live plot subprocess must yield between refreshes; a 1 ms Matplotlib pause
created a busy loop and consumed a full CPU core. Its refresh rate remains 5 Hz,
but the event-loop pause is capped at 20 ms.

Godot frames are RGB and the existing detector converts them to HSV correctly.
At the 80 m preset the red cube can occupy fewer than 80 contour pixels, so the
minimum accepted contour area is reduced to 20 px². The red-only HSV mask still
rejects non-red objects.
