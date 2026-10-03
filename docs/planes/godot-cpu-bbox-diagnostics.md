# Godot CPU and distant-target detection

The live plot subprocess must yield between refreshes. Matplotlib's TkAgg
``plt.pause()`` services the event loop by polling, so even a 100 ms pause can
consume a full CPU core for this eight-axis figure. Keep telemetry redraws at
3 Hz, and use ``time.sleep`` for the remaining wait. This preserves the
display rate without a plotter busy loop; ``refresh_plot`` already flushes GUI
events after each redraw.

Validation: the running Godot session showed the plot child at about 96% CPU
while the simulation and Godot stayed near 12% and 14%. A full-figure harness
with 30 ``plt.pause(0.10)`` calls consumed about 9.9 CPU seconds; replacing
the wait with ``time.sleep`` should make the idle path sleep instead of
polling.

Godot frames are RGB and the existing detector converts them to HSV correctly.
At the 80 m preset the red cube can occupy fewer than 80 contour pixels, so the
minimum accepted contour area is reduced to 20 px². The red-only HSV mask still
rejects non-red objects.
