# TTC seconds plot

The telemetry figure gains a dedicated time-to-contact subplot below the
alpha-beta bbox-growth plot. It displays the existing raw and filtered TTC
series in seconds, but renders them only after filtered TTC first enters the
configured activation gate. A vertical marker identifies that crossing.

The alpha-beta growth subplot gets a matching marker at the first valid
filtered-growth sample. Late collision spikes are masked from the display
using a robust per-log ceiling; raw values remain unchanged in telemetry and
CSV output.
