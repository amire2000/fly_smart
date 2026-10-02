# Separate live plot process

Keep Matplotlib out of the PyBullet physics loop. When live plotting is
enabled, the runner sends compact telemetry samples over best-effort UDP on
`127.0.0.1:9103`; a child process owns the Matplotlib figure and redraws it.
The physics process remains authoritative for `FlightLog`, CSV, and final PNG
output. Dropping a display sample is harmless because the saved telemetry is
complete.

```mermaid
sequenceDiagram
    participant S as PyBullet runner
    participant U as UDP 127.0.0.1:9103
    participant P as Plot subprocess
    participant M as Matplotlib

    S->>U: telemetry_sample JSON (about 5 Hz)
    U->>P: latest sample
    P->>P: append sample to child FlightLog
    P->>M: redraw live figure
    S->>U: reset / collision / close lifecycle message
```

Plotting is display-only: a missing or slow child never blocks physics, and
closing the plot does not stop the simulation. Final CSV/PNG generation stays
in the parent process after the run.

The grouped simulation configuration is passed through Python's spawn pickle
boundary; its compatibility attribute lookup must tolerate fields being
restored after object construction.
