# ADRC pitch and thrust guidance

TRACK and COMMIT use linear active-disturbance-rejection observers instead of
the forward-speed and vertical-velocity PID loops. The pitch channel regulates
forward velocity and outputs a bounded pitch target. The collective channel
regulates the corrected vertical-velocity target and outputs a bounded thrust
correction around hover thrust.

The TTC gate remains unchanged. Valid TTC supplies the planned descent target;
invalid TTC selects the existing DY fallback, including compensated bbox DY
correction when available. Both modes update the same observers, so changing
between TTC and DY changes only the target and does not reset controller state.
TAKEOFF and ABORT retain their altitude PID safety loops. Existing PID fields
remain accepted in YAML for compatibility and rollback.
