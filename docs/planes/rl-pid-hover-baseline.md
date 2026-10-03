# PID hover baseline

The PPO hover task has a deterministic baseline with the same normalized
four-action interface: collective throttle, roll torque, pitch torque, and yaw
torque. A throttle PID is centered on the calculated per-motor hover PWM;
attitude PIDs target level roll/pitch/yaw and use measured body rates for
damping. Each output is bounded to the action range and integrators are reset
between runs.

`fly-smart-pid-hover` runs a stationary 3 m check for 10 seconds. It passes
only when altitude remains within 0.15 m and all Euler angles remain within
5 degrees. This is a trim/steady-state check, not a disturbance-recovery test.
