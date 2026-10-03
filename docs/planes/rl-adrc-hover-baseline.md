# ADRC hover baseline

The RL branch includes a small active disturbance rejection controller (ADRC)
beside the PID baseline. Each channel uses a linear extended-state observer
(ESO) to estimate output, rate, and lumped disturbance, then applies bounded
linear state-error feedback. Throttle is a correction around the calculated
hover PWM; roll, pitch, and yaw produce normalized torque actions.

`fly-smart-adrc-hover` runs the same stationary 3 m, 10-second real-time check
as the PID example. It is a deterministic controller baseline, not an RL policy
and not a replacement for the Gymnasium environment.
