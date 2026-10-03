# ADRC hover baseline

The ADRC branch includes a small active disturbance rejection controller.
Each channel uses a linear extended-state observer
(ESO) to estimate output, rate, and lumped disturbance, then applies bounded
linear state-error feedback. Throttle is a correction around the calculated
hover PWM; roll, pitch, and yaw produce normalized torque actions.

`fly-smart-adrc-hover` starts at 0.05 m, takes off to 3 m, and holds the target
for 10 seconds by default. Use `--seconds` to extend the hold and
`--takeoff-timeout` to bound the climb phase. With `--render`, the PyBullet
window displays phase, time, altitude, attitude, and all four actions. It is a
deterministic standalone baseline and does not import Gymnasium or
Stable-Baselines3.
