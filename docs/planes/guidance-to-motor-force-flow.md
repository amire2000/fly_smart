# Guidance command to motor force flow

## Simple block diagram

```text
Sensors / camera / barometer / IMU
              │
              ▼
       StrikeGuidance.update()
       ┌──────────────────────┐
       │ altitude / speed /   │
       │ vertical-velocity PID│
       └──────────┬───────────┘
                  │ GuidanceCommand
                  │ thrust_n + pitch_target_rad
                  ▼
       AttitudeController.update(IMU)
       ┌──────────────────────┐
       │ roll / pitch / yaw   │
       │ attitude PID loops   │
       └──────────┬───────────┘
                  │ body torque (τroll, τpitch, τyaw)
                  ▼
        total thrust ÷ 4, then PWM conversion
                  │ collective PWM
                  ▼
             PhysicsEngine.step()
       ┌──────────────────────┐
       │ thrust → RPM          │
       │ torque mixer           │
       │ battery + motor lag   │
       └──────────┬───────────┘
                  │ four motor RPMs
                  ▼
       per-rotor thrust and reaction torque
                  │
                  ▼
        PyBullet external forces/torques
                  │
                  ▼
             vehicle state update
```

## 1. Guidance produces the high-level command

`StrikeGuidance.update()` is the mission-level controller. It consumes the
barometer, camera/TTC state, forward velocity, measured pitch, and alignment
error. Its output is [`GuidanceCommand`](../../src/fly_smart/guidance.py),
which contains:

- `thrust_n`: collective vehicle thrust in newtons;
- `pitch_target_rad`: desired pitch attitude;
- `trajectory`: forward and vertical velocity targets;
- phase/mode metadata such as `DY`, `TTC`, `COMMIT`, or `ABORT`.

The guidance command does not contain a complete RPY command. In this mission,
roll and yaw targets are zero and pitch is the active attitude target.

Guidance contains the outer control loops:

- `TAKEOFF`: altitude PID produces collective thrust;
- `TRACK`: forward-speed PID produces pitch, and vertical-velocity PID produces
  the thrust correction needed to follow the DY/TTC vertical target;
- `COMMIT`: the last pitch is held while the vertical-velocity PID continues;
- `ABORT`: pitch is neutralized and altitude control damps vertical motion.

The thrust is compensated for measured pitch so that tilting the vehicle does
not remove the requested world-vertical lift.

## 2. Attitude control converts RPY error to body torque

At each control tick, the runner passes the measured IMU attitude and rates to
[`AttitudeController.update()`](../../src/fly_smart/common/flight_control.py).
The controller compares the desired and measured roll, pitch, and yaw and runs
three PID controllers:

```text
attitude error + angular rate
            │
            ▼
 roll PID / pitch PID / yaw PID
            │
            ▼
 (τroll, τpitch, τyaw) in N·m
```

For the normal strike path, the call is effectively:

```python
torque = attitude_controller.update(
    read_imu(drone),
    yaw_target=0.0,
    pitch_target=command.pitch_target_rad,
)
```

That means the pitch target from `GuidanceCommand` is not sent directly to a
motor. It first passes through the attitude feedback loop and becomes a pitch
torque request.

## 3. Total thrust becomes a collective PWM command

The runner treats `GuidanceCommand.thrust_n` as total collective thrust. Before
calling the physics engine it divides the request evenly among four motors:

```python
per_motor_thrust = clamp(command.thrust_n / 4, 0, max_thrust_per_motor_n)
pwm = engine.pwm_from_thrust(per_motor_thrust)
```

The mapping in `drone_physics.py` is square-root based:

```text
normalized = sqrt(per_motor_thrust / max_thrust_per_motor)
PWM = 1000 + 1000 × normalized
```

So the PWM value represents the equal per-motor collective baseline. Attitude
corrections are added later by the mixer.

## 4. The mixer adds torque corrections to the four motors

`PhysicsEngine.step()` first converts the collective PWM back to a per-motor
requested thrust. It then calls `_mix_motor_thrusts()` with:

```text
collective per-motor thrust
(τroll, τpitch, τyaw)
```

The mixer computes four thrust deltas using the pseudoinverse of the vehicle
allocation matrix. The matrix is built from:

- each rotor's lateral position for roll torque;
- each rotor's longitudinal position for pitch torque;
- each rotor's configured rotation sign for yaw reaction torque.

Conceptually:

```text
motor_thrusts = collective + scale × allocation_pseudoinverse × body_torque
```

The common scale factor reduces the torque correction if any motor would go
below zero or above `max_thrust_per_motor_n`. This preserves valid motor
commands but means attitude authority is reduced when collective thrust is
near a motor limit.

## 5. Motor model and force application

Each mixed motor thrust is converted to a target RPM using:

```text
target_rpm = sqrt(requested_thrust / thrust_coefficient)
```

The battery model scales the target according to bus voltage and current
limits. The actual RPM follows the target through a first-order motor response:

```text
rpm_actual += (time_step / motor_time_constant) × (rpm_target - rpm_actual)
```

Actual rotor thrust is then calculated from `thrust_coefficient × rpm²`, with
optional inflow and ground-effect multipliers. For every rotor, the engine
applies:

- an upward thrust force at the rotor link;
- a reaction torque around the body z-axis;
- optional blade-flapping force.

The engine also applies rotor drag, body drag, angular damping, and optional
gyroscopic torque before calling `p.stepSimulation()`.

## 6. Is there a control loop in the way?

Yes. There are nested feedback loops; this is not a direct
`GuidanceCommand → motor force` path.

```text
Mission guidance loop
  altitude / speed / vertical velocity → thrust and pitch target

Attitude loop
  measured RPY + rates → roll/pitch/yaw body torque

Actuator loop
  requested motor RPM → lagged actual RPM

Physics loop
  motor forces → vehicle motion → sensors → next control update
```

The normal timing domains are:

- PyBullet physics: typically 240 Hz;
- guidance and attitude control: typically 120 Hz;
- camera/TTC updates: typically 30 Hz;
- telemetry/live plot display: separate from the physics authority.

Between control ticks, the latest thrust and torque command is reused while
the physics engine continues stepping. Camera and TTC values therefore update
more slowly than the motor-force integration.

## Source locations

- [guidance.py](../../src/fly_smart/guidance.py): mission phases, PID guidance,
  and `GuidanceCommand`.
- [flight_control.py](../../src/fly_smart/common/flight_control.py): RPY
  attitude PID and body-torque output.
- [runner.py](../../src/fly_smart/simulation/runner.py): control-loop timing,
  thrust division, PWM conversion, and engine call.
- [drone_physics.py](../../src/fly_smart/simulation/drone_physics.py): mixer,
  battery/motor response, rotor forces, drag, and PyBullet integration.
