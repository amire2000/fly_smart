# Gymnasium hover environment

The first reinforcement-learning seam is a state-based hover task around the
existing PyBullet `PhysicsEngine`. The Gymnasium environment owns episode
lifecycle only; a bullet simulator adapter owns the world and physics, a task
module owns observations/rewards/termination, and an optional GUI observer
shows progress without affecting the episode.

The agent controls collective PWM plus roll, pitch, and yaw torque. It observes
altitude error, velocity, attitude, body rates, motor RPMs, and battery state;
horizontal position and camera pixels are intentionally excluded. The first
environment is single-instance because current PyBullet helpers use the global
client API. Vectorized clients are a later seam.
