# PPO hover training and evaluation

The existing `PyBulletHoverEnv` is the learning boundary. Stable-Baselines3
owns PPO optimization; the environment continues to own physics, observations,
rewards, and termination. `fly-smart-rl-train` creates a generated run folder,
saves checkpoints and the final model, then evaluates the final policy with
deterministic actions and writes `evaluation.json`.

Training is headless by default. Rendering is an evaluation-only option so the
GUI cannot become part of the training contract. The first workflow measures
hover stabilization only; mission tracking and TensorBoard are deferred. The
hover reward now strongly penalizes altitude error, actuator effort, and smooth
control violations, with an additional terminal failure penalty so climbing or
descending out of the hover region cannot produce a good episode score.
