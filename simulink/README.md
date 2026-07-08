# Simulink models

Place Simulink models (`.slx`) here. Suggested models to build:

- **`armPlant.slx`** — the 3-DOF arm dynamics as a plant block. Wrap the
  MATLAB functions (`massMatrix`, `coriolisMatrix`, `gravityVector`,
  `forwardDynamics`) in a MATLAB Function block, or use Simscape Multibody
  bodies driven by the same `armParameters`.
- **`controlLoop.slx`** — closed loop: reference trajectory → controller
  (fixed or adaptive PID) → plant → sensors, logging torque and hand position.

Keep the physics defined in `src/` (`armParameters.m`) as the single source of
truth and reference it from the models so MATLAB and Simulink stay in sync.

> Build artifacts (`slprj/`, `*.slxc`) are gitignored — commit only the `.slx`.
