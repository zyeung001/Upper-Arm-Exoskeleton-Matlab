# Upper-Arm Exoskeleton — Adaptive vs Fixed Control (MATLAB / Simulink)

Simulation study of a **3-DOF upper-arm exoskeleton** (2 shoulder + 1 elbow)
that compares a **fixed PID controller** against an **adaptive PID controller**
whose gains are scheduled by task difficulty (Fitts' index). The goal is to see
which controller produces better **tracking accuracy** and lower **actuator
effort** across reaching tasks of varying distance and target width.

> Status: **scaffold**. The core model (kinematics, Lagrangian dynamics,
> controllers, metrics, single-reach simulation) is implemented; the experiment
> sweep, plotting, and Simulink models are stubs marked `TODO`.

## The idea

- **Arm:** 3 degrees of freedom — 2 at the shoulder, 1 at the elbow. One DOF
  at a time to start, then combined.
- **Dynamics:** Lagrangian formulation `M(q)q̈ + C(q,q̇)q̇ + G(q) = τ`, built
  from Jacobians, so joint **torque = actuator effort** in the simulation.
- **Forward kinematics** converts joint angles → 3-D hand position, which is
  what we track against each target.
- **Difficulty measure:** Fitts' index `ID = log2(D/W + 1)` from the reach
  distance `D` and target width `W`.
- **Objects** are modelled as boxes/targets in the workspace; each reach is a
  "grab" toward one.
- **Experiment:** trials over different `D` and `W`, run with each controller.
  Fixed PID = **control group**, adaptive PID = **experimental group**. Compare
  effort and accuracy vs difficulty.

## Repository layout

```
src/
  kinematics/   forward/inverse kinematics, rotations, hand Jacobian
  dynamics/     mass matrix, Coriolis, gravity, forward/inverse dynamics
  control/      fixedPIDController, adaptivePIDController, torque saturation
  metrics/      fittsIndex, actuatorEffort, trackingError
  trajectory/   minJerkTrajectory (reference paths)
  simulation/   simulateReach (closed-loop trial)
  model/        armParameters, targetObjects
experiments/    controllerGains, trialConfig, demoSingleReach,
                runTrials, compareControllers        (sweep/plots = TODO)
tests/          runAllTests                          (more asserts = TODO)
simulink/       Simulink models live here            (TODO)
docs/           MODEL.md — maths behind the code
results/        generated data + figures (gitignored)
setupPath.m     add everything to the MATLAB path
```

## Getting started

```matlab
>> setupPath          % add project folders to the path
>> demoSingleReach    % one reach, fixed vs adaptive (starter script)
>> runAllTests        % basic sanity checks
```

Everything reads its physical constants from `armParameters.m` — edit the arm
there and the whole project follows.

## Where to go next (`TODO`)

1. `runTrials.m` — build a target for each `(D, W)` pair and run both
   controllers; save per-trial effort + accuracy to `results/`.
2. `compareControllers.m` — plot effort and accuracy vs `ID` for both groups.
3. `simulink/` — port the plant + control loop into Simulink/Simscape.
4. `docs/MODEL.md` — confirm joint conventions and anthropometric parameters.

## Requirements

- MATLAB (base). The core intentionally avoids extra toolboxes; Simulink /
  Simscape Multibody are optional for the model work in `simulink/`.
