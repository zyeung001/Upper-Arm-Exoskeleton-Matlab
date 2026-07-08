# Model notes

Reference for the maths behind the code. Expand as the model is refined.

## Kinematic model (3 DOF)

World frame: `x` forward, `y` left, `z` up. The arm hangs down `-z` at zero.

| Joint | Meaning                        | Axis                     |
|-------|--------------------------------|--------------------------|
| `q1`  | shoulder flexion/extension     | world `y`                |
| `q2`  | shoulder abduction/adduction   | `q1`-rotated `x`         |
| `q3`  | elbow flexion                  | upper-arm `y`            |

Two links: upper arm (`L1`) and forearm+hand (`L2`).

- **Forward kinematics** (`computeKinematics`, `forwardKinematics`): compose
  the joint rotations from shoulder out to the hand to get the 3-D hand
  position — this is what we track against the target.
- **Jacobians** (`handJacobian`, `linkJacobians`): geometric form
  `J_col_i = z_i × (p − o_i)`. Relates joint velocity/torque to end-effector
  motion/force, and builds the mass matrix.
- **Inverse kinematics** (`inverseKinematics`): damped least-squares Newton
  solve for the joint angles that reach a target position.

## Dynamics (Lagrangian)

Manipulator equation:

```
M(q) q̈ + C(q, q̇) q̇ + G(q) = τ
```

- `M(q)` (`massMatrix`): from link COM Jacobians, `Σ Jvᵀ m Jv + Jwᵀ Ic Jw`.
- `C(q,q̇)` (`coriolisMatrix`): Christoffel symbols from `∂M/∂q`.
- `G(q)` (`gravityVector`): gradient of gravitational potential energy.
- `forwardDynamics` / `inverseDynamics` solve for `q̈` / `τ`.

Actuator **effort** is read from the torque history `τ(t)` (`actuatorEffort`).

## Task difficulty — Fitts' index

```
ID = log2(D / W + 1)      (Shannon form)
```

`D` = reach distance, `W` = target width. Drives both the experiment grid and
the adaptive controller's gain schedule (`adaptivePIDController`).

## Controllers

- **Fixed PID** (`fixedPIDController`) — control group, constant gains.
- **Adaptive PID** (`adaptivePIDController`) — experimental group, gains
  scheduled by task ID (stiffer/more damped for harder reaches).

## Open questions / TODO

- Confirm the joint-axis convention matches the intended shoulder anatomy.
- Validate inertia/mass values against anthropometric tables for the subject.
- Decide the movement-time / dwell protocol for the trials.
- Add human-arm impedance / disturbance if modelling human effort explicitly.
