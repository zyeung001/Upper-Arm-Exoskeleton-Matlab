# Task-Difficulty-Aware Adaptive Assistance for Upper-Limb Exoskeletons Carrying Sloshing Liquids

MATLAB simulation of a person wearing a 3-DOF arm exoskeleton carrying a container
of liquid from a start position to a target. The liquid sloshes (pendulum-equivalent
surrogate, after Bai et al. 2025), adding a motion-dependent disturbance. The total
required torque is split between exoskeleton and human: the exo supplies a fraction
`alpha`, the human the remainder.

Three controllers are compared over a 10x10 task grid (fill level x reach distance):

1. **Fixed** — constant `alpha`.
2. **Fill-only adaptive** — `alpha` scales with fill level.
3. **Difficulty-adaptive** — `alpha` scales with an a-priori task-difficulty metric
   `D` (integrated required torque), driven by both fill and distance.

The primary metric is **human effort** against a target effort band
`[E_low, E_high]`: below = over-assisted (wasteful), above = under-supported.
Exo effort is reported as a guard against the trivial `alpha = 1` solution
(`alpha` is bounded inside `(0, 1)` by design).

## Architecture

The arm follows a prescribed quintic reference; required torque comes from
**inverse dynamics along the reference** (full 4-DOF model: 3 links + slosh
pendulum). The **only forward-integrated state is the slosh pendulum**, forced by
the end-effector acceleration. There is deliberately **no feedback tracking loop**
(see spec §0), and tracking RMSE is deliberately not a metric.

## Run

```matlab
main   % derives dynamics once (Symbolic Math Toolbox), runs sweep, saves figures
```

Outputs: `results/sweep_results.mat`, PNGs in `results/figures/`.
Requires MATLAB with the Symbolic Math Toolbox (developed on R2026a).

## Files

| File | Purpose |
|---|---|
| `params.m` | all constants (nothing downstream hard-codes numbers) |
| `derive_dynamics.m` | symbolic 4-DOF Lagrangian → `generated/{M,C,G,ee,aee}_fun` + reduction validation (3/2/1-DOF) |
| `pack_pvec.m` | packs the parameter vector consumed by the generated functions |
| `make_trajectory.m` | IK endpoints + quintic joint-space reference for distance `d` |
| `compute_required_torque.m` | slosh forward-sim (`ode45`) + inverse dynamics → `tau_total`, `peak_slosh` |
| `compute_difficulty.m` | scalar difficulty `D` from `tau_total` |
| `controllers.m` | three `alpha` laws + torque split |
| `compute_metrics.m` | `E_human`, `E_exo`, in-band status |
| `run_sweep.m` | fill x distance sweep, all controllers |
| `make_figures.m` | effort surfaces, difference surface, coverage, peak slosh |
| `main.m` | derive once → sweep → figures |

## Modeling decisions

- **Decision A:** movement time `T_move` is fixed for all reach distances, so longer
  reaches are faster and excite more slosh (intended difficulty coupling).
- **Decision B:** the difficulty→`alpha` schedule is a parameterized clamped ramp
  (`alpha_min`, `alpha_max`, `D_lo`, `D_hi` in `params.m`), tuned so the
  difficulty-adaptive controller keeps human effort in the target band.

## Grounding

- Arm link parameters (2-link values) and quintic planning: Zhang et al. 2024,
  Front. Bioeng. Biotechnol. 11:1332689 (third link is a documented extension).
- Slosh pendulum surrogate and bounded load-sharing split: Bai et al. 2025,
  Biomimetics 10(12):815 — the closest prior work. This project differs in that
  `alpha` is scheduled **a-priori from task difficulty** (not reactively from a
  force sensor), targets an **effort band** for a healthy user (not therapeutic
  slosh retention), and characterizes a **2D performance surface** over the task
  space with a worn multi-DOF exoskeleton model.
