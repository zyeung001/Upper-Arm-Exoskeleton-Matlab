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

Outputs: `results/sweep_results.mat`, PNGs in `results/figures/`. Figures pop up
when run from the MATLAB desktop and stay hidden under `matlab -batch`.
Requires MATLAB with the Symbolic Math Toolbox (developed on R2026a).

To watch a single carry as an animation (arm + sloshing container + live
slosh/torque plots):

```matlab
animate_carry             % default hard-ish task (f = 0.8, d = 0.4)
animate_carry(1.0, 0.5)   % hardest corner
animate_carry(0.8, 0.4, Speed=0.25, Gif=true)  % slow-mo + save a GIF
```

## How to read the results

The study asks one question: **which controller keeps the human's effort inside a
comfortable band across all tasks?** Every output answers a piece of that.

**Animation (`animate_carry`)** — left: the arm carries the cup from *start* to
*target*; the liquid surface inside the cup tilts with the slosh angle (true
scale). Right, top to bottom: (1) the cup seen side-on, so the tilt is obvious;
(2) the slosh angle over time — the disturbance the carry excites; (3) the
instantaneous torque demand and how the difficulty-adaptive controller splits it
between exo and human; (4) **the payoff plot**: cumulative human effort for all
three controllers against the grey target band — a curve that ends inside the
band means that controller kept the human comfortable on this task. A metrics
table is printed to the console for the task.

**Sweep figures (`main` / `make_figures`)**
1. `fig1` human-effort surfaces — the central result. The fixed controller's
   surface pierces the two grey band planes (below on easy tasks = wasted
   assistance, above on hard tasks = human straining); the difficulty-adaptive
   surface stays between them.
2. `fig2` difference surface — where adaptation relieves the human most
   (negative/blue = relief, concentrated in the hard high-fill long-reach corner).
3. `fig3` exo-effort surfaces — sanity check that the adaptive controller wins
   without the exo just doing everything (alpha is capped below 1).
4. `fig4` coverage bars — the headline number: % of the 100 tasks in band.
5. `fig5` peak-slosh surface — why hard tasks are hard (more slosh).
6. `fig6` status maps — a scorecard of the task space: blue = over-assisted
   (waste), green = in band, red = under-supported (strain).
7. `fig7` alpha maps — what each controller actually commands: fixed is flat,
   fill-only varies with fill level, difficulty-adaptive follows the true
   difficulty gradient.


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
| `make_figures.m` | effort surfaces, difference, coverage, peak slosh, status + alpha maps |
| `main.m` | derive once → sweep → figures |
| `animate_carry.m` | animation of a single carry (demo/debug, optional GIF export) |

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
