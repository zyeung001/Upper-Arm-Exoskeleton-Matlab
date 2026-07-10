# Task-Difficulty-Aware Adaptive Assistance for Upper-Limb Exoskeletons Carrying Sloshing Liquids

MATLAB simulation of a person wearing a 3-DOF arm exoskeleton carrying a container
of liquid from a start position to a target. The liquid sloshes (pendulum-equivalent
surrogate, after Bai et al. 2025), adding a motion-dependent disturbance. The total
required torque is split between exoskeleton and human: the exo supplies a fraction
`alpha`, the human the remainder.

Three controllers are compared over a 10x10 task grid (fill level x reach distance):

1. **Fixed** — constant `alpha`.
2. **Fill-only adaptive** — `alpha` scales with fill level.
3. **Difficulty-adaptive** — `alpha = clamp(1 - E*/D)`, the **closed-form
   solution** for holding human effort at a target `E*`: since
   `E_human = (1 - alpha) * D`, setting it equal to `E*` and solving gives the
   law directly. `D` is the a-priori task difficulty (integrated required
   torque, driven by both fill and distance) and `E*` is the target-band
   midpoint, fixed before any evaluation data is seen. Beyond the shared
   clamps the law has **no free parameters**. (A linear `D`-ramp was evaluated
   as an ablation baseline and rejected — see Validation & robustness.)

The primary metric is **human effort** against a target effort band
`[E_low, E_high]`: below = over-assisted (wasteful), above = under-supported.
Exo effort is reported as a guard against the trivial `alpha = 1` solution
(`alpha` is bounded inside `(0, 1)` by design).

## Architecture

The arm follows a prescribed quintic reference; required torque comes from
**inverse dynamics along the reference** (full 4-DOF model: 3 links + slosh
pendulum). The **only forward-integrated state is the slosh pendulum**, forced by
the end-effector acceleration. There is deliberately **no feedback tracking loop**
(see spec §0), and tracking RMSE is deliberately not a metric — because tracking
is perfect by construction, no tracking benefit is (or can be) claimed; every
claim is about effort distribution.

## Calibration / evaluation split

The difficulty ramp (`D_lo`, `D_hi`) and the band floor `E_low` are tuned by
**fixed rules on a 5x5 calibration grid that is disjoint from the evaluation
grid** (`calibrate_controller.m`, disjointness asserted): `D_lo`/`D_hi` = the
min/max difficulty observed during calibration, and
`E_low = (1 - alpha_min) * D_lo` — the lowest effort any in-bounds controller
can leave on the easiest calibration task. `E_high` is fixed **a priori** in
`params.m` (~1.1 N m mean torque per joint over the carry) and never adjusted
to fit results. Coverage is then reported on the 10x10 evaluation grid, whose
corners lie **outside** the calibrated range — so the headline number is
measured on tasks the tuning never saw.

## Run

```matlab
main   % derives dynamics once (Symbolic Math Toolbox), calibrates on the
       % held-out grid, runs the evaluation sweep, saves figures
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

**Sweep figures (`main` / `make_figures`)** — every map is a 2D fill x distance
heatmap of the 10x10 task grid (one simulated carry per cell):
1. `fig1` human-effort maps — the central result. Color diverges about the
   target band: gray = in band, blue = over-assisted (waste), red =
   under-supported (strain). Fixed fails in both corners, fill-only fixes the
   fill axis but is blind to distance (red column at long reach), the
   difficulty-adaptive map is entirely gray.
2. `fig2` difference map — E_human, difficulty-adaptive minus fixed
   (blue = relief, concentrated in the hard high-fill long-reach corner).
3. `fig3` exo-effort maps — sanity check that the adaptive controller wins
   without the exo just doing everything (alpha is capped below 1).
4. `fig4` band-status bars — the headline: stacked share of the 100 tasks each
   controller leaves over-assisted / in band / under-supported.
5. `fig5` peak-slosh map — why hard tasks are hard (more slosh at long reach).
6. `fig6` alpha maps — what each controller actually commands: fixed is flat,
   fill-only varies with fill level only, difficulty-adaptive follows the true
   difficulty gradient.
7. `fig7` the assistance law — `alpha(D) = 1 - E*/D` against the rejected
   linear ramp, with the clamps and every evaluation task marked as
   law-controlled (effort held at `E*` exactly) or clamp-limited. The residual
   in-band misses are the `alpha_min`-clamped easiest tasks, explained
   physically rather than averaged away.


## Files

| File | Purpose |
|---|---|
| `params.m` | all a-priori constants (nothing downstream hard-codes numbers) |
| `calibrate_controller.m` | rule-based tuning of `D_lo`/`D_hi`/`E_low` on the held-out calibration grid |
| `derive_dynamics.m` | symbolic 4-DOF Lagrangian → `generated/{M,C,G,V,ee,aee}_fun` + validation: 3/2/1-DOF reductions, M sym/PD, `Mdot-2C` skew, free-swing energy conservation |
| `pack_pvec.m` | packs the parameter vector consumed by the generated functions |
| `make_trajectory.m` | IK endpoints + quintic joint-space reference for distance `d` |
| `compute_required_torque.m` | slosh forward-sim (`ode45`) + inverse dynamics → `tau_total`, `peak_slosh` |
| `compute_difficulty.m` | scalar difficulty `D` from `tau_total` |
| `controllers.m` | three `alpha` laws + torque split |
| `compute_metrics.m` | `E_human`, `E_exo`, in-band status |
| `run_sweep.m` | fill x distance sweep, all controllers |
| `make_figures.m` | band-diverging effort maps, difference, status bars, peak slosh + alpha maps |
| `main.m` | derive once → sweep → figures |
| `animate_carry.m` | animation of a single carry (demo/debug, optional GIF export) |

## Modeling decisions

- **Decision A:** movement time `T_move` is fixed for all reach distances, so longer
  reaches are faster and excite more slosh (intended difficulty coupling).
- **Decision B (revised):** the difficulty→`alpha` schedule is the closed-form
  target-holding law `alpha = clamp(1 - E*/D)` (only the clamps `alpha_min`,
  `alpha_max` are parameters). The originally specified linear clamped ramp is
  retained in `controllers.m` as the rejected ablation baseline
  (`'difficulty-linear'`): it matches the closed-form law on the nominal model
  but collapses when the difficulty range widens (see Validation & robustness).

## Validation & robustness

- **Dynamics checks** (`derive_dynamics`, run at every derivation): reductions to
  independently derived 3-DOF / 2-DOF planar / 1-DOF models; M symmetry and
  positive definiteness at random configurations; `Mdot - 2C` skew-symmetry
  (passivity/Christoffel consistency); free-swing energy conservation of the
  full 4-DOF model with the exported functions.
- **Slosh damping back-reaction** (neglected in the actuated rows): bounded at
  the worst-case task (f = 1, d = 0.5) by the pendulum damper torque —
  max ~0.011 N m vs ~9.6 N m peak joint torque (~0.1%), i.e. negligible.
- **Surrogate sensitivity** (`k_m` in {0.3, 0.5, 0.7} x `L_s` in {0.5Rc, Rc},
  full calibrate-then-evaluate procedure re-run per condition, adopted law):
  `L_s` barely affects effort/difficulty at all (it shapes the slosh angle,
  not the torque scale). The adopted closed-form law holds **98% in band at
  all six conditions** (fixed: 34-63%, fill-only: 52-61%), with the clamp
  budget shifting physically (at `k_m = 0.7` the `alpha_max` clamp starts
  binding on 5% of tasks). This sensitivity study is what rejected the
  originally specified linear ramp: equal to the closed-form law on the
  nominal model, it collapses to 32% at `k_m = 0.7` because its mid-range
  effort overshoots the fixed ceiling (~3.79 > 3.4) — a schedule-shape
  failure, not a difficulty-signal failure.
- **Fair comparison check**: fill-only's alpha genuinely varies over the grid
  ([0.25, 0.72]), so it is a real adaptive baseline, not a second fixed one.

## Grounding

- Arm link parameters (2-link values) and quintic planning: Zhang et al. 2024,
  Front. Bioeng. Biotechnol. 11:1332689 (third link is a documented extension).
- Slosh pendulum surrogate and bounded load-sharing split: Bai et al. 2025,
  Biomimetics 10(12):815 — the closest prior work. This project differs in that
  `alpha` is scheduled **a-priori from task difficulty** (not reactively from a
  force sensor), targets an **effort band** for a healthy user (not therapeutic
  slosh retention), and characterizes a **2D performance surface** over the task
  space with a worn multi-DOF exoskeleton model.
