# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

MATLAB simulation study (no toolchain beyond MATLAB itself): task-difficulty-aware
adaptive assistance for a 3-DOF upper-arm exoskeleton carrying a sloshing liquid
container. Three assistance laws are compared on a 10×10 task grid (fill level ×
reach distance); the primary metric is the fraction of tasks where human effort
lands inside a target band. See `README.md` for the study design and how to read
each figure, and `PAPER_CONTEXT.md` for the research-paper context (exact numbers,
citations, phrasing rules). `paper/` holds paper drafts.

## Commands

```matlab
main                    % full pipeline: derive dynamics (once) -> calibrate ->
                        % 10x10 sweep -> save figures
animate_carry           % single-carry animation, default task (f=0.8, d=0.4)
animate_carry(1.0, 0.5, Speed=0.25, Gif=true)
derive_dynamics()       % force symbolic re-derivation + validation suite
% Closed-loop Simulink validation (separate from the sweep; needs Simulink):
addpath('closed_loop')
run_closed_loop         % build .slx if missing -> simulate -> compare the
                        % three alpha laws on measured torque + figures
run_closed_loop(0.8, 0.4, Rebuild=true)   % force model rebuild
animate_closed_loop     % animate the run in results/closed_loop_results.mat
```

From a shell: `matlab -batch "main"` (figures are created hidden under `-batch`).
Requires the Symbolic Math Toolbox (developed on R2026a). Outputs go to
`results/sweep_results.mat` and `results/figures/*.png` (both gitignored).

There is no separate test runner. The validation suite (model reductions to
independently derived 3/2/1-DOF models, mass-matrix symmetry/PD, Ṁ−2C
skew-symmetry, free-swing energy conservation) runs automatically inside
`derive_dynamics` at every derivation and fails via `assert`/`error`.

Two caches to know about:
- `generated/` — numeric functions exported from the symbolic derivation
  (M_fun, C_fun, G_fun, V_fun, ee_fun, aee_fun). `main` skips derivation if it
  exists. **After editing `derive_dynamics.m`, delete `generated/`** (or call
  `derive_dynamics()` directly) or you'll run stale dynamics.
- `results/calibration.mat` — the 25-task calibration sweep, keyed on every
  physics/grid/bound parameter and auto-invalidated when any of them changes
  (`calibrate_controller.m`). No manual action needed after editing `params.m`.

## Architecture

Pipeline (`main.m`): `params` → `calibrate_controller` → `run_sweep` → `make_figures`.

Data flow for one task (f, d), all inside `run_sweep`'s triple loop:
`make_trajectory` (closed-form elbow-**down** IK + quintic reference, trajectory
reused across fills for a given d) → `compute_required_torque` (forward-simulate
only the slosh pendulum with ode45, then inverse dynamics along the reference
using the generated 4-DOF functions) → `compute_difficulty` (scalar D = ∫Σ|τ|dt)
→ `controllers` (pick α, split τ into exo/human shares) → `compute_metrics`
(effort integrals + band status).

The model is 4-DOF: q = [θ1 θ2 θ3 φ] where φ is the slosh pendulum angle — the
liquid is a pendulum-equivalent surrogate (Bai et al. 2025) coupled both ways
through M/C/G. `pack_pvec.m` defines the 15-element parameter-vector contract
between `params` and the generated functions; if you change the symbolic
parameter list in `derive_dynamics.m`, you must change `pack_pvec.m` to match.

`closed_loop/` (mentor-requested, 2026-07-13) is a SEPARATE closed-loop
Simulink validation: a PD + gravity-compensation controller tracks the same
quintic reference on the full nonlinear 4-DOF plant, with the slosh starting
at rest and excited by the carry itself (a true two-way disturbance). The
COMPARISON AXIS is the three alpha laws scored on the measured closed-loop
torque (cumulative/final human effort vs the band, momentary human demand);
tracking metrics (RMSE, settle, energy of u) are law-independent context
since all laws split the same total. `build_closed_loop_model.m` is the
reviewable source of truth for the gitignored `arm_closed_loop.slx`;
constants live in `cl_params.m`; α is still computed A PRIORI per task via
`controllers.m`. Cross-check: closed-loop E_human matches the sweep's
(1−α)·D within ~1% with identical band verdicts.

## Design invariants — do not casually break these

These are load-bearing for the study's validity (the paper's claims depend on
them); each is documented in code comments where it lives:

- **All constants live in `params.m`.** Nothing downstream hard-codes numbers.
- **No feedback tracking loop in the sweep pipeline, deliberately.** Torque
  comes from inverse dynamics along the prescribed reference; tracking is
  perfect by construction and no tracking claim is made. Do not add a
  PID/tracking controller to "improve" it. The one sanctioned exception is
  `closed_loop/` — a self-contained Simulink validation that never feeds
  back into the sweep, calibration, or reported grid numbers.
- **Train/test split.** The 5×5 calibration grid is interleaved between the
  10×10 evaluation grid points and asserted disjoint (`calibrate_controller`).
  All tuning (D_lo, D_hi, E_low) happens by fixed rules on calibration data
  only; every reported number comes from the evaluation grid.
- **A-priori quantities stay a priori.** `p.band.E_high = 3.4` and the clamps
  `alpha ∈ [0.2, 0.72]` are fixed before any results and never adjusted to fit
  them. E* (the adopted law's target) is the band midpoint, hence also fixed
  pre-evaluation.
- **NaN sentinels.** `D_lo/D_hi/E_low` are NaN until `calibrate_controller`
  fills them; `run_sweep` asserts they are set. Keep that failure mode.
- **`'difficulty'` is the adopted controller; `'difficulty-linear'` is a
  rejected ablation baseline** kept for comparison — don't remove it or promote
  it.
- **Elbow-down IK branch** (`th2 = +acos(...)` in `make_trajectory`) was a
  deliberate fix for an unnatural elbow-up pose; the start point in `params.m`
  was chosen to keep the whole grid in a natural posture and inside the
  workspace (asserted).
- **Fixed movement time** (`T_move = 1.0` for all distances) is Decision A:
  longer reaches are deliberately faster so distance is a real difficulty axis.

## Figure conventions (`make_figures.m`)

Every sweep figure is a 2D fill × distance map (`imagesc` + `axis xy`) or a bar
chart — no 3D surfaces (they were tried and replaced as unreadable). Human-effort
maps use the custom band-diverging colormap: gray = inside the target band, blues
= below (over-assisted), reds = above (under-supported), with color limits
symmetric about the band center. `new_fig` keeps figures hidden under `-batch`
and forces a light theme; figures are saved with `exportgraphics` at 200 dpi.
