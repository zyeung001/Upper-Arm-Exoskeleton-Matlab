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
run_closed_loop         % single-task deep dive: 4 sims (3 laws + ideal
                        % human), metrics table, controller + validation figs
run_closed_loop(0.8, 0.4, Rebuild=true)   % force model rebuild
run_closed_loop_grid    % 5x5 task grid x 3 laws (75 sims) ->
                        % coverage + accuracy figure at the default strength
run_closed_loop_grid(Kappa=0.095)         % same grid, other user strength
run_kappa_sweep         % THE closed-loop headline: the grid repeated at 5
                        % user strengths (375 sims, ~1 h) -> pass rate,
                        % overload and RMSE vs how weak the user is
animate_closed_loop     % animate the run in results/closed_loop_results.mat
```

From a shell: `matlab -batch "main"` (figures are created hidden under `-batch`).
Requires the Symbolic Math Toolbox (developed on R2026a). Outputs go to
`results/*.mat` (`sweep_results`, `closed_loop_results`, `closed_loop_grid`,
`closed_loop_kappa`) and `results/figures/*.png` (all gitignored).

There is no separate test runner. The validation suite (model reductions to
independently derived 3/2/1-DOF models, mass-matrix symmetry/PD, Ṁ−2C
skew-symmetry, free-swing energy conservation) runs automatically inside
`derive_dynamics` at every derivation and fails via `assert`/`error`.

Three caches to know about:
- `generated/` — numeric functions exported from the symbolic derivation
  (M_fun, C_fun, G_fun, V_fun, ee_fun, aee_fun). `main` skips derivation if it
  exists. **After editing `derive_dynamics.m`, delete `generated/`** (or call
  `derive_dynamics()` directly) or you'll run stale dynamics.
- `results/calibration.mat` — the 25-task calibration sweep, keyed on every
  physics/grid/bound parameter and auto-invalidated when any of them changes
  (`calibrate_controller.m`). No manual action needed after editing `params.m`.
- `closed_loop/arm_closed_loop.slx` — build artifact of
  `build_closed_loop_model.m`. Auto-rebuilt by the runners when the builder or
  `cl_params.m` is newer (`cl_needs_rebuild.m` — note `exist(f,'file')` returns
  4, not 2, for a `.slx`; use `isfile`). `Rebuild=true` forces it.

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
The fill→surrogate rules (m_liq → m_s, L_s, w_s, b_s, pvec) live in one place,
`slosh_surrogate.m`, shared by the sweep and `closed_loop/` so the two studies
cannot drift apart on the physics.

`closed_loop/` (mentor-requested, 2026-07-13) is a SEPARATE closed-loop
Simulink study: a PD + gravity-compensation controller tracks the same
quintic reference on the full nonlinear 4-DOF plant, with the slosh starting
at rest and excited by the carry itself (a true two-way disturbance). The
human delivers their (1−α) share through a first-order torque-development
lag (`cl_params.tau_h` = 100 ms, fixed a priori from the neuromuscular
literature, never tuned) while the exo responds instantly — this makes the
α law dynamically consequential. The COMPARISON AXIS is the three alpha
laws, one simulation each, scored on the whiteboard metrics (RMSE, settle
time, energy of u) plus human effort vs the band; a fourth ideal-human run
(lag bypassed, uncapped) is the validation cross-check, matching the sweep's
(1−α)·D within ~1% at the deep-dive task; `cl_validate_surface.m` repeats the
check across the four grid corners, the center and the default (worst case
1.2%, law-independent — the split cancels, so it is plant fidelity not the α
law), with identical band verdicts.

The human also has a **strength cap** (`cl_params.u_h_max` = κ·MVC, a
saturation on the lag integrator's *state* — activation cannot exceed 1, so
there is no windup; the exo is uncapped). This is what makes the α law
change the *task* and not just who gets tired: a law that demands more
torque than the user has simply does not get it, the applied total falls
short, and tracking degrades. The binding constraint is usually the **static
hold** at the target, not the dynamic peak during the carry — an overloaded
human can never null the error, so the arm droops permanently (`t_settle` =
NaN). This is where the mentor's accuracy metric finally discriminates.
Scoring rule: a task PASSES only if human effort is **in band AND the human
never hit the cap** — capping *deflates* measured effort, so in-band alone
could be "passed" by overloading the user.

κ is a *scenario* (who wears the exo), never tuned to results. Each law has
an a-priori **critical κ** — the weakest user it never over-demands, from
inverse dynamics on the **calibration grid only** (`cl_critical_kappa.m`,
which computes it; don't quote stale numbers after changing physics): fixed
9.1 %, fill 8.5 %, difficulty 6.4 % MVC. So the difficulty law serves a
~30 % weaker user than fixed can. `run_kappa_sweep` (the headline) tests
that prediction on the held-out evaluation grid across five κ, so no single
κ is privileged; `run_closed_loop_grid` is one κ slice of it. The closed-loop
grid is **5×5, interleaved between the calibration points and asserted
disjoint** — it is evaluation data, and critical κ is trained on calibration
data, so an overlapping task would test the prediction on its own training
set. Measured zero-overload points land ~1.5–2.5 points *right* of the
predicted critical κ (eval grid exceeds calibration extremes; closed-loop PD
demand exceeds open-loop feedforward). The difficulty law's ~2-point margin
below the other two survives the shift; the fixed/fill pair (predicted only
0.6 points apart) is NOT resolved at the sweep's 1.5-point κ resolution
(tied/mildly inverted) — critical κ is a rank-ordering prediction for
well-separated laws, not a threshold (caveats in `PAPER_CONTEXT.md` §7b).
**Below its critical κ the difficulty law is the WORST of the three** (at
κ = 5 %: 17 of 25 tasks over-demanded, vs 12 for fixed) — it targets constant
human effort on every task, so it never lets a very weak user off easy the way
a fixed α = 0.5 share does on an easy carry. The claim has a floor, not just a
direction; never phrase it as "good for weak users". Overload detection lives in `cl_overload.m` and is assessed over the
**whole run** — a carry-window-only check misses the hold-phase saturation
that causes the droop. Single-task runs only separate the laws at grid
extremes — mid-difficulty tasks coincide by construction.
`build_closed_loop_model.m` is the reviewable source of truth for the
gitignored `arm_closed_loop.slx`; constants live in `cl_params.m`; α is
still computed A PRIORI per task via `controllers.m`.

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
  pre-evaluation. In `closed_loop/`, `tau_h`, `MVC` and the strength fraction
  κ are the same kind of quantity: κ describes the *user*, not the controller.
  **Never pick a κ because it makes a law win** — that is why the headline is
  `run_kappa_sweep` (a curve over five κ) and why the κ range is derived from
  calibration-grid demands only. If you need a single number to quote, quote a
  law's *critical κ*, which is a property of the law, not a choice.
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

## Figure conventions (`make_figures.m`, `cl_figures.m`, grid/sweep figures)

Every sweep figure is a 2D fill × distance map (`imagesc` + `axis xy`) or a bar
chart — no 3D surfaces (they were tried and replaced as unreadable). Human-effort
maps use the custom band-diverging colormap: gray = inside the target band, blues
= below (over-assisted), reds = above (under-supported), with color limits
symmetric about the band center. The shared `new_fig.m` (root) keeps figures
hidden under `-batch` and forces a light theme; figures are saved with
`exportgraphics` at 200 dpi. `cl_grid_figure.m` / `cl_kappa_figure.m` are
standalone so the grid and κ-sweep figures can be regenerated from the saved
`results/*.mat` without re-running the simulations. Titles that state the
evidence base (grid size, κ) are built with `sprintf` from the result struct,
never hardcoded — a hardcoded "3x3" once shipped over 5×5 data.

Closed-loop figures: per-law Okabe-Ito colors (fixed blue `[0 114 178]`, fill
orange `[230 159 0]`, difficulty green `[0 158 115]` /255), band shading via
`yregion` (a `patch` before a categorical `bar` breaks the axis). Honesty
markers are part of the convention: an **×** = human at the strength cap (its
measured-effort point is deflated and not band-comparable), open **▽** = the
effort that was demanded; never let a capped bar/point read as a clean in-band
result. Don't hardcode interpretive claims in titles — the strength cap can
falsify them at other κ (a "never settles" bar is drawn full-height + labeled,
normalized to the ideal-human run, since fixed's `t_settle` can be NaN).
