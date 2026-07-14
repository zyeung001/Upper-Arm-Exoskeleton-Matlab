# Paper-Writing Context — Slosh-Aware Difficulty-Adaptive Exoskeleton Assistance

Handoff document for the agent writing the research paper. The immediate task is a
**~3-page Methodology/Methods section**. Everything below is verified against the
merged code on `main` (commit 7251884). When in doubt, the code is the source of
truth: all constants live in `params.m`, nothing downstream hard-codes numbers.

## 1. One-paragraph project summary

MATLAB simulation study of a 3-DOF upper-arm exoskeleton assisting a wearer who
carries an open liquid container from a start point to a target in fixed time.
Liquid slosh is modeled as a pendulum-equivalent surrogate (after Bai et al. 2025)
coupled into a full 4-DOF Lagrangian model. Required joint torque is computed by
inverse dynamics along a prescribed quintic reference; an assistance controller
splits it between exoskeleton (fraction alpha) and human (1 − alpha). Three alpha
laws are compared over a 10×10 task grid (fill level × reach distance): fixed,
fill-proportional, and the adopted **closed-form difficulty-adaptive law
alpha = clamp(1 − E\*/D)**. Primary outcome: fraction of tasks where human effort
lands inside a target band. Headline: **98% (difficulty-adaptive) vs 61%
(fill-only) vs 51% (fixed)**, with the tuning done on a disjoint held-out
calibration grid and robustness confirmed across six slosh-surrogate sensitivity
conditions.

## 2. System model (Methods §"System model")

### 2.1 Arm

- 3 actuated DOFs: θ1 shoulder flexion, θ2 elbow flexion, θ3 shoulder rotation
  about the vertical axis. Full coordinate vector q = [θ1, θ2, θ3, φ] where φ is
  the (unactuated) slosh pendulum angle → **4-DOF Lagrangian model**.
- Link parameters (from `params.m`): m1 = 1.0, m2 = 0.7, m3 = 0.7 kg;
  L1 = 0.4, L2 = 0.4, L3 = 0.3 m; lc1 = 0.2, lc2 = 0.2, lc3 = 0.15 m;
  I1 = 0.25, I2 = 0.1, I3 = 0.1 kg·m² (transverse, about COM); g = 9.8 m/s².
- **Provenance**: 2-link values (m1, m2, L1, L2, lc1, lc2, I1, I2) are from
  Zhang et al. 2024. The third link (vertical-axis rotation) is a documented
  *extension* with comparable values — an assumption, not a cited value. Say so.
- Dynamics derived symbolically (`derive_dynamics.m`): kinetic energy T, then
  M = hessian(T, q̇) (valid because T = ½q̇ᵀMq̇), G = jacobian(V, q)ᵀ, Coriolis
  matrix C via **Christoffel symbols** (guarantees Ṁ − 2C skew-symmetry /
  passivity). Exported to numeric functions with `matlabFunction` (gitignored
  `generated/` dir: M_fun, C_fun, G_fun, V_fun, ee_fun, aee_fun).

### 2.2 Slosh surrogate (pendulum-equivalent, after Bai et al. 2025)

- Container: radius Rc = 0.05 m, height Hc = 0.15 m, water ρ = 1000 kg/m³.
- Liquid mass at fill level f ∈ [0,1]: m_liq = ρ·π·Rc²·(f·Hc).
- **Participating (sloshing) mass**: m_s = k_m · m_liq with k_m = 0.5 (tunable
  model constant, swept in sensitivity study).
- Pendulum length L_s = L_s_factor · Rc with L_s_factor = 1.0, matching Bai et
  al.'s ω_n = √(g/R). Natural frequency ω_s = √(g/L_s). Damping ratio ζ_s = 0.05.
- Slosh dynamics (the ONLY forward-integrated state; ode45, forcing supplied via
  pchip `griddedInterpolant` of the reference EE acceleration):
  φ̈ + 2·ζ_s·ω_s·φ̇ + ω_s²·φ = −a_h / L_s,
  where a_h is the horizontal EE acceleration projected onto the reach-radial
  direction: a_h = cos(θ3)·a_ee,x + sin(θ3)·a_ee,y.
- The pendulum is coupled **both ways**: the carry excites φ, and the swinging
  m_s tugs back on the arm through the M/C/G coupling terms of the 4-DOF model.
- **Documented simplification**: only the participating fraction m_s is attached
  to the model; the non-sloshing liquid remainder and the empty container are
  NOT modeled as a rigid hand payload. This is stress-tested by the k_m
  sensitivity sweep (k_m ∈ {0.3, 0.5, 0.7} brackets the missing/extra mass).

## 3. Task battery & trajectory (Methods §"Task set and reference motion")

- Task = (fill level f, reach distance d). Hand path: straight line from start
  P0 = [0.55, 0, −0.15] m (base frame at the shoulder-rotation axis) along unit
  direction u = normalize([0.94, 0.23, 0.25]) for distance d.
- Start pose chosen for a natural elbow-down configuration (~137° elbow flexion
  at start, relaxing to ~50° at farthest target, inside human ROM).
- **Quintic (minimum-jerk) time scaling** s(τ) = 10τ³ − 15τ⁴ + 6τ⁵, τ = t/T:
  zero velocity/acceleration at both ends. Joint-space reference via closed-form
  IK (elbow-down branch; workspace membership asserted; FK∘IK round-trip
  verified < 1e-9).
- **Decision A (deliberate)**: movement time T_move = 1.0 s is FIXED for all
  distances, so longer reaches require higher accelerations and excite more
  slosh. This makes distance a genuine second difficulty axis (if time scaled
  with distance, accelerations would roughly equalize and the task grid would
  collapse to one effective axis). Sample step dt = 0.005 s.

## 4. Required torque, difficulty, and the split (Methods §"Assistance laws")

- **Inverse dynamics along the reference** (no feedback tracking loop — see
  §8 phrasing rules): at each sample, τ = [M(q)q̈ + C(q,q̇)q̇ + G(q)] rows 1–3,
  evaluated at [q_ref; φ] with the simulated slosh state. Output τ_total(t) ∈ R³.
- **Task difficulty** (a-priori, controller-independent):
  D = ∫₀ᵀ Σⱼ |τ_total,j(t)| dt  (units N·m·s). Computed with `trapz`.
- **Torque split**: τ_exo = α·τ_total, τ_human = (1−α)·τ_total, one scalar α per
  task, constant over the carry.
- **Key identity (exact, because α is constant per task)**:
  E_human = ∫ Σ|τ_human| dt = (1 − α)·D.
- **Controllers** (`controllers.m`):
  1. `fixed`: α = 0.5.
  2. `fill`: α = α_min + (α_max − α_min)·f. Genuine adaptive baseline — its α
     spans [0.25, 0.72] over the grid (fairness check, reported).
  3. `difficulty` (ADOPTED): α = 1 − E\*/D with E\* = (E_low + E_high)/2 = 2.80
     N·m·s. This is the **closed-form solution** of (1−α)·D = E\* — "hold human
     effort at the band midpoint." Zero free parameters beyond the shared clamps.
  4. `difficulty-linear` (REJECTED ablation): linear ramp α_min→α_max over
     [D_lo, D_hi]. Matches the closed-form law on the nominal model but collapses
     in sensitivity (see §7).
- All adaptive laws share clamps **α ∈ [α_min, α_max] = [0.2, 0.72]**, applied
  identically. Rationale: α_min > 0 keeps assistance meaningfully on; α_max < 1
  keeps the human engaged and forbids the trivial α = 1 solution. Precedent for
  bounded assistance: Zhang et al.'s Kmin/Kmax stiffness bounds; Bai et al.'s
  retained "perceptible share." The exact values 0.2/0.72 are a-priori design
  constants (no citation) — fixed before results, shared by all controllers,
  and their binding locations are reported (clamp accounting, fig 7).

## 5. Target effort band & calibration protocol (Methods §"Calibration")

- Band on E_human: [E_low, E_high] N·m·s. Below → over-assisted (status −1);
  inside → in band (0); above → under-supported (+1).
- **E_high = 3.4 fixed A PRIORI** in `params.m`, never adjusted to results.
  Interpretation: ~1.1 N·m mean torque per joint sustained over the 1-s carry, a
  small fraction (~2%) of voluntary shoulder/elbow strength. Design constant, not
  a cited physiological value (limitation; see §9).
- **E_low = (1 − α_min)·D_lo = 2.20**, set by FIXED RULE on calibration data
  only: the effort the least-helpful in-bounds controller leaves on the easiest
  calibration task — below it, assistance cannot be the explanation.
- **E\* = band midpoint = 2.80** — therefore fully determined before any
  evaluation data is seen (ceiling a priori, floor by rule on calibration).
- **Train/test split**:
  - Calibration grid: 5×5, f ∈ linspace(0.15, 0.95, 5), d ∈ linspace(0.125,
    0.475, 5) m — deliberately interleaved BETWEEN evaluation grid points;
    disjointness asserted in code (`ismembertol`).
  - Evaluation grid: 10×10, f ∈ [0.1, 1.0], d ∈ [0.1, 0.5] m (100 tasks). All
    reported numbers come from here. Its corners lie OUTSIDE the calibrated
    difficulty range → genuine extrapolation.
  - Tuned-on-calibration quantities, all by fixed rule: D_lo/D_hi = min/max
    calibration difficulty; E_low as above. Nothing else is tuned anywhere.
- One-sentence defense against circularity: "Every tuned quantity was set by
  fixed rules on 25 held-out tasks; coverage is measured on 100 disjoint tasks
  whose extremes exceed the calibrated range."

## 6. Metrics (Methods §"Outcome measures")

- Primary: E_human = ∫ Σⱼ|τ_human| dt per task; **coverage** = fraction of the
  100 evaluation tasks with E_human inside the band.
- Secondary: E_exo (same integral of τ_exo) — guard against the trivial
  full-takeover solution; tau_pk_human = max_t Σⱼ|τ_human| (peak momentary
  demand); peak slosh angle per task; clamp accounting (which tasks are
  law-controlled vs α_min/α_max-limited, tolerance 1e-9).
- Tracking error is deliberately NOT a metric (motion is prescribed; tracking is
  perfect by construction; no tracking claim is made).

## 7. Validation & sensitivity (Methods §"Model validation")

Run automatically at every derivation (`derive_dynamics.m`):
- Reductions to independently derived models: 3-DOF arm (slosh decoupled),
  textbook 2-link planar arm, 1-DOF compound pendulum — all match < 1e-9.
- M symmetry (< 1e-9) and positive definiteness at 5 random configurations.
- Ṁ − 2C skew-symmetry via central finite difference h = 1e-6 (< 1e-5) —
  passivity / Christoffel consistency.
- Free-swing energy conservation of the full 4-DOF model with the exported
  functions (ode45 RelTol 1e-10; relative drift < 1e-5).

Robustness studies (reported in Results, but the *procedure* belongs in Methods):
- **Damping back-reaction bound**: the slosh damper torque neglected in the
  actuated rows is bounded at the worst-case task (f = 1, d = 0.5):
  max ≈ 0.011 N·m vs ≈ 9.6 N·m peak joint torque (≈ 0.1%) → negligible.
- **Surrogate sensitivity**: k_m ∈ {0.3, 0.5, 0.7} × L_s ∈ {0.5·Rc, 1.0·Rc}
  (6 conditions), with the FULL calibrate-then-evaluate procedure re-run per
  condition (not just re-evaluation). Findings: adopted law holds 98% in band at
  ALL six conditions; fixed 34–63%; fill-only 52–61%. L_s barely affects
  effort/difficulty (it shapes slosh angle, not torque scale). At k_m = 0.7 the
  α_max clamp binds on ~5% of tasks (physically sensible shift).
- **Ablation that rejected the linear ramp**: `difficulty-linear` equals the
  closed-form law on the nominal model but collapses to **32% in band at
  k_m = 0.7** — its mid-range effort overshoots the fixed ceiling (~3.79 > 3.4).
  A schedule-shape failure, not a difficulty-signal failure. This is WHY the
  closed-form law was adopted (Decision B, revised).

## 7b. Closed-loop study (`closed_loop/`, mentor-requested) — the ONLY source of tracking claims

A self-contained Simulink study, separate from the sweep and never feeding back
into it. A PD + gravity-compensation controller (Kp = 90, Kd = 14, diagonal)
tracks the same quintic reference on the FULL nonlinear 4-DOF plant; the slosh
starts at rest and is excited by the carry itself (a genuine two-way
disturbance). α is still computed a priori per task by `controllers.m`.

Two human-model elements, both fixed a priori, never tuned:
- **Torque-development lag** τ_h = 100 ms (first-order; neuromuscular activation
  surrogate, cf. Zajac 1989). The exo responds instantly; the human does not.
- **Strength cap** u_h_max = κ·MVC, MVC = [50, 45, 30] N·m for [shoulder flexion,
  elbow flexion, shoulder rotation]. Implemented as saturation on the lag
  integrator's *state* (activation ≤ 1 ⇒ no windup). The exo is uncapped.

**Why the cap matters (the answer to "why don't RMSE/settle/energy differ?").**
All three laws command the same *total* torque; α only decides who supplies it.
With an unlimited human, the task therefore looks identical under every law and
only the effort band separates them — correct, but it leaves the reviewer's
obvious objection ("maybe the human just gets tired, who cares") unanswered.
With a cap, a law that demands more than the user has simply does not get it:
the applied torque falls short, and the task degrades. The binding constraint is
the **static hold** at the target, not the dynamic peak during the carry — an
over-demanded human can never null the error, so the arm droops permanently
(t_settle = NaN). Single task (f = 1.0, d = 0.5, κ = 8 %): fixed sits at its cap
79 % of the run, 2.8 N·m of demanded torque is never delivered, RMSE 14.6 mm and
it never settles; fill and difficulty track at 7.7 mm and settle at 1.16 s.

**Scoring rule (load-bearing).** A task passes only if human effort is **in band
AND the human never hit the cap**. Both halves are required: capping *deflates*
measured effort, so a law can otherwise be scored "in band" *because* the human
failed to deliver what it demanded. Report raw in-band coverage as a disclosed
secondary, never as the headline.

**Critical κ — the tuning-free claim.** Each law has an a-priori *critical κ*:
the weakest user it never over-demands, = max over tasks/joints of
|(1−α)·τ_j| / MVC_j, computed from inverse dynamics on the **calibration grid
only** (`cl_critical_kappa.m`; no simulation, no evaluation data — the train/test
split holds). Result: **fixed 9.1 %, fill 8.5 %, difficulty 6.4 % MVC** — the
difficulty law serves a user ~30 % weaker than fixed can. This is a property of
each law, not a parameter anyone picked. NEVER choose a κ because it makes a law
win; that is why the headline is the κ *sweep* (`run_kappa_sweep`, 5 κ × 9 tasks
× 3 laws), evaluated on the held-out grid, and why κ = 8 % is presented as a
disclosed representative scenario rather than a privileged one.

Closed-loop κ sweep (3×3 evaluation grid at each κ; pass = in band AND within
strength; overloaded = tasks demanding more torque than the user has, of 9):

| κ (% MVC) | fixed pass / overloaded | fill | difficulty |
|---|---|---|---|
| 5.0 | 11 % / 5 | 0 % / 5 | 22 % / 6 |
| 6.5 | 22 % / 4 | 22 % / 3 | 67 % / 2 |
| 8.0 | 33 % / 2 | 33 % / 2 | **89 % / 0** |
| 9.5 | 33 % / 1 | 33 % / 1 | **89 % / 0** |
| 11.0 | 33 % / 0 | 33 % / 0 | **89 % / 0** |

Mean end-effector RMSE falls with κ and converges (~5.6–5.7 mm) once nobody is
over-demanded — i.e. **the accuracy cost is entirely attributable to overload**,
not to the assistance law per se. That is the cleanest form of the claim: the
difficulty law does not track better because it is a better tracker; it tracks
better because it never asks the user for torque they do not have.

Two caveats to state, not hide:
- **The measured zero-overload point sits ~1.5–2 points right of each law's
  predicted critical κ** (difficulty 8 % vs 6.4 %; fixed 11 % vs 9.1 %). Two
  causes, both expected: the evaluation grid extends *beyond* the calibration
  extremes (that is the train/test split working as designed), and closed-loop
  PD demand exceeds the open-loop feedforward the prediction was computed from.
  The *ordering and spacing* of the laws are preserved exactly — which is the
  claim. Present critical κ as a rank-ordering prediction, not a threshold to
  be read off to two decimals.
- **At κ = 5 % (below every law's critical κ) all three laws overload, the
  difficulty law most often (6/9)** — on easy carries its α floor deliberately
  leaves the human more work. The claim is "serves a substantially weaker user,"
  never "serves an arbitrarily weak user."

**Validation cross-check.** A fourth run per task with the lag bypassed and the
cap removed ("ideal human") reproduces the open-loop sweep's (1−α)·D within ~1 %
with identical band verdicts — the closed loop and the sweep agree.

## 8. Nominal results (for context; belongs in Results, not Methods)

- Coverage on the 100 evaluation tasks: **difficulty-adaptive 98%, fill-only
  61%, fixed 51%**.
- The 2% misses are α_min-clamped easiest tasks (E_human = 0.8·D < E_low there);
  explained physically, reported via clamp accounting, not averaged away.
- Fixed fails both corners (over-assists easy, abandons hard); fill-only fixes
  the fill axis but shows a red (under-supported) column at long reach.
- Unclamped tasks land at exactly E\* = 2.80 by construction — the
  difficulty-adaptive effort map is flat gray.
- NOTE: these numbers are from the merged run (`results/sweep_results.mat`). If
  the sweep is regenerated after any parameter change, re-verify before quoting.

## 9. Honest-phrasing rules for the Methods section

1. **Simulation-only study.** No human subjects, no hardware. Never imply
   otherwise.
2. **No tracking claims *from the sweep*.** The 10×10 sweep deliberately has no
   feedback loop; the arm follows the reference by construction, so every claim
   drawn from it is about effort distribution. Tracking results come *only* from
   the closed-loop Simulink study (§7b), and must be attributed to it explicitly.
   Never let a closed-loop RMSE migrate into a sentence about the sweep.
3. **Band values are design constants.** The band *concept* has precedent
   (Zhang: bounded stiffness + effort term; Bai: perceptible retained share);
   the numbers 2.2/3.4 do not. Defense pattern (reuse for clamps too): fixed
   before evaluation → applied identically to all controllers → effect reported,
   not hidden. Physiological validation (EMG, perceived exertion, metabolic
   cost) is future work — worth one limitation sentence.
4. **Third arm link and k_m are assumptions**; both are documented as such and
   k_m is swept. Say "extension"/"model constant," not "from the literature."
5. **Decision A (fixed T_move) and Decision B (closed-form law; linear ramp as
   rejected ablation) are deliberate** — present them as design decisions with
   rationale, not incidental choices.
6. **The adopted law has no free parameters beyond the shared clamps.** This is
   a selling point; state it precisely (E\* is derived from the band, which is
   fixed pre-evaluation).
7. Units: D and E_human/E_exo/E\*/band in N·m·s; torques in N·m; write alpha as
   α consistently; "in band / over-assisted / under-supported" is the canonical
   status terminology.
8. **MVC and κ are modelling assumptions, and κ is a *scenario*, not a result.**
   The MVC vector is order-of-magnitude, not measured; say so. Present the
   strength cap as "what if the wearer is weak," never as a calibrated human
   model. The defensible quantity is each law's *critical κ* (a property of the
   law, computed a priori on calibration data); the defensible presentation is
   the κ sweep. Quoting a single κ that happens to separate the laws, without
   the sweep beside it, would be results-tuning — do not do it. Note explicitly
   that at κ = 5 % the adopted law also over-demands the user: the claim is
   "serves a substantially weaker user," not "serves any user."

## 10. Citations (the only two external sources)

- **Zhang et al. 2024**, Front. Bioeng. Biotechnol. 11:1332689 — arm link
  parameters (2-link values), quintic planning precedent, bounded-assistance
  (Kmin/Kmax) precedent.
- **Bai et al. 2025**, Biomimetics 10(12):815 — pendulum-equivalent slosh
  surrogate (ω_n = √(g/R)), bounded load-sharing split, "perceptible share"
  precedent. Closest prior work. **Differentiation** (worth a sentence in
  Methods or Intro): this project schedules α a priori from task difficulty
  (not reactively from a force sensor), targets an effort band for a healthy
  user (not therapeutic slosh retention), and characterizes a 2D performance
  surface over the task space with a worn multi-DOF exoskeleton model.

## 11. Repo map (where to verify anything)

| File | What it holds |
|---|---|
| `params.m` | ALL constants + inline rationale comments (band, clamps, grids) |
| `derive_dynamics.m` | symbolic 4-DOF Lagrangian + all dynamics validation |
| `pack_pvec.m` | 15-element parameter-vector contract for generated functions |
| `make_trajectory.m` | IK (elbow-down) + quintic reference |
| `compute_required_torque.m` | slosh ode45 forward sim + inverse dynamics |
| `compute_difficulty.m` | D = trapz(t, Σ\|τ\|) |
| `controllers.m` | four α laws + torque split (incl. rejected ablation) |
| `calibrate_controller.m` | held-out calibration, disjointness assert, cache |
| `compute_metrics.m` | E_human, E_exo, peak, band status |
| `run_sweep.m` | 10×10 × controllers sweep + clamp accounting + coverage |
| `make_figures.m` | 7 figures (band-diverging effort maps, status bars, α maps, law plot) |
| `animate_carry.m` | single-carry animation demo (not part of the evaluation) |
| `main.m` | pipeline: derive (once) → calibrate → sweep → figures |
| `README.md` | prose overview incl. "How to read the results" per figure |
| `closed_loop/cl_params.m` | closed-loop constants: PD gains, τ_h, MVC, κ, κ sweep |
| `closed_loop/build_closed_loop_model.m` | source of truth for the gitignored `.slx` |
| `closed_loop/cl_critical_kappa.m` | **critical κ per law** (a priori, calibration grid) |
| `closed_loop/cl_overload.m` | strength-cap saturation detection (whole run, time-weighted) |
| `closed_loop/run_closed_loop.m` | single-task deep dive: 3 laws + ideal-human cross-check |
| `closed_loop/run_closed_loop_grid.m` | 3×3 grid × 3 laws at one κ (pass = in band AND within strength) |
| `closed_loop/run_kappa_sweep.m` | **the closed-loop headline**: grid × 5 κ |

Run `main` in MATLAB (Symbolic Math Toolbox required, R2026a) to regenerate
`results/sweep_results.mat` and `results/figures/*.png`.

## 12. Suggested 3-page Methods skeleton

1. **System model** (~0.75 p): 3-DOF arm + parameters/provenance; slosh
   pendulum surrogate + coupling; 4-DOF Lagrangian derivation with validation
   summary (reductions, passivity, energy) in 2–3 sentences.
2. **Task battery and reference motion** (~0.5 p): (f, d) grid definition,
   start pose, quintic profile, Decision A with rationale.
3. **Required torque and task difficulty** (~0.4 p): inverse dynamics along the
   reference (state the no-tracking-loop design explicitly), definition of D.
4. **Assistance laws** (~0.6 p): split equation, E_human = (1−α)D identity,
   the three controllers + closed-form derivation of the adopted law, clamps
   with rationale, linear ramp introduced as ablation baseline.
5. **Calibration and evaluation protocol** (~0.4 p): band definition (ceiling a
   priori, floor by rule), disjoint 5×5/10×10 split, fixed tuning rules,
   extrapolating corners.
6. **Outcome measures and robustness procedures** (~0.35 p): coverage + secondary
   metrics; sensitivity design (6 conditions, full re-calibration per condition);
   damping back-reaction bound method.

Existing drafting aids: a ~230-word abstract draft and a 175-word cut were
already written in conversation (2026-07-12); a line-by-line code study guide
lives at https://claude.ai/code/artifact/17f8694b-784f-4e62-8d53-ed42c36e7eb0
(private artifact, not in the repo — includes a Defense Q&A anticipating
reviewer attacks).
