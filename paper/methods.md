# Methods

This is a simulation-only study: no human subjects or physical hardware were involved, and every quantity reported below is computed from a mathematical model implemented in MATLAB (R2026a, Symbolic Math Toolbox). The methods comprise (A) a coupled arm–slosh dynamic model, (B) a parameterized battery of carrying tasks with a prescribed reference motion, (C) an inverse-dynamics definition of required torque and task difficulty, (D) a family of assistance laws that split the required torque between exoskeleton and wearer, (E) a held-out calibration and evaluation protocol, and (F) outcome measures and robustness procedures.

## A. System model

### 1) Arm and exoskeleton kinematics

The wearer's arm and the worn exoskeleton are modeled as a single 3-degree-of-freedom (DOF) serial chain with actuated joint angles θ₁ (shoulder flexion), θ₂ (elbow flexion), and θ₃ (shoulder rotation about the vertical axis). Together with the unactuated slosh coordinate φ introduced below, the full configuration vector is q = [θ₁, θ₂, θ₃, φ]ᵀ, yielding a 4-DOF Lagrangian model.

Link parameters are m₁ = 1.0 kg, m₂ = 0.7 kg, m₃ = 0.7 kg; L₁ = 0.4 m, L₂ = 0.4 m, L₃ = 0.3 m; center-of-mass offsets lc₁ = 0.2 m, lc₂ = 0.2 m, lc₃ = 0.15 m; and transverse moments of inertia about each center of mass I₁ = 0.25 kg·m², I₂ = 0.1 kg·m², I₃ = 0.1 kg·m², with g = 9.8 m/s². The two-link (upper-arm/forearm) values are taken from Zhang et al. [1]. The third link, which adds shoulder rotation about the vertical axis, is a modeling *extension* introduced in this work with values comparable to the cited links; it is an assumption of this study rather than a literature value.

The equations of motion were derived symbolically. The kinetic energy T is quadratic in the generalized velocities, so the inertia matrix follows as M(q) = ∂²T/∂q̇², the gravity vector as G(q) = (∂V/∂q)ᵀ, and the Coriolis/centrifugal matrix C(q, q̇) via Christoffel symbols of the first kind, which guarantees the skew-symmetry of Ṁ − 2C (passivity). The symbolic expressions were exported to numeric functions for all downstream computation.

### 2) Slosh surrogate

The carried liquid is represented by a pendulum-equivalent surrogate following Bai et al. [2]. The container is an open cylinder of radius R_c = 0.05 m and height H_c = 0.15 m filled with water (ρ = 1000 kg/m³) to a fill level f ∈ [0, 1], giving a liquid mass m_liq = ρ·π·R_c²·(f·H_c). Only a participating fraction of this mass sloshes; it is modeled as a point mass m_s = k_m·m_liq on a pendulum of length L_s attached at the hand, with k_m = 0.5 a tunable model constant (swept in the sensitivity study, Sec. F). The pendulum length is L_s = R_c, matching the first-mode natural frequency ω_n = √(g/R) used by Bai et al. [2], so ω_s = √(g/L_s); the damping ratio is ζ_s = 0.05. The pendulum angle φ obeys

  φ̈ + 2ζ_s ω_s φ̇ + ω_s² φ = −a_h / L_s,          (1)

where a_h is the horizontal end-effector acceleration projected onto the reach-radial direction, a_h = cos θ₃ · a_ee,x + sin θ₃ · a_ee,y. The coupling is bidirectional: the carrying motion excites φ through (1), and the swinging mass m_s reacts back on the arm through the coupling terms of the 4-DOF inertia, Coriolis, and gravity matrices. As a documented simplification, only the participating mass m_s is attached to the dynamic model; the non-sloshing liquid remainder and the empty container are not modeled as an additional rigid hand payload. This simplification is stress-tested by the k_m sensitivity sweep, whose range k_m ∈ {0.3, 0.5, 0.7} brackets the effect of missing or extra carried mass.

### 3) Model validation

The derived dynamics are verified automatically at every derivation. First, the 4-DOF model reduces exactly (residuals < 10⁻⁹) to three independently derived special cases: the 3-DOF arm with the slosh coordinate decoupled, a textbook two-link planar arm, and a one-DOF compound pendulum. Second, M(q) is confirmed symmetric (< 10⁻⁹) and positive definite at randomly sampled configurations, and the skew-symmetry of Ṁ − 2C is verified by central finite differences (residual < 10⁻⁵), confirming Christoffel consistency. Third, an unforced free swing of the full model integrated with the exported numeric functions (ode45, relative tolerance 10⁻¹⁰) conserves total energy to a relative drift below 10⁻⁵.

## B. Task set and reference motion

A task is defined by the pair (f, d): the container fill level f and the reach distance d. In every task the hand carries the container along a straight line from the fixed start point P₀ = [0.55, 0, −0.15] m (base frame at the shoulder-rotation axis) in the fixed unit direction u = normalize([0.94, 0.23, 0.25]) for a distance d. The start pose was chosen for a natural elbow-down arm configuration (approximately 137° elbow flexion at the start, relaxing to approximately 50° at the farthest target), keeping all reference poses within the human range of motion.

The hand path follows a quintic (minimum-jerk) time-scaling s(τ) = 10τ³ − 15τ⁴ + 6τ⁵ with τ = t/T_move, which imposes zero velocity and zero acceleration at both endpoints. The joint-space reference is obtained by closed-form inverse kinematics on the elbow-down branch; workspace membership is asserted for every sample, and the forward–inverse kinematics round trip is verified to below 10⁻⁹. The reference is sampled at dt = 0.005 s.

The movement time is fixed at T_move = 1.0 s for all tasks, independent of distance. This is a deliberate design decision: with fixed time, longer reaches require higher hand accelerations and therefore excite more slosh, making reach distance a genuine second axis of task difficulty. Had the movement time been scaled with distance, peak accelerations would roughly equalize across the grid and the two-dimensional task space would collapse to a single effective difficulty axis.

## C. Required torque and task difficulty

By design, the study contains no feedback tracking controller: the arm follows the prescribed reference exactly, and all claims concern the distribution of the torque required to do so, not tracking performance. The only forward-integrated state is the slosh angle: (1) is integrated with ode45, with the forcing term a_h(t) supplied by a piecewise-cubic (pchip) interpolant of the reference end-effector acceleration.

The required joint torque is then obtained by inverse dynamics along the reference. At each sample the actuated rows (1–3) of

  τ = M(q)q̈ + C(q, q̇)q̇ + G(q)          (2)

are evaluated at the reference joint trajectory combined with the simulated slosh state, yielding the required torque profile τ_total(t) ∈ ℝ³ for each task.

From this profile we define a scalar, controller-independent task difficulty

  D = ∫₀ᵀ Σⱼ |τ_total,j(t)| dt   (N·m·s),          (3)

computed by trapezoidal integration. D is an a-priori property of the task under the model — it depends only on (f, d) and the dynamics, not on any assistance law.

## D. Assistance laws

The exoskeleton assists by taking a fraction α of the required torque at every joint:

  τ_exo = α · τ_total,  τ_human = (1 − α) · τ_total,          (4)

with one scalar α per task, held constant over the carry. Defining the human effort as E_human = ∫₀ᵀ Σⱼ |τ_human,j| dt, the constancy of α makes the identity

  E_human = (1 − α) · D          (5)

exact. This identity is the analytical basis of the adopted controller.

Four laws for choosing α are compared:

1. **Fixed:** α = 0.5, a non-adaptive baseline.
2. **Fill-proportional:** α = α_min + (α_max − α_min)·f. This is a genuine adaptive baseline: over the evaluation grid its commanded α spans [0.25, 0.72], so its weaker performance cannot be attributed to a frozen output.
3. **Difficulty-adaptive (adopted):** α = 1 − E*/D. This is the closed-form solution of (1 − α)·D = E* obtained from identity (5) — it holds the predicted human effort at a target value E* (Sec. E) for every task, regardless of what makes the task hard. The law has no free parameters beyond the shared clamps below: E* is fully determined by the effort band, which is fixed before any evaluation data is seen.
4. **Difficulty-linear (ablation):** a linear ramp of α from α_min to α_max as D traverses [D_lo, D_hi], the difficulty range observed on the calibration grid. This law uses the same difficulty signal as the adopted law and matches it on the nominal model; it is retained as an ablation to separate the value of the difficulty signal from the value of the closed-form schedule shape, and is rejected on sensitivity grounds (Sec. F).

All adaptive laws share identical clamps α ∈ [α_min, α_max] = [0.2, 0.72]. The lower bound keeps the assistance meaningfully engaged on easy tasks; the upper bound keeps the wearer physically engaged and excludes the trivial full-takeover solution α = 1. Bounded assistance has precedent in the stiffness bounds of Zhang et al. [1] and the retained "perceptible share" of Bai et al. [2]; the specific values 0.2 and 0.72 are a-priori design constants of this study, fixed before any results were produced, applied identically to every controller, and reported explicitly through clamp accounting (Sec. F). In contrast to the reactive, sensor-driven load sharing of Bai et al. [2], the adopted law schedules α a priori from the task-difficulty model, targets an effort band for a healthy wearer rather than therapeutic slosh retention, and is characterized over a two-dimensional task space with a worn multi-DOF exoskeleton model.

## E. Calibration and evaluation protocol

**Effort band.** The target for the wearer is a band [E_low, E_high] on E_human. Tasks with E_human below the band are classified as *over-assisted*, inside as *in band*, and above as *under-supported*. The ceiling E_high = 3.4 N·m·s was fixed a priori in the parameter file and never adjusted to results; it corresponds to a mean torque of roughly 1.1 N·m per joint sustained over the 1-s carry, a small fraction (on the order of 2%) of voluntary shoulder and elbow strength. It is a design constant, not a cited physiological threshold; physiological validation of the band (EMG, perceived exertion, metabolic cost) is future work.

**Held-out calibration.** All tuned quantities were set by fixed rules on a calibration grid disjoint from the evaluation grid. The calibration grid contains 25 tasks, f ∈ linspace(0.15, 0.95, 5) × d ∈ linspace(0.125, 0.475, 5) m, deliberately interleaved between evaluation grid points; disjointness is asserted programmatically. Three quantities are derived from it, each by a fixed rule: D_lo and D_hi are the minimum and maximum calibration difficulty, and the band floor is E_low = (1 − α_min)·D_lo = 2.20 N·m·s — the effort that the least-helpful in-bounds controller leaves on the easiest calibration task, below which assistance cannot be the explanation for low effort. The effort target of the adopted law is the band midpoint, E* = (E_low + E_high)/2 = 2.80 N·m·s, and is therefore fully determined before any evaluation data is seen (ceiling fixed a priori, floor by rule on calibration data). Nothing else is tuned anywhere in the pipeline.

**Evaluation.** All reported outcomes come from a separate 10×10 evaluation grid of 100 tasks, f ∈ [0.1, 1.0] × d ∈ [0.1, 0.5] m. The corners of this grid lie outside the difficulty range spanned by the calibration grid, so the evaluation includes genuine extrapolation beyond the calibrated range. In summary: every tuned quantity was set by fixed rules on 25 held-out tasks, and coverage is measured on 100 disjoint tasks whose extremes exceed the calibrated range.

## F. Outcome measures and robustness procedures

**Primary outcome.** For each controller, *coverage* is the fraction of the 100 evaluation tasks whose E_human lies inside the band.

**Secondary outcomes.** E_exo (the same integral applied to τ_exo) guards against a trivial full-takeover solution; the peak momentary human demand τ_pk,human = max_t Σⱼ |τ_human,j(t)| captures instantaneous load; the peak slosh angle per task characterizes excitation; and clamp accounting records, with tolerance 10⁻⁹, which tasks each adaptive law controls freely versus where α_min or α_max binds, so clamp-limited behavior is reported rather than averaged away. Tracking error is deliberately not a metric: the motion is prescribed and followed by construction, so no tracking claim is made or implied.

**Damping back-reaction bound.** Evaluating (2) at the reference neglects the slosh damper's reaction torque on the actuated joints. This term is bounded at the worst-case task (f = 1, d = 0.5 m): its maximum, approximately 0.011 N·m, is about 0.1% of the roughly 9.6 N·m peak joint torque there, and is therefore negligible.

**Surrogate sensitivity.** Because k_m and L_s are model constants rather than measured values, the entire pipeline is re-run under six surrogate conditions, k_m ∈ {0.3, 0.5, 0.7} × L_s ∈ {0.5·R_c, 1.0·R_c}. Critically, each condition repeats the *full* calibrate-then-evaluate procedure — calibration-grid difficulties, D_lo/D_hi, E_low, and E* are all re-derived under the perturbed surrogate — rather than merely re-evaluating controllers tuned on the nominal model. The rejected linear-ramp ablation is evaluated under this identical protocol, which is what exposes its schedule-shape failure and motivated the adoption of the closed-form law.

---

## References

[1] Zhang et al., "…," *Frontiers in Bioengineering and Biotechnology*, 11:1332689, 2024.

[2] Bai et al., "…," *Biomimetics*, 10(12):815, 2025.
