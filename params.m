function p = params()
%PARAMS All constants for the slosh-aware adaptive-assistance simulation.
%   p = PARAMS() returns a struct. Nothing downstream hard-codes numbers.
%
%   Arm link values (m1, m2, L1, L2, lc1, lc2, I1, I2) are from
%   Zhang et al. 2024 (Front. Bioeng. Biotechnol. 11:1332689). The third
%   link (shoulder rotation about the vertical axis) is an extension of
%   that 2-link model with comparable values - an assumption, not a cited
%   value.

% ---- Arm links (3-DOF) --------------------------------------------------
% Joint 1 (th1): shoulder flexion, link 1 = upper arm      [Zhang et al.]
% Joint 2 (th2): elbow flexion,    link 2 = forearm        [Zhang et al.]
% Joint 3 (th3): shoulder rotation about vertical z-axis,
%                link 3 = horizontal base segment from the rotation axis
%                to the shoulder                            [extension]
p.arm.m1  = 1.0;   p.arm.m2  = 0.7;   p.arm.m3  = 0.7;    % kg
p.arm.L1  = 0.4;   p.arm.L2  = 0.4;   p.arm.L3  = 0.3;    % m
p.arm.lc1 = 0.2;   p.arm.lc2 = 0.2;   p.arm.lc3 = 0.15;   % m
p.arm.I1  = 0.25;  p.arm.I2  = 0.1;   p.arm.I3  = 0.1;    % kg m^2 (transverse, about COM)
p.g       = 9.8;                                           % m/s^2

% ---- Container / payload ------------------------------------------------
p.cont.Rc  = 0.05;   % container radius (m)
p.cont.Hc  = 0.15;   % container height (m)
p.cont.rho = 1000;   % liquid density (kg/m^3, water)

% ---- Slosh pendulum surrogate (Bai et al. 2025 pendulum-equivalent) -----
% m_liq(f) = rho*pi*Rc^2*(f*Hc);  m_s(f) = k_m*m_liq(f);  L_s = Rc
% (container radius sets the slosh length scale, matching Bai et al.'s
% w_n = sqrt(g/R));  w_s = sqrt(g/L_s);  fixed damping ratio zeta_s.
p.slosh.k_m    = 0.5;    % participating-mass fraction (tunable model constant)
p.slosh.zeta_s = 0.05;   % slosh damping ratio (fixed)

% ---- Reach geometry -----------------------------------------------------
% Start point of the carry (m, base frame at the shoulder-rotation axis).
% Chosen so the elbow-down IK gives a natural pose: hand slightly below the
% shoulder in front of the body, elbow flexion ~137 deg at the start (well
% inside human ROM), relaxing toward ~50 deg at the farthest target.
% The target is start + d*u_reach; u_reach is a unit vector, mostly radial
% with a small tangential and upward component. All grid targets must stay
% inside the arm workspace (asserted in make_trajectory).
p.reach.start = [0.55; 0; -0.15];
u = [0.94; 0.23; 0.25];
p.reach.dir   = u / norm(u);

% ---- Simulation ----------------------------------------------------------
% Decision A: fixed movement time for ALL distances, so longer reaches are
% faster/more aggressive and excite more slosh.
p.sim.T_move = 1.0;    % s
p.sim.dt     = 0.005;  % s

% ---- Sweep grids ---------------------------------------------------------
p.sweep.n_f   = 10;  p.sweep.f_min = 0.1;  p.sweep.f_max = 1.0;   % fill level
p.sweep.n_d   = 10;  p.sweep.d_min = 0.1;  p.sweep.d_max = 0.5;   % reach (m)

% ---- Controllers (Decision B: parameterized, adjustable) ----------------
% alpha_min > 0 and alpha_max < 1: the human stays engaged and the exo
% never does everything (bounded assistance, cf. Zhang et al.'s
% Kmin/Kmax; Bai et al.'s retained perceptible share).
p.ctrl.alpha_fixed = 0.5;   % controller 1
p.ctrl.alpha_min   = 0.2;   % lower bound for controllers 2 and 3
p.ctrl.alpha_max   = 0.72;  % upper bound for controllers 2 and 3
% Difficulty-adaptive ramp (controller 3): alpha rises linearly from
% alpha_min at D_lo to alpha_max at D_hi, clamped. Tuned to the observed
% difficulty range of the sweep, D in [2.41, 9.62] N m s (printed by
% run_sweep), so that (1-alpha(D))*D stays inside the target band.
p.ctrl.D_lo = 2.4;    % N m s
p.ctrl.D_hi = 9.7;    % N m s

% ---- Target human-effort band (units of E_human, N m s) -----------------
% The load-bearing assumption: below E_low = over-assisted (wasteful),
% above E_high = under-supported. Precedent for a bounded, non-zero human
% share: Zhang et al. 2024 (effort/deviation reward, bounded stiffness),
% Bai et al. 2025 (perceptible retained share). E_high ~ 3.4 N m s is
% ~1.1 N m mean torque per joint over the 1 s carry, a comfortable
% fraction of shoulder/elbow voluntary strength. E_low sits just under
% the lightest task at minimum assistance (0.8 * D_min = 1.93): any lower
% floor would call even minimal help on the lightest task "over-assisted".
p.band.E_low  = 1.9;
p.band.E_high = 3.4;
end
