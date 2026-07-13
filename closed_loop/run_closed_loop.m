function res = run_closed_loop(f, d, opts)
%RUN_CLOSED_LOOP Closed-loop Simulink validation of the assistance study.
%   res = RUN_CLOSED_LOOP            default task (f = 0.8, d = 0.4)
%   res = RUN_CLOSED_LOOP(f, d)      fill level f, reach distance d (m)
%   res = RUN_CLOSED_LOOP(f, d, Rebuild=true)  force model rebuild
%
%   One closed-loop simulation per task: a PD + gravity-compensation
%   controller tracks the sweep's quintic reference on the FULL nonlinear
%   4-DOF plant (arm + slosh pendulum, out = sim("arm_closed_loop.slx")).
%   The slosh starts at rest and is excited by the arm motion itself,
%   exactly as in the open-loop sweep; it acts back on the arm as a true
%   two-way disturbance (F_water in the mentor's diagram - its sign just
%   says which way the liquid is currently pushing).
%
%   The MEASURED total torque u(t) is then split by the three alpha laws
%   (fixed / fill / difficulty, controllers.m) - alpha is constant per
%   task and computed A PRIORI from the reference trajectory - and the
%   controllers are compared on the closed-loop measurements:
%   cumulative/final human effort vs the target band, and peak human
%   torque. Tracking metrics (RMSE, settle time, energy of u) are shared
%   by all three laws since they split the same total.
%
%   Figures -> results/figures/closed_loop_controllers.png (the comparison)
%              results/figures/closed_loop_tracking.png    (context)
%   Data    -> results/closed_loop_results.mat  (input to animate_closed_loop)

arguments
    f (1,1) double = NaN
    d (1,1) double = NaN
    opts.Rebuild (1,1) logical = false
    opts.Figures (1,1) logical = true
end

% ---- Paths / prerequisites -------------------------------------------------
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(root, fullfile(root, 'generated'), here);
if exist(fullfile(root, 'generated', 'M_fun.m'), 'file') ~= 2
    derive_dynamics();
end

p  = calibrate_controller(params());
cl = cl_params();
if isnan(f), f = cl.f_default; end
if isnan(d), d = cl.d_default; end

% ---- Slosh surrogate for this fill level (same rules as the sweep) ---------
m_liq = p.cont.rho * pi * p.cont.Rc^2 * (f * p.cont.Hc);
m_s   = p.slosh.k_m * m_liq;
L_s   = p.slosh.L_s_factor * p.cont.Rc;
w_s   = sqrt(p.g / L_s);
b_s   = 2 * p.slosh.zeta_s * w_s * m_s * L_s^2;   % matches the damped surrogate
pvec  = pack_pvec(p, m_s, L_s);

% ---- Reference + a-priori task difficulty and alphas ------------------------
[Xref, VelRef, traj, tt, qref] = cl_reference(d, cl.Tend, p);
[tau_total, ~, ~] = compute_required_torque(traj, f, p);
D = compute_difficulty(traj.t, tau_total);

laws = {'fixed', 'fill', 'difficulty'};
alpha = zeros(1, numel(laws));
for L = 1:numel(laws)
    [~, ~, alpha(L)] = controllers(laws{L}, tau_total, f, D, p);
end
i_adopt = find(strcmp(laws, 'difficulty'));

% ---- Model ------------------------------------------------------------------
mdl = 'arm_closed_loop';
mdlfile = fullfile(here, [mdl '.slx']);
if ~exist(mdlfile, 'file') || opts.Rebuild
    build_closed_loop_model();
end

% ---- Simulate (slosh starts at rest, excited by the carry itself) ----------
q0 = traj.q(:, 1);
in = Simulink.SimulationInput(mdl);
in = in.setVariable('cl_Xref',   Xref);
in = in.setVariable('cl_VelRef', VelRef);
in = in.setVariable('cl_x0',    [q0; 0; zeros(4, 1)]);
in = in.setVariable('cl_Kp',    cl.Kp);
in = in.setVariable('cl_Kd',    cl.Kd);
in = in.setVariable('cl_alpha', alpha(i_adopt));   % in-model split = adopted law
in = in.setVariable('cl_pvec',  pvec);
in = in.setVariable('cl_bs',    b_s);
in = in.setVariable('cl_rigid', 0);
in = in.setVariable('cl_Tend',  cl.Tend);
out = sim(in);

res.t   = out.y.Time.';
res.q   = out.y.Data.';        % 3xN simulated joint angles
res.err = out.err.Data.';      % 3xN joint tracking error
res.u   = out.u.Data.';        % 3xN total torque (exo + human)
res.phi = out.phi.Data(:).';   % 1xN slosh angle
assert(numel(res.t) == size(res.err, 2) && numel(res.t) == numel(res.phi), ...
    'run_closed_loop: logged signals disagree on time base');

% ---- Metrics ------------------------------------------------------------------
% Tracking metrics: law-independent (all laws split the same total u).
res.metrics = cl_metrics(res.t, res.q, res.err, res.u, res.phi, tt, qref, pvec, cl);

% Controller comparison on the MEASURED torque, over the carry window
% (band-comparable units, same integration window as the sweep).
win = res.t <= p.sim.T_move;
res.E_cum = zeros(numel(laws), numel(res.t));
for L = 1:numel(laws)
    a = alpha(L);
    res.law(L) = compute_metrics(res.t(win), ...
        (1 - a) * res.u(:, win), a * res.u(:, win), p);
    res.E_cum(L, :) = cumtrapz(res.t, (1 - a) * sum(abs(res.u), 1));
end

% ---- Console report ------------------------------------------------------------
astr = arrayfun(@(L) sprintf('%s = %.2f', laws{L}, alpha(L)), ...
    1:numel(laws), 'UniformOutput', false);
fprintf('\nClosed-loop validation: fill = %.2f, distance = %.2f m\n', f, d);
fprintf('  difficulty D = %.2f N m s (a priori), alpha: %s\n', D, strjoin(astr, ', '));
m = res.metrics;
fprintf(['  tracking (all laws): RMSE = %.1f mm, peak = %.1f mm, ' ...
    'settle = %.2f s, energy(u) = %.1f, peak slosh = %.1f deg\n'], ...
    1e3*m.rmse_ee, 1e3*m.max_ee, m.t_settle, m.energy_u, rad2deg(m.peak_slosh));
fprintf('  target human-effort band = [%.2f, %.2f] N m s (carry window)\n\n', ...
    p.band.E_low, p.band.E_high);
status_str = {'BELOW band (over-assisted, wasteful)', 'in band', ...
              'ABOVE band (under-supported)'};
fprintf('  %-12s %6s %9s %9s %8s %12s   %s\n', 'controller', 'alpha', ...
    'E_human', 'open-loop', 'E_exo', 'pk tau_hum', 'band status');
for L = 1:numel(laws)
    ml = res.law(L);
    fprintf('  %-12s %6.2f %9.2f %9.2f %8.2f %9.1f N m   %s\n', laws{L}, ...
        alpha(L), ml.E_human, (1 - alpha(L)) * D, ml.E_exo, ...
        ml.tau_pk_human, status_str{ml.status + 2});
end
fprintf('\n');

% ---- Package + save --------------------------------------------------------------
res.f = f;  res.d = d;  res.D = D;
res.laws = laws;  res.alpha = alpha;
res.tt = tt;  res.qref = qref;  res.traj = traj;
res.pvec = pvec;  res.p = p;  res.cl = cl;

resdir = fullfile(root, 'results');
if ~exist(resdir, 'dir'), mkdir(resdir); end
save(fullfile(resdir, 'closed_loop_results.mat'), 'res');
fprintf('Saved %s\n', fullfile(resdir, 'closed_loop_results.mat'));

if opts.Figures
    cl_figures(res, fullfile(resdir, 'figures'));
end
end
