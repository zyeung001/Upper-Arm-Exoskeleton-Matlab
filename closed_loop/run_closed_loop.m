function res = run_closed_loop(f, d, opts)
%RUN_CLOSED_LOOP Closed-loop Simulink comparison of the assistance laws.
%   res = RUN_CLOSED_LOOP            default task (f = 0.8, d = 0.4)
%   res = RUN_CLOSED_LOOP(f, d)      fill level f, reach distance d (m)
%   res = RUN_CLOSED_LOOP(f, d, Rebuild=true)  force model rebuild
%
%   The mentor's whiteboard, made consequential: a PD + gravity-comp
%   controller tracks the sweep's quintic reference on the FULL nonlinear
%   4-DOF plant (out = sim("arm_closed_loop.slx")); the slosh starts at
%   rest, is excited by the carry itself, and acts back on the arm as the
%   F_water disturbance. The HUMAN delivers their (1-alpha) share through
%   a first-order torque-development lag (cl_params.tau_h, a priori) while
%   the exo responds instantly - so the alpha law changes the physics:
%   laws that leave more share on the human track worse.
%
%   Four simulations per task:
%     1-3. one per alpha law (fixed / fill / difficulty, lag active) -
%          scored on the whiteboard metrics: Accuracy (RMSE), Time
%          (settle), Energy of u, plus human effort vs the target band
%          (carry window, band-comparable units).
%     4.   ideal human (lag bypassed) - the validation cross-check that
%          closed-loop effort reproduces the open-loop sweep's (1-alpha)*D.
%
%   alpha is constant per task and computed A PRIORI from the reference
%   trajectory via controllers.m, exactly as in the sweep.
%
%   Figures -> results/figures/closed_loop_controllers.png (the comparison)
%              results/figures/closed_loop_validation.png  (ideal-human check)
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
ss   = slosh_surrogate(f, p);
b_s  = ss.b_s;      % matches the damped surrogate in compute_required_torque
pvec = ss.pvec;

% ---- Reference + a-priori task difficulty and alphas ------------------------
[Xref, VelRef, traj, tt, qref] = cl_reference(d, cl.Tend, p);
[tau_total, ~, ~] = compute_required_torque(traj, f, p);
D = compute_difficulty(traj.t, tau_total);

laws = {'fixed', 'fill', 'difficulty'};
nl = numel(laws);
alpha = zeros(1, nl);
for L = 1:nl
    [~, ~, alpha(L)] = controllers(laws{L}, tau_total, f, D, p);
end
i_adopt = find(strcmp(laws, 'difficulty'));

% ---- Model ------------------------------------------------------------------
mdl = 'arm_closed_loop';
mdlfile = fullfile(here, [mdl '.slx']);
if cl_needs_rebuild(mdlfile) || opts.Rebuild
    build_closed_loop_model();
end

% ---- Simulate: one run per law (lag + strength cap), + the ideal check -----
q0 = traj.q(:, 1);
G0 = G_fun([q0; 0], pvec);   % human starts already holding their static share
cap = cl.u_h_max(:);
base = Simulink.SimulationInput(mdl);
base = base.setVariable('cl_Xref',   Xref);
base = base.setVariable('cl_VelRef', VelRef);
base = base.setVariable('cl_x0',    [q0; 0; zeros(4, 1)]);
base = base.setVariable('cl_Kp',    cl.Kp);
base = base.setVariable('cl_Kd',    cl.Kd);
base = base.setVariable('cl_pvec',  pvec);
base = base.setVariable('cl_bs',    b_s);
base = base.setVariable('cl_rigid', 0);
base = base.setVariable('cl_tauh',  cl.tau_h);
base = base.setVariable('cl_uhmax', cap);
base = base.setVariable('cl_Tend',  cl.Tend);

win_fun = @(t) t <= p.sim.T_move;   % carry window (band-comparable)
for L = 1:nl
    in = base.setVariable('cl_alpha', alpha(L));
    in = in.setVariable('cl_ideal', 0);
    % Lag integrator starts at the human's static gravity share, clamped
    % into the cap (a user too weak to even hold the pose starts AT it).
    in = in.setVariable('cl_uh0', ...
        max(-cap, min(cap, (1 - alpha(L)) * G0(1:3))));
    r = extract_run(sim(in), laws{L});
    r.ucmd_hum = (1 - alpha(L)) * r.ucmd;   % what the law ASKED of the human
    r.metrics  = cl_metrics(r.t, r.q, r.err, r.u, r.phi, tt, qref, pvec, cl);
    r.metrics.energy_ucmd = trapz(r.t, sum(r.ucmd.^2, 1));
    win = win_fun(r.t);
    r.band = compute_metrics(r.t(win), r.uhum(:, win), r.uexo(:, win), p);
    r.over = cl_overload(r.t, r.uhum, r.ucmd_hum, win, cl);
    % The verdict: assistance is right only if the human's effort lands in
    % the band AND they were never asked for more than they can produce.
    % A capped human's measured effort is DEFLATED by their own failure, so
    % in-band alone could be "passed" by overloading them - hence the AND.
    r.ok    = (r.band.status == 0) && ~r.over.overload;
    r.E_cum = cumtrapz(r.t, sum(abs(r.uhum), 1));
    res.runs(L) = r;
end
in = base.setVariable('cl_alpha', alpha(i_adopt));
in = in.setVariable('cl_ideal', 1);
in = in.setVariable('cl_uh0', zeros(3, 1));
ideal = extract_run(sim(in), 'ideal human');
ideal.metrics = cl_metrics(ideal.t, ideal.q, ideal.err, ideal.u, ideal.phi, ...
    tt, qref, pvec, cl);
win = win_fun(ideal.t);
for L = 1:nl   % post-hoc split of the ideal run's measured total torque
    a = alpha(L);
    ideal.band(L) = compute_metrics(ideal.t(win), ...
        (1 - a) * ideal.u(:, win), a * ideal.u(:, win), p);
end
res.ideal = ideal;

% ---- Console report ------------------------------------------------------------
astr = arrayfun(@(L) sprintf('%s = %.2f', laws{L}, alpha(L)), 1:nl, ...
    'UniformOutput', false);
fprintf('\nClosed-loop comparison: fill = %.2f, distance = %.2f m\n', f, d);
fprintf('  difficulty D = %.2f N m s (a priori), alpha: %s\n', D, strjoin(astr, ', '));
fprintf('  human torque lag tau_h = %.0f ms; exo instantaneous\n', 1e3 * cl.tau_h);
fprintf('  human strength cap = %.0f%% MVC -> [%.1f %.1f %.1f] N m per joint\n', ...
    100 * cl.kappa, cl.u_h_max);
fprintf('  target human-effort band = [%.2f, %.2f] N m s (carry window)\n\n', ...
    p.band.E_low, p.band.E_high);
status_str = {'BELOW band', 'in band', 'ABOVE band'};
fprintf('  %-12s %6s %9s %9s %9s %10s  %-11s %s\n', 'controller', 'alpha', ...
    'E_human', 'RMSE_ee', 't_settle', 'energy(u)', 'band', 'human at cap?');
for L = 1:nl
    r = res.runs(L);
    if r.over.hold_capped
        ostr = sprintf('AT CAP %2.0f%% of run, incl. the hold (deficit %.1f Nm)', ...
            100 * r.over.duty, r.over.deficit);
    elseif r.over.overload
        ostr = sprintf('AT CAP %2.0f%% of run (deficit %.1f Nm)', ...
            100 * r.over.duty, r.over.deficit);
    else
        ostr = 'within strength';
    end
    fprintf('  %-12s %6.2f %9.2f %6.1f mm %7.2f s %10.1f  %-11s %s\n', ...
        r.name, alpha(L), r.band.E_human, ...
        1e3 * r.metrics.rmse_ee, r.metrics.t_settle, r.metrics.energy_u, ...
        status_str{r.band.status + 2}, ostr);
end
ok_str = {'FAIL', 'PASS'};
fprintf('\n  Verdict (in band AND within strength): %s\n', strjoin( ...
    arrayfun(@(L) sprintf('%s = %s', laws{L}, ok_str{res.runs(L).ok + 1}), ...
    1:nl, 'UniformOutput', false), ',  '));
fprintf('\n  Validation, ideal human (RMSE %.1f mm; compare open-loop E = (1-a)*D):\n', ...
    1e3 * res.ideal.metrics.rmse_ee);
for L = 1:nl
    fprintf('  %-12s closed-loop %5.2f | open-loop %5.2f | %s\n', laws{L}, ...
        res.ideal.band(L).E_human, (1 - alpha(L)) * D, ...
        status_str{res.ideal.band(L).status + 2});
end
fprintf('\n');

% ---- Package + save --------------------------------------------------------------
res.f = f;  res.d = d;  res.D = D;
res.laws = laws;  res.alpha = alpha;  res.i_adopt = i_adopt;
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

% ------------------------------------------------------------------------
function r = extract_run(out, name)
% Pull logged signals out of a SimulationOutput into plain arrays (3xN/1xN).
r.name = name;
r.t    = out.y.Time.';
r.q    = out.y.Data.';
r.err  = out.err.Data.';
r.u    = out.u.Data.';       % applied total torque (exo + human delivered)
r.ucmd = out.ucmd.Data.';    % commanded total (PD + gravity comp)
r.uexo = out.uexo.Data.';
r.uhum = out.uhum.Data.';    % human torque as DELIVERED (after lag/bypass)
r.phi  = out.phi.Data(:).';
assert(numel(r.t) == size(r.err, 2) && numel(r.t) == numel(r.phi), ...
    'run_closed_loop: logged signals disagree on time base');
end
