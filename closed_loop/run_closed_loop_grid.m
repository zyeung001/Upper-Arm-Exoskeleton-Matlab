function g = run_closed_loop_grid(opts)
%RUN_CLOSED_LOOP_GRID Closed-loop band coverage across the task grid.
%   g = RUN_CLOSED_LOOP_GRID          5x5 grid from cl_params (extremes)
%   g = RUN_CLOSED_LOOP_GRID(Rebuild=true)
%
%   g = RUN_CLOSED_LOOP_GRID(Kappa=0.095)     other human strength fraction
%
%   The study's primary question, asked in closed loop: over a grid of
%   tasks spanning the sweep's extremes, which alpha law leaves the human
%   working inside the target band WITHOUT asking them for torque they
%   cannot produce? The three laws coincide on mid-difficulty tasks by
%   construction - their differences live at the corners (fixed over-assists
%   easy carries and under-supports hard ones; the difficulty law holds
%   effort at the band midpoint everywhere) - so this grid, not a single
%   task, is the closed-loop analogue of the sweep's coverage headline.
%
%   A task PASSES only if it is in band AND the human never hit their
%   strength cap (cl_params.u_h_max). Both halves are needed: capping
%   DEFLATES the measured effort, so a law can otherwise be scored "in
%   band" precisely because the human failed to deliver what it demanded.
%   Overload also has a task cost - the applied torque falls short of what
%   the carry needs and tracking degrades - which is why panel 2 of the
%   figure reports RMSE. This is where the mentor's accuracy metric finally
%   discriminates between the laws.
%
%   One simulation per task per law (human torque lag + strength cap, slosh
%   excited by the carry), 75 sims by default; takes ~10-15 minutes. alpha
%   is computed A PRIORI per task via controllers.m, as everywhere else.
%
%   Figure -> results/figures/closed_loop_grid.png
%   Data   -> results/closed_loop_grid.mat

arguments
    opts.Rebuild (1,1) logical = false
    opts.Figures (1,1) logical = true
    opts.Kappa (1,1) double = NaN    % human strength fraction; default cl.kappa
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

% The closed-loop grid is EVALUATION data: the critical-kappa prediction is
% computed on the calibration grid (cl_critical_kappa), so a task that is also
% a calibration task would be testing the prediction on its own training data.
% Same discipline, and same failure mode, as the assert in calibrate_controller.
assert(~any(ismembertol(cl.grid_f, p.calib.f, 1e-9)) && ...
       ~any(ismembertol(cl.grid_d, p.calib.d, 1e-9)), ...
    'run_closed_loop_grid: closed-loop grid overlaps the calibration grid');

if ~isnan(opts.Kappa)          % the kappa sweep drives this (run_kappa_sweep)
    cl.kappa   = opts.Kappa;
    cl.u_h_max = cl.kappa * cl.MVC;
end
cap = cl.u_h_max(:);
laws = {'fixed', 'fill', 'difficulty'};
nl = numel(laws);
nf = numel(cl.grid_f);
nd = numel(cl.grid_d);

mdl = 'arm_closed_loop';
mdlfile = fullfile(here, [mdl '.slx']);
if cl_needs_rebuild(mdlfile) || opts.Rebuild
    build_closed_loop_model();
end

g.f = cl.grid_f;  g.d = cl.grid_d;  g.laws = laws;
g.kappa    = cl.kappa;
g.D        = zeros(nf, nd);
g.alpha    = zeros(nf, nd, nl);
g.E_human  = zeros(nf, nd, nl);   % effort the human actually DELIVERED
g.E_demand = zeros(nf, nd, nl);   % effort the law ASKED for (differs if capped)
g.status   = zeros(nf, nd, nl);   % -1 below band / 0 in band / +1 above
g.overload = false(nf, nd, nl);   % the human hit their strength cap
g.duty     = zeros(nf, nd, nl);   % fraction of the carry spent at the cap
g.ok       = false(nf, nd, nl);   % in band AND within strength - the verdict
g.rmse_ee  = zeros(nf, nd, nl);

fprintf('\nClosed-loop grid: %d fills x %d distances x %d laws = %d simulations\n', ...
    nf, nd, nl, nf * nd * nl);
for j = 1:nd
    d = cl.grid_d(j);
    [Xref, VelRef, traj, tt, qref] = cl_reference(d, cl.Tend_grid, p);
    q0 = traj.q(:, 1);
    for i = 1:nf
        f = cl.grid_f(i);
        ss   = slosh_surrogate(f, p);   % same rules as the sweep
        b_s  = ss.b_s;
        pvec = ss.pvec;
        G0   = G_fun([q0; 0], pvec);

        [tau_total, ~, ~] = compute_required_torque(traj, f, p);
        D = compute_difficulty(traj.t, tau_total);
        g.D(i, j) = D;

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
        base = base.setVariable('cl_ideal', 0);
        base = base.setVariable('cl_Tend',  cl.Tend_grid);

        for L = 1:nl
            [~, ~, a] = controllers(laws{L}, tau_total, f, D, p);
            g.alpha(i, j, L) = a;
            in = base.setVariable('cl_alpha', a);
            in = in.setVariable('cl_uh0', max(-cap, min(cap, (1 - a) * G0(1:3))));
            out = sim(in);

            t    = out.y.Time.';
            uhum = out.uhum.Data.';          % delivered (after lag + cap)
            uexo = out.uexo.Data.';
            uhc  = (1 - a) * out.ucmd.Data.';   % demanded of the human
            win  = t <= p.sim.T_move;
            mb = compute_metrics(t(win), uhum(:, win), uexo(:, win), p);
            ov = cl_overload(t, uhum, uhc, win, cl);
            g.E_human(i, j, L)  = mb.E_human;
            g.status(i, j, L)   = mb.status;
            g.E_demand(i, j, L) = ov.E_demanded;
            g.overload(i, j, L) = ov.overload;
            g.duty(i, j, L)     = ov.duty;
            g.ok(i, j, L)       = (mb.status == 0) && ~ov.overload;
            mt = cl_metrics(t, out.y.Data.', out.err.Data.', out.u.Data.', ...
                out.phi.Data(:).', tt, qref, pvec, cl);
            g.rmse_ee(i, j, L) = mt.rmse_ee;
        end
        fprintf('  f = %.2f, d = %.2f: D = %5.2f | E_human %s | RMSE %s mm%s\n', ...
            f, d, D, ...
            strjoin(compose('%.2f', squeeze(g.E_human(i, j, :)).'), '/'), ...
            strjoin(compose('%.1f', 1e3 * squeeze(g.rmse_ee(i, j, :)).'), '/'), ...
            overload_tag(squeeze(g.overload(i, j, :)), laws));
    end
end

% ---- Console summary ---------------------------------------------------------
n_tasks = nf * nd;
fprintf(['\n  Closed-loop coverage, band = [%.2f, %.2f] N m s, human at ' ...
    '%.0f%% MVC ([%.1f %.1f %.1f] N m):\n'], p.band.E_low, p.band.E_high, ...
    100 * cl.kappa, cap);
fprintf(['    A task PASSES only if the human''s effort lands in the band AND ' ...
    'they were\n    never asked for more torque than they can produce ' ...
    '(a capped human''s measured\n    effort is deflated by their own ' ...
    'failure, so in-band alone can be "passed"\n    by overloading them).\n\n']);
g.coverage = zeros(1, nl);   % in band (raw, disclosed as secondary)
g.pass     = zeros(1, nl);   % in band AND within strength (the verdict)
for L = 1:nl
    s = g.status(:, :, L);
    ov = g.overload(:, :, L);
    g.coverage(L) = nnz(s == 0) / n_tasks;
    g.pass(L)     = nnz(g.ok(:, :, L)) / n_tasks;
    fprintf(['    %-12s PASS %d/%d (%2.0f%%) | in band %d/%d ' ...
        '[below %d, above %d] | overloaded %d/%d | mean RMSE %.1f mm\n'], ...
        laws{L}, nnz(g.ok(:, :, L)), n_tasks, 100 * g.pass(L), ...
        nnz(s == 0), n_tasks, nnz(s < 0), nnz(s > 0), nnz(ov), n_tasks, ...
        1e3 * mean(g.rmse_ee(:, :, L), 'all'));
end
% Tracking cost of overload: RMSE on tasks where the human was capped.
any_ov = any(g.overload, 3);
if any(any_ov(:))
    fprintf('\n    On the %d task(s) where SOME law overloads the human:\n', ...
        nnz(any_ov));
    for L = 1:nl
        r = g.rmse_ee(:, :, L);
        o = g.overload(:, :, L);
        fprintf('      %-12s mean RMSE %.1f mm (%s)\n', laws{L}, ...
            1e3 * mean(r(any_ov)), ...
            sprintf('overloads %d of them', nnz(o & any_ov)));
    end
end
fprintf('\n');

g.p = p;  g.cl = cl;
resdir = fullfile(root, 'results');
if ~exist(resdir, 'dir'), mkdir(resdir); end
save(fullfile(resdir, 'closed_loop_grid.mat'), 'g');
fprintf('Saved %s\n', fullfile(resdir, 'closed_loop_grid.mat'));

if opts.Figures
    cl_grid_figure(g, fullfile(resdir, 'figures'));
end
end

% ------------------------------------------------------------------------
function s = overload_tag(ov, laws)
% Name the laws that over-demanded the human on this task (console only).
if ~any(ov)
    s = '';
else
    s = sprintf('  <- OVERLOADS: %s', strjoin(laws(logical(ov)), ', '));
end
end
