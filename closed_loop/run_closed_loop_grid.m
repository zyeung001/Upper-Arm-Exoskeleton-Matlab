function g = run_closed_loop_grid(opts)
%RUN_CLOSED_LOOP_GRID Closed-loop band coverage across the task grid.
%   g = RUN_CLOSED_LOOP_GRID          3x3 grid from cl_params (extremes)
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
%   excited by the carry), 27 sims by default; takes a few minutes. alpha is
%   computed A PRIORI per task via controllers.m, as everywhere else.
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

L_s = p.slosh.L_s_factor * p.cont.Rc;
w_s = sqrt(p.g / L_s);

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
        m_liq = p.cont.rho * pi * p.cont.Rc^2 * (f * p.cont.Hc);
        m_s   = p.slosh.k_m * m_liq;
        b_s   = 2 * p.slosh.zeta_s * w_s * m_s * L_s^2;
        pvec  = pack_pvec(p, m_s, L_s);
        G0    = G_fun([q0; 0], pvec);

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
    grid_figure(g, fullfile(resdir, 'figures'));
end
end

% ------------------------------------------------------------------------
function grid_figure(g, outdir)
% Three panels: who ends in the band (and who got there by overloading the
% human), what that costs in tracking, and the resulting pass rate.
if ~exist(outdir, 'dir'), mkdir(outdir); end
col_ctrl = [0 114 178; 230 159 0; 0 158 115] / 255;
nl = numel(g.laws);
yb = [g.p.band.E_low, g.p.band.E_high];
[Dv, order] = sort(g.D(:));

fig = new_fig([1320 420]);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf(['Closed loop across the task grid (%dx%d tasks): human at ' ...
    '%.0f%% MVC, torque lag %.0f ms'], numel(g.f), numel(g.d), ...
    100 * g.kappa, 1e3 * g.cl.tau_h));

% (1) Measured human effort vs difficulty, with overloaded tasks called out
ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
h = gobjects(1, nl);
for L = 1:nl
    E = g.E_human(:, :, L);
    h(L) = plot(ax, Dv, E(order), '-o', 'Color', col_ctrl(L,:), ...
        'MarkerFaceColor', col_ctrl(L,:), 'MarkerSize', 5, 'LineWidth', 1.4);
end
% An overloaded task's measured effort is NOT comparable to the band: it is
% low because the human physically failed to deliver what was demanded. Mark
% those points, and ghost in what the law actually asked for.
hov = gobjects(1);  hgh = gobjects(1);
for L = 1:nl
    ov = g.overload(:, :, L);  ov = ov(order);
    E  = g.E_human(:, :, L);   E  = E(order);
    Ed = g.E_demand(:, :, L);  Ed = Ed(order);
    if any(ov)
        hov = plot(ax, Dv(ov), E(ov), 'x', 'Color', [0.15 0.15 0.15], ...
            'MarkerSize', 11, 'LineWidth', 1.6);
        hgh = plot(ax, Dv(ov), Ed(ov), 'v', 'Color', col_ctrl(L,:), ...
            'MarkerFaceColor', 'none', 'MarkerSize', 6, 'LineWidth', 1.0);
    end
end
hband = yregion(ax, yb(1), yb(2), 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.18);
yline(ax, mean(yb), ':', 'E*', 'Color', [0.45 0.45 0.45]);
lh = [h, hband];  ltxt = [g.laws, {'target band'}];
if isgraphics(hov)
    lh = [lh, hov, hgh];
    ltxt = [ltxt, {'human at cap (effort not comparable)', 'effort demanded'}];
end
legend(lh, ltxt, 'Location', 'northwest', 'Box', 'off', 'FontSize', 7);
xlabel(ax, 'task difficulty D (N m s, a priori)');
ylabel(ax, 'measured E_{human} over the carry (N m s)');
title(ax, 'Human effort vs the target band');

% (2) The cost of overload: tracking error
ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
for L = 1:nl
    R = 1e3 * g.rmse_ee(:, :, L);
    plot(ax, Dv, R(order), '-o', 'Color', col_ctrl(L,:), ...
        'MarkerFaceColor', col_ctrl(L,:), 'MarkerSize', 5, 'LineWidth', 1.4);
    ov = g.overload(:, :, L);  ov = ov(order);  R = R(order);
    if any(ov)
        plot(ax, Dv(ov), R(ov), 'x', 'Color', [0.15 0.15 0.15], ...
            'MarkerSize', 11, 'LineWidth', 1.6);
    end
end
xlabel(ax, 'task difficulty D (N m s, a priori)');
ylabel(ax, 'end-effector RMSE (mm)');
title(ax, 'Accuracy: what over-demanding the human costs');

% (3) Pass rate (in band AND within strength), with raw in-band disclosed
ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
names = categorical(g.laws, g.laws);
b = bar(ax, names, 100 * g.pass, 0.55);
b.FaceColor = 'flat';  b.CData = col_ctrl;
hraw = plot(ax, names, 100 * g.coverage, 'd', 'Color', 'k', ...
    'MarkerFaceColor', 'w', 'MarkerSize', 7, 'LineStyle', 'none');
text(ax, 1:nl, 100 * g.pass + 4, compose('%.0f%%', 100 * g.pass), ...
    'HorizontalAlignment', 'center');
legend(hraw, {'in band only (ignores overload)'}, 'Location', 'northeast', ...
    'Box', 'off', 'FontSize', 7);
ylim(ax, [0 118]);
ylabel(ax, 'tasks passed: in band AND within strength (%)');
title(ax, 'Coverage');

exportgraphics(fig, fullfile(outdir, 'closed_loop_grid.png'), 'Resolution', 200);
fprintf('Wrote %s\n', fullfile(outdir, 'closed_loop_grid.png'));
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

% ------------------------------------------------------------------------
function fig = new_fig(sz)
vis = 'off';
if usejava('desktop'), vis = 'on'; end
fig = figure('Visible', vis, 'Color', [252 252 251]/255, 'Position', [80 80 sz]);
fig.Theme = 'light';
end
