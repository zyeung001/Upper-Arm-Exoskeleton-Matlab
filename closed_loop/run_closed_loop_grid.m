function g = run_closed_loop_grid(opts)
%RUN_CLOSED_LOOP_GRID Closed-loop band coverage across the task grid.
%   g = RUN_CLOSED_LOOP_GRID          3x3 grid from cl_params (extremes)
%   g = RUN_CLOSED_LOOP_GRID(Rebuild=true)
%
%   The study's primary question, asked in closed loop: over a grid of
%   tasks spanning the sweep's extremes, which alpha law keeps the MEASURED
%   human effort inside the target band? The three laws coincide on
%   mid-difficulty tasks by construction - their differences live at the
%   corners (fixed over-assists easy carries and under-supports hard ones;
%   the difficulty law holds effort at the band midpoint everywhere) - so
%   this grid, not a single task, is the closed-loop analogue of the
%   sweep's coverage headline.
%
%   One simulation per task per law (human torque lag active, slosh excited
%   by the carry), 27 sims total by default; takes a few minutes. alpha is
%   computed A PRIORI per task via controllers.m, as everywhere else.
%
%   Figure -> results/figures/closed_loop_grid.png
%   Data   -> results/closed_loop_grid.mat

arguments
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
laws = {'fixed', 'fill', 'difficulty'};
nl = numel(laws);
nf = numel(cl.grid_f);
nd = numel(cl.grid_d);

mdl = 'arm_closed_loop';
mdlfile = fullfile(here, [mdl '.slx']);
if ~exist(mdlfile, 'file') || opts.Rebuild
    build_closed_loop_model();
end

L_s = p.slosh.L_s_factor * p.cont.Rc;
w_s = sqrt(p.g / L_s);

g.f = cl.grid_f;  g.d = cl.grid_d;  g.laws = laws;
g.D       = zeros(nf, nd);
g.alpha   = zeros(nf, nd, nl);
g.E_human = zeros(nf, nd, nl);
g.status  = zeros(nf, nd, nl);   % -1 below band / 0 in band / +1 above
g.rmse_ee = zeros(nf, nd, nl);

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
        base = base.setVariable('cl_ideal', 0);
        base = base.setVariable('cl_Tend',  cl.Tend_grid);

        for L = 1:nl
            [~, ~, a] = controllers(laws{L}, tau_total, f, D, p);
            g.alpha(i, j, L) = a;
            in = base.setVariable('cl_alpha', a);
            in = in.setVariable('cl_uh0', (1 - a) * G0(1:3));
            out = sim(in);

            t    = out.y.Time.';
            uhum = out.uhum.Data.';
            uexo = out.uexo.Data.';
            win  = t <= p.sim.T_move;
            mb = compute_metrics(t(win), uhum(:, win), uexo(:, win), p);
            g.E_human(i, j, L) = mb.E_human;
            g.status(i, j, L)  = mb.status;
            mt = cl_metrics(t, out.y.Data.', out.err.Data.', out.u.Data.', ...
                out.phi.Data(:).', tt, qref, pvec, cl);
            g.rmse_ee(i, j, L) = mt.rmse_ee;
        end
        fprintf('  f = %.2f, d = %.2f: D = %5.2f | E_human %s\n', f, d, D, ...
            strjoin(compose('%.2f', squeeze(g.E_human(i, j, :)).'), ' / '));
    end
end

% ---- Console summary ---------------------------------------------------------
n_tasks = nf * nd;
fprintf('\n  Band coverage (closed loop, measured effort), band = [%.2f, %.2f]:\n', ...
    p.band.E_low, p.band.E_high);
g.coverage = zeros(1, nl);
for L = 1:nl
    s = g.status(:, :, L);
    g.coverage(L) = nnz(s == 0) / n_tasks;
    fprintf('    %-12s %d/%d in band (%2.0f%%)  [below: %d, above: %d]  mean RMSE %.1f mm\n', ...
        laws{L}, nnz(s == 0), n_tasks, 100 * g.coverage(L), ...
        nnz(s < 0), nnz(s > 0), 1e3 * mean(g.rmse_ee(:, :, L), 'all'));
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
% Measured human effort vs task difficulty, per law, band shaded - the
% closed-loop analogue of the sweep's coverage figure.
if ~exist(outdir, 'dir'), mkdir(outdir); end
col_ctrl = [0 114 178; 230 159 0; 0 158 115] / 255;
nl = numel(g.laws);
yb = [g.p.band.E_low, g.p.band.E_high];
[Dv, order] = sort(g.D(:));

fig = new_fig([980 420]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf(['Closed-loop band coverage across the task grid ' ...
    '(%dx%d tasks, human torque lag %.0f ms)'], ...
    numel(g.f), numel(g.d), 1e3 * g.cl.tau_h));

ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
h = gobjects(1, nl);
for L = 1:nl
    E = g.E_human(:, :, L);
    h(L) = plot(ax, Dv, E(order), '-o', 'Color', col_ctrl(L,:), ...
        'MarkerFaceColor', col_ctrl(L,:), 'MarkerSize', 5, 'LineWidth', 1.4);
end
hband = yregion(ax, yb(1), yb(2), 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.18);
yline(ax, mean(yb), ':', 'E*', 'Color', [0.45 0.45 0.45]);
legend([h, hband], [g.laws, {'target band'}], 'Location', 'northwest', 'Box', 'off');
xlabel(ax, 'task difficulty D (N m s, a priori)');
ylabel(ax, 'measured E_{human} over the carry (N m s)');
title(ax, 'Fixed climbs through the band; difficulty holds E*');

ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
names = categorical(g.laws, g.laws);
b = bar(ax, names, 100 * g.coverage, 0.55);
b.FaceColor = 'flat';  b.CData = col_ctrl;
text(ax, 1:nl, 100 * g.coverage + 3, ...
    compose('%.0f%%', 100 * g.coverage), 'HorizontalAlignment', 'center');
ylim(ax, [0 115]);
ylabel(ax, 'tasks with human effort in band (%)');
title(ax, 'Band coverage (closed loop)');

exportgraphics(fig, fullfile(outdir, 'closed_loop_grid.png'), 'Resolution', 200);
fprintf('Wrote %s\n', fullfile(outdir, 'closed_loop_grid.png'));
end

% ------------------------------------------------------------------------
function fig = new_fig(sz)
vis = 'off';
if usejava('desktop'), vis = 'on'; end
fig = figure('Visible', vis, 'Color', [252 252 251]/255, 'Position', [80 80 sz]);
fig.Theme = 'light';
end
