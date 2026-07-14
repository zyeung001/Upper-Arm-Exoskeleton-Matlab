function s = run_kappa_sweep(opts)
%RUN_KAPPA_SWEEP How weak can the user be before each law fails them?
%   s = RUN_KAPPA_SWEEP            sweeps cl_params.kappa_sweep
%   s = RUN_KAPPA_SWEEP(Kappas=[0.06 0.08 0.10])
%
%   The closed-loop grid, repeated at several human strength fractions
%   kappa (the user's peak joint torque as a fraction of healthy MVC). This
%   is the HEADLINE strength-cap result, and it is deliberately a curve
%   rather than a number: no single kappa is privileged, so the comparison
%   cannot be accused of having picked the one value that flatters the
%   adopted law.
%
%   Predicted a priori from CALIBRATION-grid inverse dynamics alone (no
%   simulation, no evaluation data - see cl_params.m), each law has a
%   CRITICAL KAPPA: the weakest user it never over-demands on any task.
%       fixed 9.1% MVC | fill 8.5% | difficulty 6.4%
%   A user below a law's critical kappa gets asked for torque they do not
%   have; the shortfall is not delivered, the applied torque falls short of
%   what the carry needs, and tracking degrades. So the difficulty law
%   should serve users roughly 30% weaker than the fixed law can, and the
%   pass-rate curves should separate in that window. This sweep tests that
%   prediction on the held-out evaluation grid.
%
%   Figure -> results/figures/closed_loop_kappa.png
%   Data   -> results/closed_loop_kappa.mat

arguments
    opts.Kappas (1,:) double = []
    opts.Figures (1,1) logical = true
end

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(root, fullfile(root, 'generated'), here);
cl = cl_params();
kappas = opts.Kappas;
if isempty(kappas), kappas = cl.kappa_sweep; end
nk = numel(kappas);

s.kappas = kappas;
s.laws = {'fixed', 'fill', 'difficulty'};
nl = numel(s.laws);

% The prediction, computed here rather than quoted: each law's critical
% kappa, from calibration-grid inverse dynamics alone (see cl_critical_kappa).
p = calibrate_controller(params());
[s.kcrit, s.pk_demand] = cl_critical_kappa(p, cl, s.laws);
s.pass     = zeros(nk, nl);   % in band AND within strength
s.coverage = zeros(nk, nl);   % in band only (what we would have reported blind)
s.n_over   = zeros(nk, nl);   % tasks where the human was over-demanded
s.rmse     = zeros(nk, nl);   % mean end-effector RMSE (mm)

fprintf('\nStrength sweep: %d values of kappa x %d tasks x %d laws = %d sims.\n', ...
    nk, numel(cl.grid_f) * numel(cl.grid_d), nl, ...
    nk * numel(cl.grid_f) * numel(cl.grid_d) * nl);
fprintf(['  A-priori prediction (calibration grid, inverse dynamics only) - ' ...
    'critical kappa,\n  the weakest user each law never over-demands:\n']);
for L = 1:nl
    fprintf('    %-12s %.1f%% MVC   (worst demand [%.1f %.1f %.1f] N m)\n', ...
        s.laws{L}, 100 * s.kcrit(L), s.pk_demand(:, L));
end
for k = 1:nk
    fprintf('\n--- kappa = %.1f%% MVC ---\n', 100 * kappas(k));
    g = run_closed_loop_grid(Kappa=kappas(k), Figures=false);
    s.pass(k, :)     = g.pass;
    s.coverage(k, :) = g.coverage;
    for L = 1:nl
        s.n_over(k, L) = nnz(g.overload(:, :, L));
        s.rmse(k, L)   = 1e3 * mean(g.rmse_ee(:, :, L), 'all');
    end
    s.grids(k) = g; %#ok<AGROW>
end

hdr = sprintf('%-12s', s.laws{:});
fprintf('\n  Pass rate (in band AND within strength), %% of tasks:\n');
fprintf('    %-9s %s\n', 'kappa', hdr);
for k = 1:nk
    row = '';
    for L = 1:nl
        row = [row sprintf('%-12s', sprintf('%.0f%%', 100 * s.pass(k, L)))]; %#ok<AGROW>
    end
    fprintf('    %-9s %s\n', sprintf('%.1f%%', 100 * kappas(k)), row);
end
fprintf('\n  Tasks where the human was over-demanded (of %d):\n', ...
    numel(cl.grid_f) * numel(cl.grid_d));
fprintf('    %-9s %s\n', 'kappa', hdr);
for k = 1:nk
    row = '';
    for L = 1:nl
        row = [row sprintf('%-12d', s.n_over(k, L))]; %#ok<AGROW>
    end
    fprintf('    %-9s %s\n', sprintf('%.1f%%', 100 * kappas(k)), row);
end
fprintf('\n');

s.cl = cl;
resdir = fullfile(root, 'results');
if ~exist(resdir, 'dir'), mkdir(resdir); end
save(fullfile(resdir, 'closed_loop_kappa.mat'), 's');
fprintf('Saved %s\n', fullfile(resdir, 'closed_loop_kappa.mat'));

if opts.Figures
    kappa_figure(s, fullfile(resdir, 'figures'));
end
end

% ------------------------------------------------------------------------
function kappa_figure(s, outdir)
if ~exist(outdir, 'dir'), mkdir(outdir); end
col_ctrl = [0 114 178; 230 159 0; 0 158 115] / 255;
nl = numel(s.laws);
kp = 100 * s.kappas;

fig = new_fig([1320 420]);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, ['How weak can the user be before each assistance law fails them? ' ...
    '(closed loop, 3x3 task grid at each strength)']);

ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
for L = 1:nl
    plot(ax, kp, 100 * s.pass(:, L), '-o', 'Color', col_ctrl(L,:), ...
        'MarkerFaceColor', col_ctrl(L,:), 'MarkerSize', 5, 'LineWidth', 1.6);
end
legend(ax, s.laws, 'Location', 'southeast', 'Box', 'off');
xlabel(ax, 'user strength \kappa (% of healthy MVC)');
ylabel(ax, 'tasks passed: in band AND within strength (%)');
ylim(ax, [0 105]);
title(ax, 'Pass rate vs user strength');

ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
for L = 1:nl
    plot(ax, kp, s.n_over(:, L), '-o', 'Color', col_ctrl(L,:), ...
        'MarkerFaceColor', col_ctrl(L,:), 'MarkerSize', 5, 'LineWidth', 1.6);
end
% The a-priori prediction, drawn over the measured curves: right of a law's
% critical kappa it should never over-demand the user (curve hits zero).
for L = 1:nl
    xline(ax, 100 * s.kcrit(L), '--', sprintf('%s: %.1f%%', s.laws{L}, ...
        100 * s.kcrit(L)), 'Color', col_ctrl(L,:), 'FontSize', 7, ...
        'LabelVerticalAlignment', 'top', 'LabelHorizontalAlignment', 'left');
end
xlabel(ax, 'user strength \kappa (% of healthy MVC)');
ylabel(ax, 'tasks demanding more torque than the user has');
title(ax, 'Over-demanded tasks vs the a-priori critical \kappa (dashed)');

ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
for L = 1:nl
    plot(ax, kp, s.rmse(:, L), '-o', 'Color', col_ctrl(L,:), ...
        'MarkerFaceColor', col_ctrl(L,:), 'MarkerSize', 5, 'LineWidth', 1.6);
end
xlabel(ax, 'user strength \kappa (% of healthy MVC)');
ylabel(ax, 'mean end-effector RMSE (mm)');
title(ax, 'The task cost of over-demanding the user');

exportgraphics(fig, fullfile(outdir, 'closed_loop_kappa.png'), 'Resolution', 200);
fprintf('Wrote %s\n', fullfile(outdir, 'closed_loop_kappa.png'));
end

% ------------------------------------------------------------------------
function fig = new_fig(sz)
vis = 'off';
if usejava('desktop'), vis = 'on'; end
fig = figure('Visible', vis, 'Color', [252 252 251]/255, 'Position', [80 80 sz]);
fig.Theme = 'light';
end
