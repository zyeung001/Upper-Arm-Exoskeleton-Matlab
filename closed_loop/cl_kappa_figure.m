function cl_kappa_figure(s, outdir)
%CL_KAPPA_FIGURE Strength-sweep figure from a run_kappa_sweep result.
%   CL_KAPPA_FIGURE(s, outdir) with s from run_kappa_sweep. Standalone so
%   the figure can be regenerated from results/closed_loop_kappa.mat
%   without re-running the 375 simulations.
if ~exist(outdir, 'dir'), mkdir(outdir); end
col_ctrl = [0 114 178; 230 159 0; 0 158 115] / 255;
nl = numel(s.laws);
kp = 100 * s.kappas;

fig = new_fig([1320 420]);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf(['How weak can the user be before each assistance law ' ...
    'fails them? (closed loop, %dx%d task grid at each strength)'], ...
    numel(s.cl.grid_f), numel(s.cl.grid_d)));

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
