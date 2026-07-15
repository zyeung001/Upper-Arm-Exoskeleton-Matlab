function cl_grid_figure(g, outdir)
%CL_GRID_FIGURE Closed-loop grid figure from a run_closed_loop_grid result.
%   CL_GRID_FIGURE(g, outdir) with g from run_closed_loop_grid (or one
%   slice of run_kappa_sweep's s.grids). Standalone so the figure can be
%   regenerated from results/closed_loop_grid.mat without re-simulating.
%
%   Three panels: who ends in the band (and who got there by overloading
%   the human), what that costs in tracking, and the resulting pass rate.
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
