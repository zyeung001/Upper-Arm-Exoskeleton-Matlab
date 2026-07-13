function cl_figures(res, outdir)
%CL_FIGURES Closed-loop comparison figures (2D only, sweep conventions).
%   CL_FIGURES(res, outdir) with res from run_closed_loop. Saves:
%     closed_loop_controllers.png  THE comparison: with the human's
%                                  torque-development lag, the alpha law
%                                  changes the physics - tracking error,
%                                  human effort vs band, and the
%                                  whiteboard metrics per law.
%     closed_loop_validation.png   ideal-human cross-check: closed-loop
%                                  effort reproduces the open-loop
%                                  sweep's (1-alpha)*D.

if ~exist(outdir, 'dir'), mkdir(outdir); end
col_ctrl = [0 114 178; 230 159 0; 0 158 115] / 255;   % fixed / fill / difficulty
col_gray = [0.4 0.4 0.4];
T_move = res.p.sim.T_move;
nl = numel(res.laws);
yb = [res.p.band.E_low, res.p.band.E_high];

% ================= Controller comparison (the study's question) ==============
fig = new_fig([1180 700]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf(['Three assistance laws in closed loop with human torque lag ' ...
    '(\\tau_h = %.0f ms), fill = %.2f, d = %.2f m  (D = %.2f N m s, ' ...
    '\\alpha: %.2f / %.2f / %.2f)'], 1e3 * res.cl.tau_h, res.f, res.d, ...
    res.D, res.alpha));

% (1) Tracking error per law - the lag makes alpha matter dynamically
ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
hi = plot(ax, res.ideal.t, 1e3 * res.ideal.metrics.ee_err, '--', ...
    'Color', [0.6 0.6 0.65], 'LineWidth', 1.0);
h = gobjects(1, nl);
leg = cell(1, nl);
for L = 1:nl
    r = res.runs(L);
    h(L) = plot(ax, r.t, 1e3 * r.metrics.ee_err, '-', ...
        'Color', col_ctrl(L,:), 'LineWidth', 1.5);
    leg{L} = sprintf('%s (RMSE %.1f mm)', r.name, 1e3 * r.metrics.rmse_ee);
end
xline(ax, T_move, ':', 'carry ends', 'Color', col_gray, ...
    'LabelVerticalAlignment', 'top', 'FontSize', 8);
legend([h, hi], [leg, {sprintf('ideal human (%.1f mm)', ...
    1e3 * res.ideal.metrics.rmse_ee)}], 'Location', 'northeast', 'Box', 'off');
xlabel(ax, 't (s)');  ylabel(ax, '||ee - ee_{ref}|| (mm)');
title(ax, 'Accuracy: more human share = slightly worse tracking');

% (2) Cumulative human effort (as delivered) vs the target band
ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
h = gobjects(1, nl);
for L = 1:nl
    r = res.runs(L);
    win = r.t <= T_move;
    h(L) = plot(ax, r.t(win), r.E_cum(win), '-', 'Color', col_ctrl(L,:), ...
        'LineWidth', 1.8);
    plot(ax, T_move, r.band.E_human, 'o', 'Color', col_ctrl(L,:), ...
        'MarkerFaceColor', col_ctrl(L,:), 'MarkerSize', 5);
end
hband = yregion(ax, yb(1), yb(2), 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.18);
legend([h, hband], [res.laws, {'target band'}], 'Location', 'northwest', 'Box', 'off');
xlabel(ax, 't (s)');  ylabel(ax, 'E_{human}(t) (N m s)');
title(ax, 'Human effort accumulating - who ends in the band?');

% (3) Final human effort per law vs band
ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
names = categorical(res.laws, res.laws);
b = bar(ax, names, [arrayfun(@(L) res.runs(L).band.E_human, 1:nl)], 0.55);
b.FaceColor = 'flat';  b.CData = col_ctrl;
hband = yregion(ax, yb(1), yb(2), 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.18);
legend(hband, {'target band'}, 'Location', 'northeast', 'Box', 'off');
ylabel(ax, 'E_{human} over the carry (N m s)');
title(ax, 'Delivered human effort vs band');

% (4) Whiteboard metrics per law, relative to the fixed law
ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
met = zeros(3, nl);
for L = 1:nl
    m = res.runs(L).metrics;
    met(:, L) = [m.rmse_ee; m.t_settle; m.energy_u];
end
met = met ./ met(:, 1);   % normalize to the fixed law
mnames = categorical({'accuracy (RMSE)', 'time (settle)', 'energy of u'}, ...
    {'accuracy (RMSE)', 'time (settle)', 'energy of u'});
b = bar(ax, mnames, met);
for L = 1:nl, b(L).FaceColor = col_ctrl(L, :); end
yline(ax, 1, ':', 'Color', col_gray);
legend(b, res.laws, 'Location', 'northeast', 'Box', 'off');
ylabel(ax, 'relative to fixed law');
title(ax, 'Task metrics barely move - the law changes who works');

exportgraphics(fig, fullfile(outdir, 'closed_loop_controllers.png'), 'Resolution', 200);

% ================= Ideal-human validation cross-check ========================
fig = new_fig([1180 360]);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf(['Ideal-human validation run, fill = %.2f, d = %.2f m: ' ...
    'the closed loop reproduces the open-loop sweep'], res.f, res.d));

ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
m = res.ideal.metrics;
plot(ax, res.ideal.t, 1e3 * m.ee_err, '-', 'Color', [0 114 178]/255, ...
    'LineWidth', 1.5);
yline(ax, 1e3 * m.rmse_ee, ':', sprintf('RMSE = %.1f mm', 1e3 * m.rmse_ee), ...
    'Color', [0 114 178]/255, 'LabelHorizontalAlignment', 'right');
xline(ax, T_move, ':', 'carry ends', 'Color', col_gray, ...
    'LabelVerticalAlignment', 'top', 'FontSize', 8);
xlabel(ax, 't (s)');  ylabel(ax, '||ee - ee_{ref}|| (mm)');
title(ax, 'Tracking error');

ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
plot(ax, res.ideal.t, rad2deg(res.ideal.phi), '-', 'Color', col_gray, ...
    'LineWidth', 1.2);
xline(ax, T_move, ':', 'Color', col_gray);
xlabel(ax, 't (s)');  ylabel(ax, '\phi (deg)');
title(ax, sprintf('Slosh excited by the carry (peak %.1f%s)', ...
    rad2deg(m.peak_slosh), char(176)));

ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
names = categorical(res.laws, res.laws);
b = bar(ax, names, [res.ideal.band.E_human], 0.55);
b.FaceColor = 'flat';  b.CData = col_ctrl;
hol = plot(ax, names, (1 - res.alpha) * res.D, 'd', 'Color', 'k', ...
    'MarkerFaceColor', 'w', 'MarkerSize', 7, 'LineStyle', 'none');
hband = yregion(ax, yb(1), yb(2), 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.18);
legend([hol, hband], {'open-loop (1-\alpha)D', 'target band'}, ...
    'Location', 'northeast', 'Box', 'off');
ylabel(ax, 'E_{human} over the carry (N m s)');
title(ax, 'Effort matches the sweep');

exportgraphics(fig, fullfile(outdir, 'closed_loop_validation.png'), 'Resolution', 200);
fprintf('Wrote closed-loop figures to %s\n', outdir);
end

% ------------------------------------------------------------------------
function fig = new_fig(sz)
% Same convention as make_figures.m: pop up on a desktop, hidden in -batch.
vis = 'off';
if usejava('desktop'), vis = 'on'; end
fig = figure('Visible', vis, 'Color', [252 252 251]/255, 'Position', [80 80 sz]);
fig.Theme = 'light';
end
