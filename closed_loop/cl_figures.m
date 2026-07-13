function cl_figures(res, outdir)
%CL_FIGURES Closed-loop validation figures (2D only, sweep conventions).
%   CL_FIGURES(res, outdir) with res from run_closed_loop. Saves:
%     closed_loop_controllers.png  THE comparison: the three alpha laws
%                                  scored on the measured closed-loop
%                                  torque - cumulative human effort vs the
%                                  target band, final effort vs the
%                                  open-loop prediction, and the
%                                  instantaneous human-torque demand.
%     closed_loop_tracking.png     context: tracking error + slosh angle
%                                  (law-independent - all laws split the
%                                  same total torque).

if ~exist(outdir, 'dir'), mkdir(outdir); end
col_ctrl = [0 114 178; 230 159 0; 0 158 115] / 255;   % fixed / fill / difficulty
col_gray = [0.4 0.4 0.4];
T_move = res.p.sim.T_move;
nl  = numel(res.laws);
win = res.t <= T_move;                 % carry window (band-comparable)
tw  = res.t(win);
yb  = [res.p.band.E_low, res.p.band.E_high];

% ================= Controller comparison (the study's question) ==============
fig = new_fig([1180 400]);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf(['Three assistance laws on the closed-loop measured torque, ' ...
    'fill = %.2f, d = %.2f m  (D = %.2f N m s, \\alpha: %.2f / %.2f / %.2f)'], ...
    res.f, res.d, res.D, res.alpha));

% (1) Cumulative human effort over the carry vs the target band
ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
h = gobjects(1, nl);
for L = 1:nl
    h(L) = plot(ax, tw, res.E_cum(L, win), '-', 'Color', col_ctrl(L,:), ...
        'LineWidth', 1.8);
    plot(ax, tw(end), res.law(L).E_human, 'o', 'Color', col_ctrl(L,:), ...
        'MarkerFaceColor', col_ctrl(L,:), 'MarkerSize', 5);
end
hband = yregion(ax, yb(1), yb(2), 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.18);
legend([h, hband], [res.laws, {'target band'}], 'Location', 'northwest', 'Box', 'off');
xlabel(ax, 't (s)');  ylabel(ax, 'E_{human}(t) (N m s)');
title(ax, 'Human effort accumulating - who ends in the band?');

% (2) Final human effort vs the open-loop prediction (1-\alpha) D
ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
names = categorical(res.laws, res.laws);
b = bar(ax, names, [res.law.E_human], 0.55);
b.FaceColor = 'flat';  b.CData = col_ctrl;
hol = plot(ax, names, (1 - res.alpha) * res.D, 'd', 'Color', 'k', ...
    'MarkerFaceColor', 'w', 'MarkerSize', 7, 'LineStyle', 'none');
hband = yregion(ax, yb(1), yb(2), 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.18);
legend([hol, hband], {'open-loop (1-\alpha)D', 'target band'}, ...
    'Location', 'northeast', 'Box', 'off');
ylabel(ax, 'E_{human} over the carry (N m s)');
title(ax, 'Closed loop reproduces the sweep''s effort');

% (3) Instantaneous human torque demand
ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
hT = plot(ax, tw, sum(abs(res.u(:, win)), 1), '-', 'Color', col_gray, ...
    'LineWidth', 1.2);
h = gobjects(1, nl);
for L = 1:nl
    h(L) = plot(ax, tw, (1 - res.alpha(L)) * sum(abs(res.u(:, win)), 1), ...
        '-', 'Color', col_ctrl(L,:), 'LineWidth', 1.5);
end
legend([hT, h], [{'total demand'}, res.laws], 'Location', 'northwest', 'Box', 'off');
xlabel(ax, 't (s)');  ylabel(ax, '\Sigma_j |\tau_{human,j}| (N m)');
title(ax, 'Momentary demand on the human');

exportgraphics(fig, fullfile(outdir, 'closed_loop_controllers.png'), 'Resolution', 200);

% ================= Tracking context (law-independent) ========================
fig = new_fig([880 340]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf(['Closed-loop tracking, fill = %.2f, d = %.2f m ' ...
    '(PD + gravity comp; identical for all three laws)'], res.f, res.d));

ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
m = res.metrics;
plot(ax, res.t, 1e3 * m.ee_err, '-', 'Color', [0 114 178]/255, 'LineWidth', 1.5);
yline(ax, 1e3 * m.rmse_ee, ':', sprintf('RMSE = %.1f mm', 1e3 * m.rmse_ee), ...
    'Color', [0 114 178]/255, 'LabelHorizontalAlignment', 'right');
xline(ax, T_move, ':', 'carry ends', 'Color', col_gray, ...
    'LabelVerticalAlignment', 'top', 'FontSize', 8);
xlabel(ax, 't (s)');  ylabel(ax, '||ee - ee_{ref}|| (mm)');
title(ax, 'End-effector tracking error');

ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');
plot(ax, res.t, rad2deg(res.phi), '-', 'Color', col_gray, 'LineWidth', 1.2);
xline(ax, T_move, ':', 'Color', col_gray);
xlabel(ax, 't (s)');  ylabel(ax, '\phi (deg)');
title(ax, sprintf('Slosh excited by the carry (peak %.1f%s)', ...
    rad2deg(m.peak_slosh), char(176)));

exportgraphics(fig, fullfile(outdir, 'closed_loop_tracking.png'), 'Resolution', 200);
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
