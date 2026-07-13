function animate_closed_loop(res, opts)
%ANIMATE_CLOSED_LOOP Animate the closed-loop run (whiteboard "Animate Function").
%   ANIMATE_CLOSED_LOOP            loads results/closed_loop_results.mat
%   ANIMATE_CLOSED_LOOP(res)       res from run_closed_loop
%   ANIMATE_CLOSED_LOOP(res, Speed=0.5, Gif=true)
%
%   Consumes exactly what the whiteboard pipes out of sim(): out.y (the
%   SIMULATED joint angles - unlike animate_carry, which replays the
%   prescribed reference), out.err, out.time. Shows:
%     LEFT   the arm as simulated (solid) vs the reference pose (ghost),
%            carrying the cup; liquid surface tilts with the slosh phi.
%     RIGHT  end-effector tracking error and slosh angle vs time, with a
%            playback cursor.

arguments
    res = []
    opts.Speed (1,1) double {mustBePositive} = 0.5
    opts.Gif (1,1) logical = false
end

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(root, fullfile(root, 'generated'), here);
if isempty(res)
    matfile = fullfile(root, 'results', 'closed_loop_results.mat');
    assert(exist(matfile, 'file') == 2, ...
        'animate_closed_loop: run run_closed_loop first (missing %s)', matfile);
    S = load(matfile);  res = S.res;
end
ca = res;    % single run: signals live directly on res
p  = res.p;

% Resample to ~120 uniform frames
nfr = 120;
tf  = linspace(ca.t(1), ca.t(end), nfr);
q    = interp1(ca.t.', ca.q.',   tf.').';
phi  = interp1(ca.t.', ca.phi.', tf.').';
eerr = interp1(ca.t.', ca.metrics.ee_err.', tf.').';
qr   = interp1(res.tt.', res.qref.', tf.').';

[sh,  el,  ee ] = fk_points(q,  p);
[shr, elr, eer] = fk_points(qr, p);

Rc = p.cont.Rc;  Hc = p.cont.Hc;  hf = res.f * Hc;
uu = linspace(0, 2*pi, 25);  cs = cos(uu);  sn = sin(uu);
col_gray = [0.4 0.4 0.4];  col_liq = [86 180 233]/255;  col_err = [213 94 0]/255;

fig = figure('Color', 'w', 'Position', [60 60 1150 640], ...
    'Name', sprintf('Closed loop: f = %.2f, d = %.2f m', res.f, res.d));
fig.Theme = 'light';
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf(['Closed-loop carry, f = %.2f, d = %.2f m   |   ' ...
    'RMSE = %.1f mm, peak slosh = %.1f%s'], res.f, res.d, ...
    1e3*ca.metrics.rmse_ee, rad2deg(ca.metrics.peak_slosh), char(176)));

% Left: 3D arm, simulated vs reference ghost
ax1 = nexttile(tl, 1, [2 1]);
hold(ax1, 'on');  grid(ax1, 'on');  daspect(ax1, [1 1 1]);
view(ax1, -40, 18);
xlabel(ax1, 'x (m)'); ylabel(ax1, 'y (m)'); zlabel(ax1, 'z (m)');
lim = p.arm.L3 + p.arm.L1 + p.arm.L2;
xlim(ax1, [-0.1 lim]); ylim(ax1, [-0.35 0.55]); zlim(ax1, [-0.75 0.55]);
plot3(ax1, [0 0], [0 0], [-0.75 0.15], 'k-', 'LineWidth', 3);
plot3(ax1, eer(1,:), eer(2,:), eer(3,:), ':', 'Color', col_gray);
h_ref = plot3(ax1, nan, nan, nan, '--o', 'LineWidth', 1.2, ...
    'Color', [0.65 0.65 0.7], 'MarkerSize', 4);
h_arm = plot3(ax1, nan, nan, nan, '-o', 'LineWidth', 3.5, ...
    'Color', [0.25 0.25 0.3], 'MarkerFaceColor', 'w', 'MarkerSize', 5);
h_cupb = plot3(ax1, nan, nan, nan, '-', 'Color', col_gray);
h_cupt = plot3(ax1, nan, nan, nan, '-', 'Color', col_gray);
h_liq  = patch(ax1, nan, nan, nan, col_liq, 'FaceAlpha', 0.75, ...
    'EdgeColor', col_liq);
legend(ax1, [h_arm, h_ref], {'simulated', 'reference'}, ...
    'Location', 'northeast', 'Box', 'off');
h_time = title(ax1, 't = 0.00 s');

% Right top: tracking error
ax2 = nexttile(tl, 2);
hold(ax2, 'on');  grid(ax2, 'on');
plot(ax2, ca.t, 1e3 * ca.metrics.ee_err, '-', 'Color', col_err, 'LineWidth', 1.2);
yline(ax2, 1e3 * ca.metrics.rmse_ee, ':', 'RMSE', 'Color', col_err);
xline(ax2, p.sim.T_move, ':', 'carry ends', 'Color', col_gray, 'FontSize', 8);
h_cur2 = xline(ax2, 0, '-', 'Color', col_liq, 'LineWidth', 1);
ylabel(ax2, '||ee err|| (mm)');
title(ax2, 'Tracking error (out.err)');

% Right bottom: slosh angle
ax3 = nexttile(tl, 4);
hold(ax3, 'on');  grid(ax3, 'on');
plot(ax3, ca.t, rad2deg(ca.phi), '-', 'Color', col_gray);
h_cur3 = xline(ax3, 0, '-', 'Color', col_liq, 'LineWidth', 1);
xlabel(ax3, 't (s)');  ylabel(ax3, '\phi (deg)');
title(ax3, 'Slosh angle');

% ---- Playback ----------------------------------------------------------------
if opts.Gif
    gifdir = fullfile(root, 'results', 'figures');
    if ~exist(gifdir, 'dir'), mkdir(gifdir); end
    giffile = fullfile(gifdir, 'closed_loop_carry.gif');
    if exist(giffile, 'file'), delete(giffile); end
end
zhat = [0; 0; 1];
for k = 1:nfr
    if ~ishandle(fig), return; end
    th3 = q(3, k);
    er = [cos(th3); sin(th3); 0];
    et = [-sin(th3); cos(th3); 0];
    P  = ee(:, k);
    set(h_arm, 'XData', [0, sh(1,k), el(1,k), P(1)], ...
               'YData', [0, sh(2,k), el(2,k), P(2)], ...
               'ZData', [0, sh(3,k), el(3,k), P(3)]);
    set(h_ref, 'XData', [0, shr(1,k), elr(1,k), eer(1,k)], ...
               'YData', [0, shr(2,k), elr(2,k), eer(2,k)], ...
               'ZData', [0, shr(3,k), elr(3,k), eer(3,k)]);
    ringb = P + Rc*(er*cs + et*sn);
    ringt = ringb + Hc*zhat;
    set(h_cupb, 'XData', ringb(1,:), 'YData', ringb(2,:), 'ZData', ringb(3,:));
    set(h_cupt, 'XData', ringt(1,:), 'YData', ringt(2,:), 'ZData', ringt(3,:));
    a1 = cos(phi(k))*er + sin(phi(k))*zhat;
    surfpts = P + hf*zhat + Rc*(a1*cs + et*sn);
    set(h_liq, 'XData', surfpts(1,:), 'YData', surfpts(2,:), 'ZData', surfpts(3,:));
    h_time.String = sprintf('t = %.2f s   (err = %.1f mm)', tf(k), 1e3*eerr(k));
    h_cur2.Value = tf(k);  h_cur3.Value = tf(k);
    drawnow;
    if opts.Gif, exportgraphics(fig, giffile, 'Append', true); end
    pause((tf(2) - tf(1)) / opts.Speed);
end
if opts.Gif, fprintf('Wrote %s\n', giffile); end
end

% ------------------------------------------------------------------------
function [shoulder, elbow, ee] = fk_points(q, p)
% Same geometry as animate_carry.m / derive_dynamics.
a = p.arm;
c3 = cos(q(3,:));  s3 = sin(q(3,:));
c1 = cos(q(1,:));  s1 = sin(q(1,:));
c12 = cos(q(1,:) + q(2,:));  s12 = sin(q(1,:) + q(2,:));
shoulder = a.L3 * [c3; s3; zeros(1, size(q, 2))];
elbow    = shoulder + a.L1 * [c3.*c1; s3.*c1; s1];
ee       = elbow    + a.L2 * [c3.*c12; s3.*c12; s12];
end
