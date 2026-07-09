function animate_carry(f, d, opts)
%ANIMATE_CARRY Animate one carry task: 3-DOF arm + sloshing container.
%   ANIMATE_CARRY            animates a fairly hard task (f = 0.8, d = 0.4)
%   ANIMATE_CARRY(f, d)      fill level f in [0,1], reach distance d (m)
%   ANIMATE_CARRY(f, d, Speed=0.5, Gif=true)
%     Speed  playback speed factor, 1 = real time (default 0.5, slow-mo)
%     Gif    also write results/figures/carry_f<f>_d<d>.gif (default false)
%
%   Left: the arm carrying the container, with the slosh pendulum drawn
%   exaggerated (x4) for visibility. Right: live slosh angle and the
%   torque split of the difficulty-adaptive controller for this task.
%
%   Run from the repo root (or with it on the path), e.g.:
%       animate_carry            % default hard-ish task
%       animate_carry(0.2, 0.1)  % easy corner
%       animate_carry(1.0, 0.5)  % hardest corner

arguments
    f (1,1) double {mustBeInRange(f, 0, 1)} = 0.8
    d (1,1) double {mustBePositive} = 0.4
    opts.Speed (1,1) double {mustBePositive} = 0.5
    opts.Gif (1,1) logical = false
end

root = fileparts(mfilename('fullpath'));
if exist(fullfile(root, 'generated', 'M_fun.m'), 'file') ~= 2
    derive_dynamics();
end
addpath(fullfile(root, 'generated'));

% ---- Simulate the task ----------------------------------------------------
p = params();
traj = make_trajectory(d, p.sim.T_move, p.sim.dt, p);
[tau_total, peak_slosh, slosh] = compute_required_torque(traj, f, p);
D = compute_difficulty(traj.t, tau_total);
[tau_exo, tau_human, alpha] = controllers('difficulty', tau_total, f, D, p);
t = traj.t;  N = numel(t);

% Joint positions along the trajectory (same geometry as derive_dynamics)
[shoulder, elbow, ee] = fk_points(traj.q, p);
Lvis = 4 * slosh.L_s;   % exaggerated pendulum length, display only
bob = ee + Lvis * [cos(traj.q(3,:)).*sin(slosh.phi); ...
                   sin(traj.q(3,:)).*sin(slosh.phi); ...
                   -cos(slosh.phi)];

% ---- Figure layout ---------------------------------------------------------
col_exo = [0 114 178]/255;  col_hum = [230 159 0]/255;  col_tot = [0.35 0.35 0.35];
fig = figure('Color', 'w', 'Position', [60 60 1150 520], ...
    'Name', sprintf('Carry: f = %.2f, d = %.2f m', f, d));
fig.Theme = 'light';

ax1 = subplot(1, 2, 1);
hold(ax1, 'on');  grid(ax1, 'on');  daspect(ax1, [1 1 1]);
view(ax1, -40, 18);
xlabel(ax1, 'x (m)'); ylabel(ax1, 'y (m)'); zlabel(ax1, 'z (m)');
lim = p.arm.L3 + p.arm.L1 + p.arm.L2;
xlim(ax1, [-0.1 lim]); ylim(ax1, [-0.35 0.55]); zlim(ax1, [-0.75 0.55]);
title(ax1, sprintf(['f = %.2f, d = %.2f m  |  D = %.2f N m s, ' ...
    '\\alpha = %.2f, peak |\\phi| = %.1f%s'], f, d, D, alpha, ...
    rad2deg(peak_slosh), char(176)));
plot3(ax1, [0 0], [0 0], [-0.75 0.15], 'k-', 'LineWidth', 3);    % torso axis
plot3(ax1, ee(1,:), ee(2,:), ee(3,:), ':', 'Color', col_tot);    % hand path
plot3(ax1, ee(1,[1 N]), ee(2,[1 N]), ee(3,[1 N]), 'o', ...
    'Color', col_tot, 'MarkerFaceColor', 'w');                   % start/target
h_arm = plot3(ax1, nan, nan, nan, '-o', 'LineWidth', 3.5, ...
    'Color', [0.25 0.25 0.3], 'MarkerFaceColor', 'w', 'MarkerSize', 5);
h_rod = plot3(ax1, nan, nan, nan, '-', 'LineWidth', 2, 'Color', col_exo);
h_bob = plot3(ax1, nan, nan, nan, 'o', 'MarkerSize', 11, ...
    'Color', col_exo, 'MarkerFaceColor', col_exo);
text(ax1, 0.02, 0.35, -0.7, 'slosh pendulum drawn \times4', 'FontSize', 8);

ax2 = subplot(2, 2, 2);
hold(ax2, 'on');  grid(ax2, 'on');
plot(ax2, t, rad2deg(slosh.phi), '-', 'Color', col_tot);
h_cur2 = xline(ax2, 0, '-', 'Color', col_exo, 'LineWidth', 1);
h_dot2 = plot(ax2, 0, 0, 'o', 'Color', col_exo, 'MarkerFaceColor', col_exo);
ylabel(ax2, '\phi (deg)');
title(ax2, 'Slosh angle');

ax3 = subplot(2, 2, 4);
hold(ax3, 'on');  grid(ax3, 'on');
s_tot = sum(abs(tau_total), 1);
plot(ax3, t, s_tot, '-', 'Color', col_tot, 'LineWidth', 1.5);
plot(ax3, t, sum(abs(tau_exo), 1), '-', 'Color', col_exo, 'LineWidth', 1.5);
plot(ax3, t, sum(abs(tau_human), 1), '-', 'Color', col_hum, 'LineWidth', 1.5);
h_cur3 = xline(ax3, 0, '-', 'Color', col_exo, 'LineWidth', 1);
legend(ax3, {'total', 'exo', 'human'}, 'Location', 'best', 'Box', 'off');
xlabel(ax3, 't (s)');  ylabel(ax3, '\Sigma_j |\tau_j| (N m)');
title(ax3, sprintf('Torque split (difficulty-adaptive, \\alpha = %.2f)', alpha));

% ---- Playback ---------------------------------------------------------------
if opts.Gif
    gifdir = fullfile(root, 'results', 'figures');
    if ~exist(gifdir, 'dir'), mkdir(gifdir); end
    giffile = fullfile(gifdir, sprintf('carry_f%03.0f_d%03.0f.gif', 100*f, 100*d));
    if exist(giffile, 'file'), delete(giffile); end
end
stride = max(1, round(N / 100));   % ~100 frames
for k = [1:stride:N, N]
    if ~ishandle(fig), return; end   % window closed mid-playback
    set(h_arm, 'XData', [0, shoulder(1,k), elbow(1,k), ee(1,k)], ...
               'YData', [0, shoulder(2,k), elbow(2,k), ee(2,k)], ...
               'ZData', [0, shoulder(3,k), elbow(3,k), ee(3,k)]);
    set(h_rod, 'XData', [ee(1,k), bob(1,k)], 'YData', [ee(2,k), bob(2,k)], ...
               'ZData', [ee(3,k), bob(3,k)]);
    set(h_bob, 'XData', bob(1,k), 'YData', bob(2,k), 'ZData', bob(3,k));
    h_cur2.Value = t(k);  h_cur3.Value = t(k);
    set(h_dot2, 'XData', t(k), 'YData', rad2deg(slosh.phi(k)));
    drawnow;
    if opts.Gif
        exportgraphics(fig, giffile, 'Append', true);
    end
    pause(stride * p.sim.dt / opts.Speed);
end
if opts.Gif
    fprintf('Wrote %s\n', giffile);
end
end

% ------------------------------------------------------------------------
function [shoulder, elbow, ee] = fk_points(q, p)
%Joint positions (3xN each) for plotting; mirrors derive_dynamics geometry.
a = p.arm;
c3 = cos(q(3,:));  s3 = sin(q(3,:));
c1 = cos(q(1,:));  s1 = sin(q(1,:));
c12 = cos(q(1,:) + q(2,:));  s12 = sin(q(1,:) + q(2,:));
shoulder = a.L3 * [c3; s3; zeros(1, size(q, 2))];
elbow    = shoulder + a.L1 * [c3.*c1; s3.*c1; s1];
ee       = elbow    + a.L2 * [c3.*c12; s3.*c12; s12];
end
