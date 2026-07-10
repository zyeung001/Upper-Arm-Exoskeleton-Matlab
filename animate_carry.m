function animate_carry(f, d, opts)
%ANIMATE_CARRY Animated walkthrough of one carry task.
%   ANIMATE_CARRY            fairly hard task (f = 0.8, d = 0.4)
%   ANIMATE_CARRY(f, d)      fill level f in [0,1], reach distance d (m)
%   ANIMATE_CARRY(f, d, Speed=0.5, Gif=true)
%     Speed  playback speed factor, 1 = real time (default 0.5, slow-mo)
%     Gif    also write results/figures/carry_f<f>_d<d>.gif (default false)
%
%   What you see:
%     LEFT   - the 3-DOF arm carrying the cup from "start" to "target".
%              The liquid surface inside the cup tilts with the slosh
%              angle phi (true scale).
%     RIGHT  - top to bottom:
%              1. cup cross-section: the liquid tilting, seen side-on
%              2. slosh angle phi(t) - the disturbance the carry excites
%              3. instantaneous torque: total demand and the exo/human
%                 split made by the difficulty-adaptive controller
%              4. cumulative human effort for ALL THREE controllers vs
%                 the target band - the study's actual question. A curve
%                 ending inside the grey band = that controller kept the
%                 human comfortable on this task.
%
%   A per-controller metrics table is also printed to the console.

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

% ---- Simulate the task and all three controllers -------------------------
p = calibrate_controller(params());
traj = make_trajectory(d, p.sim.T_move, p.sim.dt, p);
[tau_total, peak_slosh, slosh] = compute_required_torque(traj, f, p);
D = compute_difficulty(traj.t, tau_total);
t = traj.t;  N = numel(t);

laws = {'fixed', 'fill', 'difficulty'};
law_names = {'fixed', 'fill-only', 'difficulty'};
alpha = zeros(1, 3);  E_cum = zeros(3, N);  met = cell(1, 3);
for c = 1:3
    [tau_exo_c, tau_human_c, alpha(c)] = controllers(laws{c}, tau_total, f, D, p);
    E_cum(c,:) = cumtrapz(t, sum(abs(tau_human_c), 1));
    met{c} = compute_metrics(t, tau_human_c, tau_exo_c, p);
end
[tau_exo, tau_human] = controllers('difficulty', tau_total, f, D, p);

% ---- Console metrics table -------------------------------------------------
status_str = {'BELOW band (over-assisted, wasteful)', 'in band', ...
              'ABOVE band (under-supported)'};
fprintf('\nTask: fill = %.2f, distance = %.2f m\n', f, d);
fprintf('  difficulty D = %.2f N m s, peak slosh = %.1f deg\n', ...
    D, rad2deg(peak_slosh));
fprintf('  target human-effort band = [%.2f, %.2f] N m s\n\n', ...
    p.band.E_low, p.band.E_high);
fprintf('  %-12s %6s %9s %8s %12s   %s\n', ...
    'controller', 'alpha', 'E_human', 'E_exo', 'pk tau_hum', 'band status');
for c = 1:3
    fprintf('  %-12s %6.2f %9.2f %8.2f %9.1f N m   %s\n', law_names{c}, ...
        alpha(c), met{c}.E_human, met{c}.E_exo, met{c}.tau_pk_human, ...
        status_str{met{c}.status + 2});
end
fprintf('\n');

% ---- Precompute geometry ----------------------------------------------------
[shoulder, elbow, ee] = fk_points(traj.q, p);
Rc = p.cont.Rc;  Hc = p.cont.Hc;  hf = f * Hc;
uu = linspace(0, 2*pi, 25);  cs = cos(uu);  sn = sin(uu);

% ---- Figure -----------------------------------------------------------------
col_ctrl = [0 114 178; 230 159 0; 0 158 115] / 255;   % fixed / fill / difficulty
col_exo  = [86 180 233] / 255;                        % sky blue
col_hum  = [204 121 167] / 255;                       % pink
col_liq  = [86 180 233] / 255;
col_gray = [0.4 0.4 0.4];

fig = figure('Color', 'w', 'Position', [40 40 1250 720], ...
    'Name', sprintf('Carry: f = %.2f, d = %.2f m', f, d));
fig.Theme = 'light';
tl = tiledlayout(fig, 4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf(['Carrying a %.0f%%-full cup over %.2f m   |   ' ...
    'difficulty D = %.2f N m s, peak slosh = %.1f%s'], ...
    100*f, d, D, rad2deg(peak_slosh), char(176)));

% Left: 3D arm + cup (spans all rows)
ax1 = nexttile(tl, 1, [4 1]);
hold(ax1, 'on');  grid(ax1, 'on');  daspect(ax1, [1 1 1]);
view(ax1, -40, 18);
xlabel(ax1, 'x (m)'); ylabel(ax1, 'y (m)'); zlabel(ax1, 'z (m)');
lim = p.arm.L3 + p.arm.L1 + p.arm.L2;
xlim(ax1, [-0.1 lim]); ylim(ax1, [-0.35 0.55]); zlim(ax1, [-0.75 0.55]);
plot3(ax1, [0 0], [0 0], [-0.75 0.15], 'k-', 'LineWidth', 3);   % torso axis
text(ax1, 0, 0, 0.2, 'torso', 'HorizontalAlignment', 'center', 'FontSize', 8);
plot3(ax1, ee(1,:), ee(2,:), ee(3,:), ':', 'Color', col_gray);  % hand path
plot3(ax1, ee(1,[1 N]), ee(2,[1 N]), ee(3,[1 N]), 'o', ...
    'Color', col_gray, 'MarkerFaceColor', 'w');
text(ax1, ee(1,1), ee(2,1), ee(3,1) - 0.07, 'start', ...
    'HorizontalAlignment', 'center', 'FontSize', 9);
text(ax1, ee(1,N), ee(2,N), ee(3,N) - 0.07, 'target', ...
    'HorizontalAlignment', 'center', 'FontSize', 9);
h_arm = plot3(ax1, nan, nan, nan, '-o', 'LineWidth', 3.5, ...
    'Color', [0.25 0.25 0.3], 'MarkerFaceColor', 'w', 'MarkerSize', 5);
h_cupb = plot3(ax1, nan, nan, nan, '-', 'Color', col_gray);       % cup bottom
h_cupt = plot3(ax1, nan, nan, nan, '-', 'Color', col_gray);       % cup rim
h_cups = plot3(ax1, nan, nan, nan, '-', 'Color', col_gray);       % cup sides
h_liq  = patch(ax1, nan, nan, nan, col_liq, 'FaceAlpha', 0.75, ...
    'EdgeColor', col_liq);                                        % liquid surface
h_time = title(ax1, 't = 0.00 s');

% Right 1: cup cross-section (side view, in the swing plane)
ax2 = nexttile(tl, 2);
hold(ax2, 'on');  daspect(ax2, [1 1 1]);
plot(ax2, [-Rc -Rc nan Rc Rc nan -Rc Rc], ...
          [Hc  0  nan Hc 0  nan 0   0 ], 'k-', 'LineWidth', 1.5);
yline(ax2, hf, ':', 'Color', col_gray);            % resting liquid level
h_xsec = patch(ax2, [-Rc Rc Rc -Rc], [0 0 hf hf], col_liq, ...
    'FaceAlpha', 0.75, 'EdgeColor', col_liq);
xlim(ax2, [-2.2*Rc, 2.2*Rc]);  ylim(ax2, [-0.2*Hc, 1.35*Hc]);
set(ax2, 'XTick', [], 'YTick', []);
title(ax2, 'Inside the cup (side view)');

% Right 2: slosh angle
ax3 = nexttile(tl, 4);
hold(ax3, 'on');  grid(ax3, 'on');
plot(ax3, t, rad2deg(slosh.phi), '-', 'Color', col_gray);
h_cur3 = xline(ax3, 0, '-', 'Color', col_exo, 'LineWidth', 1);
h_dot3 = plot(ax3, 0, 0, 'o', 'Color', col_exo, 'MarkerFaceColor', col_exo);
ylabel(ax3, '\phi (deg)');
title(ax3, 'Slosh angle (liquid surface tilt)');

% Right 3: instantaneous torque split (difficulty-adaptive)
ax4 = nexttile(tl, 6);
hold(ax4, 'on');  grid(ax4, 'on');
plot(ax4, t, sum(abs(tau_total), 1), '-', 'Color', col_gray,  'LineWidth', 1.5);
plot(ax4, t, sum(abs(tau_exo), 1),   '-', 'Color', col_exo, 'LineWidth', 1.5);
plot(ax4, t, sum(abs(tau_human), 1), '-', 'Color', col_hum, 'LineWidth', 1.5);
h_cur4 = xline(ax4, 0, '-', 'Color', col_exo, 'LineWidth', 1);
legend(ax4, {'total demand', 'exo share', 'human share'}, ...
    'Location', 'northwest', 'Box', 'off', 'FontSize', 8);
ylabel(ax4, '\Sigma_j |\tau_j| (N m)');
title(ax4, sprintf('Torque split now (difficulty-adaptive, \\alpha = %.2f)', ...
    alpha(3)));

% Right 4: cumulative human effort vs target band, all three controllers
ax5 = nexttile(tl, 8);
hold(ax5, 'on');  grid(ax5, 'on');
yb = [p.band.E_low, p.band.E_high];
patch(ax5, t([1 N N 1]), yb([1 1 2 2]), [0.5 0.5 0.5], ...
    'FaceAlpha', 0.18, 'EdgeColor', 'none');
text(ax5, 0.98*t(N), mean(yb), 'target band', 'FontSize', 8, ...
    'Color', [0.35 0.35 0.35], 'HorizontalAlignment', 'right');
h_ec = gobjects(1, 3);
for c = 1:3
    h_ec(c) = plot(ax5, t, E_cum(c,:), '-', 'Color', col_ctrl(c,:), ...
        'LineWidth', 1.5);
end
h_cur5 = xline(ax5, 0, '-', 'Color', col_exo, 'LineWidth', 1);
legend(h_ec, law_names, 'Location', 'northwest', 'Box', 'off', 'FontSize', 8);
xlabel(ax5, 't (s)');  ylabel(ax5, 'E_{human}(t) (N m s)');
ylim(ax5, [0, max([E_cum(:); yb(2)]) * 1.15]);
title(ax5, 'Human effort accumulating - who ends in the band?');

% ---- Playback ---------------------------------------------------------------
if opts.Gif
    gifdir = fullfile(root, 'results', 'figures');
    if ~exist(gifdir, 'dir'), mkdir(gifdir); end
    giffile = fullfile(gifdir, sprintf('carry_f%03.0f_d%03.0f.gif', 100*f, 100*d));
    if exist(giffile, 'file'), delete(giffile); end
end
zhat = [0; 0; 1];
stride = max(1, round(N / 100));   % ~100 frames
for k = [1:stride:N, N]
    if ~ishandle(fig), return; end   % window closed mid-playback
    th3 = traj.q(3,k);  phi = slosh.phi(k);
    er = [cos(th3); sin(th3); 0];    % swing (radial) direction
    et = [-sin(th3); cos(th3); 0];
    P  = ee(:,k);

    set(h_arm, 'XData', [0, shoulder(1,k), elbow(1,k), P(1)], ...
               'YData', [0, shoulder(2,k), elbow(2,k), P(2)], ...
               'ZData', [0, shoulder(3,k), elbow(3,k), P(3)]);

    % Cup wireframe: bottom ring, rim, four side lines
    ringb = P + Rc*(er*cs + et*sn);
    ringt = ringb + Hc*zhat;
    set(h_cupb, 'XData', ringb(1,:), 'YData', ringb(2,:), 'ZData', ringb(3,:));
    set(h_cupt, 'XData', ringt(1,:), 'YData', ringt(2,:), 'ZData', ringt(3,:));
    sx = []; sy = []; sz = [];
    for e = [er, -er, et, -et]
        base = P + Rc*e;
        sx = [sx, base(1), base(1), nan]; %#ok<AGROW>
        sy = [sy, base(2), base(2), nan]; %#ok<AGROW>
        sz = [sz, base(3), base(3) + Hc, nan]; %#ok<AGROW>
    end
    set(h_cups, 'XData', sx, 'YData', sy, 'ZData', sz);

    % Liquid surface: disc tilted by phi in the swing plane (true scale)
    a1 = cos(phi)*er + sin(phi)*zhat;
    surfpts = P + hf*zhat + Rc*(a1*cs + et*sn);
    set(h_liq, 'XData', surfpts(1,:), 'YData', surfpts(2,:), ...
               'ZData', surfpts(3,:));
    h_time.String = sprintf('t = %.2f s', t(k));

    % Cross-section: liquid polygon with surface slope tan(phi)
    dz = Rc * tan(phi);
    set(h_xsec, 'XData', [-Rc Rc Rc -Rc], 'YData', [0 0 hf+dz hf-dz]);

    h_cur3.Value = t(k);  h_cur4.Value = t(k);  h_cur5.Value = t(k);
    set(h_dot3, 'XData', t(k), 'YData', rad2deg(phi));
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
