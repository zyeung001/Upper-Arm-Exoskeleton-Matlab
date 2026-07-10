function make_figures(R)
%MAKE_FIGURES All output figures from a sweep result struct R (run_sweep).
%   1. Human-effort surfaces per controller + target band planes (central)
%   2. Difference surface: E_human(difficulty-adaptive) - E_human(fixed)
%   3. Exo-effort surfaces per controller (guards the alpha=1 trivial fix)
%   4. In-band coverage headline bars
%   5. Peak-slosh surface (task-space characterization)
%   6. In-band status maps - WHERE each controller fails, and how
%   7. Assistance-level (alpha) maps - what each controller actually does
%   PNGs are saved to results/figures/.

p = R.p;
outdir = fullfile(fileparts(mfilename('fullpath')), 'results', 'figures');
if ~exist(outdir, 'dir'), mkdir(outdir); end

names = {'Fixed', 'Fill-only adaptive', 'Difficulty-adaptive'};
% Fixed categorical order, colorblind-safe (Okabe-Ito): blue, orange, green
cols  = [0 114 178; 230 159 0; 0 158 115] / 255;
[Dm, Fm] = meshgrid(R.d_grid, R.f_grid);

% ---- 1. Human-effort surfaces + band (central figure) -------------------
fig = new_fig([1100 420]);
zmax = 1.1 * max(R.E_human(:));
for c = 1:3
    ax = subplot(1, 3, c);
    surf(ax, Dm, Fm, R.E_human(:,:,c), 'FaceColor', cols(c,:), ...
        'FaceAlpha', 0.85, 'EdgeColor', 'w', 'EdgeAlpha', 0.5);
    hold(ax, 'on');
    band = @(z) surf(ax, Dm, Fm, z*ones(size(Dm)), 'FaceColor', [0.45 0.45 0.45], ...
        'FaceAlpha', 0.25, 'EdgeColor', 'none');
    band(p.band.E_low);  band(p.band.E_high);
    style_axes3(ax, 'E_{human} (N m s)');
    zlim(ax, [0 zmax]);
    title(ax, sprintf('%s — %.0f%% in band', names{c}, 100*R.coverage(c)));
end
sgtitle(fig, 'Human effort vs. task, with target band [E_{low}, E_{high}]');
save_fig(fig, outdir, 'fig1_human_effort_surfaces');

% ---- 2. Difference surface ----------------------------------------------
fig = new_fig([560 440]);
ax = axes(fig);
dE = R.E_human(:,:,3) - R.E_human(:,:,1);
surf(ax, Dm, Fm, dE, 'EdgeColor', 'w', 'EdgeAlpha', 0.5);
colormap(ax, diverging_map());
lim = max(abs(dE(:)));  clim(ax, [-lim lim]);
cb = colorbar(ax);  cb.Label.String = '\Delta E_{human} (N m s)';
style_axes3(ax, '\Delta E_{human} (N m s)');
title(ax, {'E_{human}: difficulty-adaptive − fixed', ...
    '(negative = adaptation relieves the human)'});
save_fig(fig, outdir, 'fig2_difference_surface');

% ---- 3. Exo-effort surfaces ----------------------------------------------
fig = new_fig([1100 420]);
for c = 1:3
    ax = subplot(1, 3, c);
    surf(ax, Dm, Fm, R.E_exo(:,:,c), 'EdgeColor', 'w', 'EdgeAlpha', 0.5);
    colormap(ax, sequential_map());
    clim(ax, [0 max(R.E_exo(:))]);
    style_axes3(ax, 'E_{exo} (N m s)');
    zlim(ax, [0 1.1*max(R.E_exo(:))]);
    title(ax, names{c});
end
sgtitle(fig, 'Exoskeleton effort (bounded: \alpha \leq \alpha_{max} < 1)');
save_fig(fig, outdir, 'fig3_exo_effort_surfaces');

% ---- 4. In-band coverage headline ----------------------------------------
fig = new_fig([480 380]);
ax = axes(fig);
b = bar(ax, 100*R.coverage, 0.6, 'FaceColor', 'flat', 'EdgeColor', 'none');
b.CData = cols;
text(ax, 1:3, 100*R.coverage + 3, ...
    compose('%.0f%%', 100*R.coverage), 'HorizontalAlignment', 'center');
set(ax, 'XTickLabel', {'Fixed', 'Fill-only', 'Difficulty'}, 'Box', 'off');
ylim(ax, [0 110]);  grid(ax, 'on');
ylabel(ax, 'Tasks with human effort in band (%)');
title(ax, sprintf('In-band coverage over %d tasks', numel(R.D)));
save_fig(fig, outdir, 'fig4_inband_coverage');

% ---- 5. Peak-slosh surface ------------------------------------------------
fig = new_fig([560 440]);
ax = axes(fig);
surf(ax, Dm, Fm, rad2deg(R.peak_slosh), 'EdgeColor', 'w', 'EdgeAlpha', 0.5);
colormap(ax, sequential_map());
cb = colorbar(ax);  cb.Label.String = 'peak |\phi| (deg)';
style_axes3(ax, 'peak |\phi| (deg)');
title(ax, 'Peak slosh angle over the task space');
save_fig(fig, outdir, 'fig5_peak_slosh_surface');

% ---- 6. In-band status maps ------------------------------------------------
% Reads like a scorecard: where in the task space each controller keeps the
% human in the band, over-assists (waste), or under-supports (strain).
fig = new_fig([1150 400]);
smap = [86 180 233; 0 158 115; 213 94 0] / 255;   % over / in / under
for c = 1:3
    ax = subplot(1, 3, c);
    imagesc(ax, R.d_grid, R.f_grid, R.status(:,:,c));
    axis(ax, 'xy');
    colormap(ax, smap);  clim(ax, [-1.5 1.5]);
    xlabel(ax, 'reach distance d (m)');  ylabel(ax, 'fill level f (-)');
    st = R.status(:,:,c);
    title(ax, {names{c}, sprintf('%.0f%% in band | %.0f%% over | %.0f%% under', ...
        100*mean(st(:) == 0), 100*mean(st(:) == -1), 100*mean(st(:) == 1))});
end
cb = colorbar(ax);
cb.Ticks = [-1 0 1];
cb.TickLabels = {'over-assisted (waste)', 'in band', 'under-supported'};
sgtitle(fig, 'Human-effort band status across the task space');
save_fig(fig, outdir, 'fig6_inband_status_maps');

% ---- 7. Assistance-level maps ----------------------------------------------
fig = new_fig([1150 400]);
for c = 1:3
    ax = subplot(1, 3, c);
    imagesc(ax, R.d_grid, R.f_grid, R.alpha(:,:,c));
    axis(ax, 'xy');
    colormap(ax, sequential_map());
    clim(ax, [p.ctrl.alpha_min, p.ctrl.alpha_max]);
    xlabel(ax, 'reach distance d (m)');  ylabel(ax, 'fill level f (-)');
    title(ax, names{c});
end
cb = colorbar(ax);
cb.Label.String = 'assistance fraction \alpha';
sgtitle(fig, ['What each controller does: exo share \alpha over the task ' ...
    'space (\alpha_{min} = ' sprintf('%.2f', p.ctrl.alpha_min) ...
    ', \alpha_{max} = ' sprintf('%.2f', p.ctrl.alpha_max) ')']);
save_fig(fig, outdir, 'fig7_alpha_maps');

fprintf('Figures saved to %s\n', outdir);
end

% ------------------------------------------------------------------------
function fig = new_fig(sz)
% Pop up in a desktop session; stay hidden under matlab -batch.
vis = 'off';
if usejava('desktop'), vis = 'on'; end
fig = figure('Visible', vis, 'Color', 'w', 'Position', [80 80 sz]);
fig.Theme = 'light';   % export must not follow a dark desktop theme
end

function style_axes3(ax, zlab)
xlabel(ax, 'reach distance d (m)');
ylabel(ax, 'fill level f (-)');
zlabel(ax, zlab);
grid(ax, 'on');  view(ax, -35, 22);
end

function save_fig(fig, outdir, name)
exportgraphics(fig, fullfile(outdir, [name '.png']), 'Resolution', 200);
if strcmp(fig.Visible, 'off'), close(fig); end   % leave on screen if shown
end

function map = sequential_map()
% Single-hue sequential: light -> dark blue
map = interp1([0 1], [0.93 0.96 1.00; 0 0.28 0.55], linspace(0, 1, 256));
end

function map = diverging_map()
% Blue -> neutral light gray -> vermillion, neutral at the midpoint
anchors = [0 0.447 0.698; 0.94 0.94 0.94; 0.835 0.369 0];
map = interp1([0 0.5 1], anchors, linspace(0, 1, 256));
end
