function make_figures(R)
%MAKE_FIGURES All output figures from a sweep result struct R (run_sweep).
%   Every map is a 2D fill x distance heatmap (one simulated carry per cell);
%   color encodes one job per figure (magnitude, or polarity about a target).
%   1. Human-effort maps, diverging about the target band (central figure):
%      gray = in band, blue = over-assisted (waste), red = under-supported
%   2. Difference map: E_human(difficulty-adaptive) - E_human(fixed)
%   3. Exo-effort maps (guards the alpha=1 trivial fix)
%   4. Band-status composition bars - the headline (% over / in / under)
%   5. Peak-slosh map (task-space characterization)
%   6. Assistance-level (alpha) maps - what each controller actually does
%   PNGs are saved to results/figures/.

p = R.p;
outdir = fullfile(fileparts(mfilename('fullpath')), 'results', 'figures');
if ~exist(outdir, 'dir'), mkdir(outdir); end

names = {'Fixed', 'Fill-only adaptive', 'Difficulty-adaptive'};
d = R.d_grid;  f = R.f_grid;

% Ink roles (reference dataviz palette, light mode)
ink1  = hex2rgb('#0b0b0b');    % primary
ink2  = hex2rgb('#52514e');    % secondary
muted = hex2rgb('#898781');    % axis/labels

% ---- 1. Human-effort maps, diverging about the band (central) -----------
% One map answers three questions at once: how much effort (intensity),
% which failure mode (hue), and where in the task space (position).
band = [p.band.E_low, p.band.E_high];
ctr  = mean(band);
lim  = max(abs([R.E_human(:) - ctr; band(:) - ctr]));
cl   = [ctr - lim, ctr + lim];
fig = new_fig([1180 400]);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for c = 1:3
    ax = nexttile(tl);
    imagesc(ax, d, f, R.E_human(:,:,c));
    style_map(ax, d, f, muted);
    clim(ax, cl);
    title(ax, sprintf('%s — %.0f%% in band', names{c}, 100*R.coverage(c)), ...
        'Color', ink1, 'FontWeight', 'normal');
end
colormap(fig, band_diverging_map(cl, band));
cb = colorbar(ax);  cb.Layout.Tile = 'east';
cb.Ticks = [cl(1), band(1), band(2), cl(2)];
cb.TickLabels = compose('%.1f', cb.Ticks);
cb.Label.String = 'E_{human} (N m s)   [gray = in band]';
cb.Color = ink2;
title(tl, {'Human effort vs. task', ['blue = over-assisted (waste),  ' ...
    'gray = in band,  red = under-supported (strain)']}, 'Color', ink1);
save_fig(fig, outdir, 'fig1_human_effort_maps');

% ---- 2. Difference map ----------------------------------------------------
fig = new_fig([520 420]);
ax = axes(fig);
dE = R.E_human(:,:,3) - R.E_human(:,:,1);
imagesc(ax, d, f, dE);
style_map(ax, d, f, muted);
lim = max(abs(dE(:)));  clim(ax, [-lim lim]);
colormap(ax, diverging_map());
cb = colorbar(ax);  cb.Color = ink2;
cb.Label.String = '\Delta E_{human} (N m s)';
title(ax, {'E_{human}: difficulty-adaptive − fixed', ...
    '(blue = adaptation relieves the human)'}, 'Color', ink1, ...
    'FontWeight', 'normal');
save_fig(fig, outdir, 'fig2_difference_map');

% ---- 3. Exo-effort maps ----------------------------------------------------
fig = new_fig([1180 400]);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for c = 1:3
    ax = nexttile(tl);
    imagesc(ax, d, f, R.E_exo(:,:,c));
    style_map(ax, d, f, muted);
    clim(ax, [0, max(R.E_exo(:))]);
    title(ax, names{c}, 'Color', ink1, 'FontWeight', 'normal');
end
colormap(fig, sequential_map());
cb = colorbar(ax);  cb.Layout.Tile = 'east';  cb.Color = ink2;
cb.Label.String = 'E_{exo} (N m s)';
title(tl, 'Exoskeleton effort (bounded: \alpha \leq \alpha_{max} < 1)', ...
    'Color', ink1);
save_fig(fig, outdir, 'fig3_exo_effort_maps');

% ---- 4. Band-status composition bars (headline) ----------------------------
% 100% stacked: what fraction of the 100 tasks each controller leaves
% over-assisted / in band / under-supported. Status colors are reserved
% for state and always paired with a text label.
col_over  = hex2rgb('#fab219');   % warning: waste
col_in    = hex2rgb('#0ca30c');   % good
col_under = hex2rgb('#d03b3b');   % critical: strain
fig = new_fig([640 300]);
ax = axes(fig);  hold(ax, 'on');
share = zeros(3, 3);              % rows: controller; cols: over, in, under
for c = 1:3
    st = R.status(:,:,c);
    share(c,:) = 100 * [mean(st(:) == -1), mean(st(:) == 0), mean(st(:) == 1)];
end
order = [3 2 1];                  % difficulty on top
hb = barh(ax, share(order,:), 'stacked', 'BarWidth', 0.62, ...
    'EdgeColor', 'w', 'LineWidth', 2);
hb(1).FaceColor = col_over;  hb(2).FaceColor = col_in;
hb(3).FaceColor = col_under;
xedge = [zeros(3,1), cumsum(share(order,:), 2)];
for r = 1:3
    for s = 1:3
        if share(order(r), s) >= 5
            w = (s == 2);         % bold the in-band share (the headline)
            text(ax, mean(xedge(r, s:s+1)), r, ...
                sprintf('%.0f%%', share(order(r), s)), ...
                'HorizontalAlignment', 'center', 'FontSize', 9 + w, ...
                'FontWeight', ternary(w, 'bold', 'normal'), 'Color', 'w');
        end
    end
end
set(ax, 'YTick', 1:3, 'YTickLabel', names(order), 'XLim', [0 100], ...
    'YLim', [0.4 3.6], 'Box', 'off', 'XColor', muted, 'YColor', ink2, ...
    'TickLength', [0 0]);
xlabel(ax, 'share of the 100 tasks (%)', 'Color', ink2);
legend(hb, {'over-assisted (waste)', 'in band', 'under-supported (strain)'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off', ...
    'TextColor', ink2);
title(ax, 'Where the human effort lands, per controller', 'Color', ink1, ...
    'FontWeight', 'normal');
save_fig(fig, outdir, 'fig4_band_status_stack');

% ---- 5. Peak-slosh map ------------------------------------------------------
% Second sequential context in the suite -> second hue (aqua), own ramp.
fig = new_fig([520 420]);
ax = axes(fig);
imagesc(ax, d, f, rad2deg(R.peak_slosh));
style_map(ax, d, f, muted);
colormap(ax, sequential_map_aqua());
cb = colorbar(ax);  cb.Color = ink2;
cb.Label.String = 'peak |\phi| (deg)';
title(ax, 'Peak slosh angle over the task space', 'Color', ink1, ...
    'FontWeight', 'normal');
save_fig(fig, outdir, 'fig5_peak_slosh_map');

% ---- 6. Assistance-level maps ----------------------------------------------
fig = new_fig([1180 400]);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for c = 1:3
    ax = nexttile(tl);
    imagesc(ax, d, f, R.alpha(:,:,c));
    style_map(ax, d, f, muted);
    clim(ax, [p.ctrl.alpha_min, p.ctrl.alpha_max]);
    title(ax, names{c}, 'Color', ink1, 'FontWeight', 'normal');
end
colormap(fig, sequential_map());
cb = colorbar(ax);  cb.Layout.Tile = 'east';  cb.Color = ink2;
cb.Label.String = 'assistance fraction \alpha';
title(tl, sprintf(['What each controller does: exo share \\alpha ' ...
    '(\\alpha_{min} = %.2f, \\alpha_{max} = %.2f)'], ...
    p.ctrl.alpha_min, p.ctrl.alpha_max), 'Color', ink1);
save_fig(fig, outdir, 'fig6_alpha_maps');

fprintf('Figures saved to %s\n', outdir);
end

% ------------------------------------------------------------------------
function fig = new_fig(sz)
% Pop up in a desktop session; stay hidden under matlab -batch.
vis = 'off';
if usejava('desktop'), vis = 'on'; end
fig = figure('Visible', vis, 'Color', hex2rgb('#fcfcfb'), ...
    'Position', [80 80 sz]);
fig.Theme = 'light';   % export must not follow a dark desktop theme
end

function style_map(ax, d, f, muted)
% Shared heatmap chrome: y up, hairline white cell grid, recessive axes.
axis(ax, 'xy');
hold(ax, 'on');
dx = (d(2) - d(1)) / 2;  dy = (f(2) - f(1)) / 2;
xe = [d - dx, d(end) + dx];  ye = [f - dy, f(end) + dy];
vx = [repmat(xe, 2, 1);                    nan(1, numel(xe))];
vy = [repmat([ye(1); ye(end)], 1, numel(xe)); nan(1, numel(xe))];
hx = [repmat([xe(1); xe(end)], 1, numel(ye)); nan(1, numel(ye))];
hy = [repmat(ye, 2, 1);                    nan(1, numel(ye))];
plot(ax, [vx(:); hx(:)], [vy(:); hy(:)], '-', 'Color', 'w', 'LineWidth', 0.75);
xlim(ax, xe([1 end]));  ylim(ax, ye([1 end]));
set(ax, 'XColor', muted, 'YColor', muted, 'TickLength', [0 0], 'Box', 'off');
xlabel(ax, 'reach distance d (m)');
ylabel(ax, 'fill level f (-)');
end

function save_fig(fig, outdir, name)
exportgraphics(fig, fullfile(outdir, [name '.png']), 'Resolution', 200);
if strcmp(fig.Visible, 'off'), close(fig); end   % leave on screen if shown
end

function rgb = hex2rgb(h)
rgb = double(sscanf(h(2:end), '%2x%2x%2x')') / 255;
end

function map = ramp(hexes, n)
% Interpolate an n-step colormap through a list of hex anchors.
anchors = cell2mat(cellfun(@(h) hex2rgb(h), hexes(:), 'UniformOutput', false));
map = interp1(linspace(0, 1, size(anchors, 1)), anchors, linspace(0, 1, n));
end

function map = sequential_map()
% Reference sequential blue, steps 100 -> 700 (light = near zero).
map = ramp({'#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', ...
    '#184f95', '#0d366b'}, 256);
end

function map = sequential_map_aqua()
% Second sequential hue (aqua), own light -> dark ramp.
map = ramp({'#dcf3ea', '#8fd9bd', '#3cbd8d', '#1baf7a', '#12805a', ...
    '#0a4a34'}, 256);
end

function map = diverging_map()
% Reference diverging pair: blue <-> red, neutral gray midpoint.
map = ramp({'#104281', '#3987e5', '#9ec5f4', '#f0efec', '#f2afa5', ...
    '#e34948', '#8f1d1d'}, 256);
end

function map = band_diverging_map(cl, band)
% Diverging map whose NEUTRAL ZONE is the target band [band(1), band(2)]
% mapped onto value limits cl: gray inside the band, blue deepening below
% (over-assisted), red deepening above (under-supported).
n = 256;
v = linspace(cl(1), cl(2), n)';
blues = ramp({'#cde2fb', '#6da7ec', '#3987e5', '#1c5cab', '#0d366b'}, n);
reds  = ramp({'#f6cdc6', '#ee9385', '#e34948', '#b52a2a', '#7e1616'}, n);
map = repmat(hex2rgb('#f0efec'), n, 1);
lo = v < band(1);  hi = v > band(2);
tlo = (band(1) - v(lo)) / max(band(1) - cl(1), eps);   % 0 at edge, 1 at min
thi = (v(hi) - band(2)) / max(cl(2) - band(2), eps);
map(lo,:) = interp1(linspace(0, 1, n), blues, tlo);
map(hi,:) = interp1(linspace(0, 1, n), reds,  thi);
end

function out = ternary(cond, a, b)
if cond, out = a; else, out = b; end
end
