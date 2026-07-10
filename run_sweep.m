function R = run_sweep(p)
%RUN_SWEEP Sweep fill level x reach distance for all three controllers.
%   R = RUN_SWEEP(p) evaluates the n_f x n_d task grid. For each task:
%   trajectory -> slosh + required torque -> difficulty -> per-controller
%   split and metrics. Results are 3D arrays indexed by
%   (fill_index, dist_index, controller); controller order = R.laws.
%   Also saved to results/sweep_results.mat.

assert(all(isfinite([p.ctrl.D_lo, p.ctrl.D_hi, p.band.E_low])), ...
    'run_sweep: p is uncalibrated - call p = calibrate_controller(p) first');

laws = {'fixed', 'fill', 'difficulty'};
f_grid = linspace(p.sweep.f_min, p.sweep.f_max, p.sweep.n_f);
d_grid = linspace(p.sweep.d_min, p.sweep.d_max, p.sweep.n_d);
nf = numel(f_grid);  nd = numel(d_grid);  nc = numel(laws);

R.laws = laws;  R.f_grid = f_grid;  R.d_grid = d_grid;  R.p = p;
R.D            = zeros(nf, nd);
R.peak_slosh   = zeros(nf, nd);
R.E_human      = zeros(nf, nd, nc);
R.E_exo        = zeros(nf, nd, nc);
R.alpha        = zeros(nf, nd, nc);
R.status       = zeros(nf, nd, nc);
R.tau_pk_human = zeros(nf, nd, nc);

fprintf('Running %dx%d sweep...\n', nf, nd);
for j = 1:nd
    % trajectory depends on distance only - reuse across fills
    traj = make_trajectory(d_grid(j), p.sim.T_move, p.sim.dt, p);
    for i = 1:nf
        [tau_total, peak_slosh] = compute_required_torque(traj, f_grid(i), p);
        D = compute_difficulty(traj.t, tau_total);
        R.D(i,j) = D;
        R.peak_slosh(i,j) = peak_slosh;
        for c = 1:nc
            [tau_exo, tau_human, alpha] = ...
                controllers(laws{c}, tau_total, f_grid(i), D, p);
            m = compute_metrics(traj.t, tau_human, tau_exo, p);
            R.E_human(i,j,c)      = m.E_human;
            R.E_exo(i,j,c)        = m.E_exo;
            R.alpha(i,j,c)        = alpha;
            R.status(i,j,c)       = m.status;
            R.tau_pk_human(i,j,c) = m.tau_pk_human;
        end
    end
    fprintf('  distance %2d/%d done\n', j, nd);
end

R.coverage = squeeze(mean(mean(R.status == 0, 1), 2)).';  % in-band fraction

fprintf('\nDifficulty D range: [%.2f, %.2f] N m s\n', min(R.D(:)), max(R.D(:)));
fprintf('Peak slosh range:   [%.1f, %.1f] deg\n', ...
    rad2deg(min(R.peak_slosh(:))), rad2deg(max(R.peak_slosh(:))));
fprintf('Target human-effort band: [%.2f, %.2f] N m s\n\n', ...
    p.band.E_low, p.band.E_high);
fprintf('Per-controller summary over %d tasks:\n', nf*nd);
fprintf('  %-12s %8s %10s %10s %9s %9s %11s\n', 'controller', 'in-band', ...
    'over-asst', 'under-sup', 'mean E_h', 'mean E_x', 'max pk tau_h');
for c = 1:nc
    st = R.status(:,:,c);
    fprintf('  %-12s %7.0f%% %9.0f%% %9.0f%% %9.2f %9.2f %8.1f N m\n', ...
        laws{c}, 100*mean(st(:) == 0), 100*mean(st(:) == -1), ...
        100*mean(st(:) == 1), mean(R.E_human(:,:,c), 'all'), ...
        mean(R.E_exo(:,:,c), 'all'), max(R.tau_pk_human(:,:,c), [], 'all'));
end
fprintf(['  (over-asst = human effort below band, exo working harder than ' ...
    'needed;\n   under-sup = human effort above band, left straining)\n']);

% Where the difficulty law's clamps bind (honest accounting of misses:
% inside the clamps the closed-form law holds effort at E* exactly).
a3 = R.alpha(:,:,3);
lo = abs(a3 - p.ctrl.alpha_min) < 1e-9;
hi = abs(a3 - p.ctrl.alpha_max) < 1e-9;
fprintf(['difficulty law clamp binding: %.0f%% alpha_min-clamped, ' ...
    '%.0f%% law-controlled, %.0f%% alpha_max-clamped\n'], ...
    100*mean(lo(:)), 100*mean(~lo(:) & ~hi(:)), 100*mean(hi(:)));

outdir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(outdir, 'dir'), mkdir(outdir); end
save(fullfile(outdir, 'sweep_results.mat'), 'R');
end
