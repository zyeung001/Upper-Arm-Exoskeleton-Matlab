function R = run_sweep(p)
%RUN_SWEEP Sweep fill level x reach distance for all three controllers.
%   R = RUN_SWEEP(p) evaluates the n_f x n_d task grid. For each task:
%   trajectory -> slosh + required torque -> difficulty -> per-controller
%   split and metrics. Results are 3D arrays indexed by
%   (fill_index, dist_index, controller); controller order = R.laws.
%   Also saved to results/sweep_results.mat.

laws = {'fixed', 'fill', 'difficulty'};
f_grid = linspace(p.sweep.f_min, p.sweep.f_max, p.sweep.n_f);
d_grid = linspace(p.sweep.d_min, p.sweep.d_max, p.sweep.n_d);
nf = numel(f_grid);  nd = numel(d_grid);  nc = numel(laws);

R.laws = laws;  R.f_grid = f_grid;  R.d_grid = d_grid;  R.p = p;
R.D          = zeros(nf, nd);
R.peak_slosh = zeros(nf, nd);
R.E_human    = zeros(nf, nd, nc);
R.E_exo      = zeros(nf, nd, nc);
R.alpha      = zeros(nf, nd, nc);
R.status     = zeros(nf, nd, nc);

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
            R.E_human(i,j,c) = m.E_human;
            R.E_exo(i,j,c)   = m.E_exo;
            R.alpha(i,j,c)   = alpha;
            R.status(i,j,c)  = m.status;
        end
    end
    fprintf('  distance %2d/%d done\n', j, nd);
end

R.coverage = squeeze(mean(mean(R.status == 0, 1), 2)).';  % in-band fraction

fprintf('\nDifficulty D range: [%.2f, %.2f] N m s\n', min(R.D(:)), max(R.D(:)));
fprintf('Peak slosh range:   [%.3f, %.3f] rad\n', ...
    min(R.peak_slosh(:)), max(R.peak_slosh(:)));
fprintf('In-band coverage (headline):\n');
for c = 1:nc
    fprintf('  %-11s %5.1f%%\n', laws{c}, 100*R.coverage(c));
end

outdir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(outdir, 'dir'), mkdir(outdir); end
save(fullfile(outdir, 'sweep_results.mat'), 'R');
end
