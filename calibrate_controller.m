function p = calibrate_controller(p)
%CALIBRATE_CONTROLLER Tune the difficulty ramp and band floor - calibration
%   data only, fixed rules, no hand-sliding.
%   p = CALIBRATE_CONTROLLER(p) fills in p.ctrl.D_lo, p.ctrl.D_hi and
%   p.band.E_low from the 5x5 calibration grid (p.calib), which is asserted
%   DISJOINT from the evaluation grid (p.sweep). Rules:
%
%     D_lo  = min difficulty observed on the calibration grid
%     D_hi  = max difficulty observed on the calibration grid
%     E_low = (1 - alpha_min) * D_lo   - the lowest human effort ANY
%             in-bounds controller can leave on the easiest calibration
%             task; a lower floor would call even minimal assistance on
%             that task "over-assisted".
%
%   E_high stays as set a priori in params. Because the evaluation grid
%   extends beyond the calibration points (its corners are more extreme),
%   reported coverage is measured on tasks the tuning never saw.
%
%   The 25-task calibration sweep is cached in results/calibration.mat and
%   recomputed automatically when any physics/grid/bound parameter changes.

% ---- Disjointness assert (the train/test split) --------------------------
f_eval = linspace(p.sweep.f_min, p.sweep.f_max, p.sweep.n_f);
d_eval = linspace(p.sweep.d_min, p.sweep.d_max, p.sweep.n_d);
assert(~any(ismembertol(p.calib.f, f_eval, 1e-9)) && ...
       ~any(ismembertol(p.calib.d, d_eval, 1e-9)), ...
    'calibrate_controller: calibration grid overlaps the evaluation grid');

% ---- Cache key: everything the calibration result depends on -------------
key = {p.arm, p.cont, p.slosh, p.g, p.reach, p.sim, p.calib, ...
       p.ctrl.alpha_min, p.ctrl.alpha_max};
cachefile = fullfile(fileparts(mfilename('fullpath')), 'results', ...
    'calibration.mat');
if exist(cachefile, 'file') == 2
    S = load(cachefile);
    if isequal(S.key, key)
        p.ctrl.D_lo = S.D_lo;  p.ctrl.D_hi = S.D_hi;
        p.band.E_low = S.E_low;
        return
    end
end

% ---- Calibration sweep (difficulty only; controller-independent) ---------
fprintf('Calibrating on %dx%d grid (disjoint from evaluation grid)...\n', ...
    numel(p.calib.f), numel(p.calib.d));
D = zeros(numel(p.calib.f), numel(p.calib.d));
for j = 1:numel(p.calib.d)
    traj = make_trajectory(p.calib.d(j), p.sim.T_move, p.sim.dt, p);
    for i = 1:numel(p.calib.f)
        tau_total = compute_required_torque(traj, p.calib.f(i), p);
        D(i, j) = compute_difficulty(traj.t, tau_total);
    end
end

D_lo  = min(D(:));
D_hi  = max(D(:));
E_low = (1 - p.ctrl.alpha_min) * D_lo;
p.ctrl.D_lo = D_lo;  p.ctrl.D_hi = D_hi;  p.band.E_low = E_low;
fprintf(['  calibration D range [%.2f, %.2f] N m s -> D_lo/D_hi set; ' ...
    'E_low = %.2f (E_high = %.2f a priori)\n'], D_lo, D_hi, E_low, ...
    p.band.E_high);

outdir = fileparts(cachefile);
if ~exist(outdir, 'dir'), mkdir(outdir); end
save(cachefile, 'key', 'D_lo', 'D_hi', 'E_low', 'D');
end
