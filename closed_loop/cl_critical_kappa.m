function [kcrit, pk] = cl_critical_kappa(p, cl, laws)
%CL_CRITICAL_KAPPA The weakest user each assistance law never over-demands.
%   [kcrit, pk] = CL_CRITICAL_KAPPA(p, cl, laws)
%
%   For each law, the smallest strength fraction kappa (peak joint torque as
%   a fraction of healthy MVC) at which the law still never asks the human
%   for more torque than they can produce, on ANY task:
%
%       kappa_crit(law) = max over tasks, max over joints
%                             |(1 - alpha) * tau_j(t)| / MVC_j
%
%   A user below a law's critical kappa gets asked for torque they do not
%   have; the shortfall is never delivered, the applied torque falls short
%   of the task, and tracking degrades.
%
%   This is the strength-cap result that CANNOT be gamed: it is a property
%   of each law, not a parameter anyone chose. It is computed
%     - a priori, from inverse dynamics along the reference (no closed-loop
%       simulation, no measured outcome), and
%     - on the CALIBRATION grid only, never the evaluation grid, so it
%       respects the study's train/test split and can legitimately be used
%       to set the sweep range that IS evaluated on held-out tasks.
%
%   kcrit (1 x nl), pk (3 x nl) worst per-joint demand (N m).

if nargin < 3, laws = {'fixed', 'fill', 'difficulty'}; end
nl = numel(laws);
MVC = cl.MVC(:);

kcrit = zeros(1, nl);
pk    = zeros(3, nl);
for j = 1:numel(p.calib.d)
    traj = make_trajectory(p.calib.d(j), p.sim.T_move, p.sim.dt, p);
    for i = 1:numel(p.calib.f)
        f = p.calib.f(i);
        tau_total = compute_required_torque(traj, f, p);
        D = compute_difficulty(traj.t, tau_total);
        for L = 1:nl
            [~, ~, a] = controllers(laws{L}, tau_total, f, D, p);
            pk_j     = max(abs((1 - a) * tau_total), [], 2);   % 3x1
            pk(:, L) = max(pk(:, L), pk_j);
            kcrit(L) = max(kcrit(L), max(pk_j ./ MVC));
        end
    end
end
end
