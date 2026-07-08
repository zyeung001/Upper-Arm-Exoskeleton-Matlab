function [q, info] = inverseKinematics(target, params, q0)
%INVERSEKINEMATICS Solve for joint angles that place the hand at a target.
%
%   q = inverseKinematics(target, params) returns joint angles q such that
%   forwardKinematics(q, params) ~ target (a 3x1 world position).
%
%   q = inverseKinematics(target, params, q0) seeds the solver at q0
%   (defaults to params.qHome).
%
%   [q, info] = ... also returns info.err (final position error, m) and
%   info.iters (iterations used).
%
%   Method: damped least-squares (Levenberg-Marquardt) Newton iteration on
%   the hand position Jacobian, with joint-limit clamping. Damping keeps the
%   step well behaved near kinematic singularities (e.g. full extension).

if nargin < 3 || isempty(q0)
    q0 = params.qHome;
end

q       = q0(:);
lambda  = 0.05;     % damping factor
tol     = 1e-4;     % position tolerance (m)
maxIter = 200;
err     = inf;

for it = 1:maxIter
    kin = computeKinematics(q, params);
    e   = target(:) - kin.p_hand;
    err = norm(e);
    if err < tol
        break;
    end
    J = zeros(3, 3);
    for i = 1:3
        J(:, i) = cross(kin.z(:, i), kin.p_hand - kin.o(:, i));
    end
    dq = (J' * J + lambda^2 * eye(3)) \ (J' * e);
    q  = q + dq;
    q  = max(min(q, params.qMax), params.qMin);   % respect joint limits
end

info.err   = err;
info.iters = it;
if err > 10 * tol
    warning('inverseKinematics:notConverged', ...
        'IK did not fully converge (residual %.4g m). Target may be out of reach.', err);
end
end
