function traj = make_trajectory(d, T_move, dt, p)
%MAKE_TRAJECTORY Quintic joint-space reference for a carry of distance d.
%   traj = MAKE_TRAJECTORY(d, T_move, dt, p)
%
%   The start and target END-EFFECTOR positions are separated by exactly d
%   (target = p.reach.start + d*p.reach.dir). Both endpoints are mapped to
%   joint angles by closed-form inverse kinematics (documented choice:
%   endpoints in Cartesian space, interpolation in JOINT space), then a
%   quintic polynomial with zero end velocity/acceleration interpolates
%   each joint over [0, T_move]. Decision A: T_move is the same for every
%   d, so longer reaches are faster and excite more slosh.
%
%   Output struct: t (1xN), q (3xN), qd (3xN), qdd (3xN)

P0 = p.reach.start;
P1 = P0 + d * p.reach.dir;
q0 = ik_3dof(P0, p);
q1 = ik_3dof(P1, p);

t   = 0:dt:T_move;
tau = t / T_move;
s   = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
sd  = (30*tau.^2 - 60*tau.^3 + 30*tau.^4) / T_move;
sdd = (60*tau - 180*tau.^2 + 120*tau.^3) / T_move^2;

dq = q1 - q0;
traj.t   = t;
traj.q   = q0 + dq * s;
traj.qd  = dq * sd;
traj.qdd = dq * sdd;
end

% ------------------------------------------------------------------------
function q = ik_3dof(P, p)
%Closed-form IK for the 3-DOF arm (elbow-DOWN branch: the elbow stays below
%the shoulder, matching how a human carries a cup), asserted against the
%forward kinematics exported by derive_dynamics.
a = p.arm;
th3 = atan2(P(2), P(1));                 % shoulder rotation (vertical axis)
r   = hypot(P(1), P(2)) - a.L3;          % in-plane horizontal dist. from shoulder
z   = P(3);
rr  = hypot(r, z);
assert(r > 0.05 && rr < 0.98*(a.L1 + a.L2) && rr > abs(a.L1 - a.L2) + 0.02, ...
    'ik_3dof: target [%g %g %g] outside comfortable workspace', P);
c2  = (rr^2 - a.L1^2 - a.L2^2) / (2*a.L1*a.L2);
th2 = acos(max(-1, min(1, c2)));         % elbow-down in this convention
th1 = atan2(z, r) - atan2(a.L2*sin(th2), a.L1 + a.L2*cos(th2));
q   = [th1; th2; th3];

err = norm(ee_fun(q, pack_pvec(p, 0, 0)) - P);
assert(err < 1e-9, 'ik_3dof: FK/IK mismatch (%.3g m)', err);
end
