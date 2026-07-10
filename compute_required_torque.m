function [tau_total, peak_slosh, slosh] = compute_required_torque(traj, f, p)
%COMPUTE_REQUIRED_TORQUE Slosh forward-simulation + inverse dynamics.
%   [tau_total, peak_slosh, slosh] = COMPUTE_REQUIRED_TORQUE(traj, f, p)
%
%   The slosh angle phi is the only state integrated forward in time: it is
%   a damped second-order system forced by the end-effector acceleration
%   (linearized pendulum-equivalent, same form as Bai et al. 2025,
%   theta'' + 2*delta*w_n*theta' + w_n^2*theta = -a_c/R):
%
%       phi'' + 2*zeta_s*w_s*phi' + w_s^2*phi = -a_h(t)/L_s
%
%   where a_h is the horizontal end-effector acceleration in the pendulum's
%   swing plane (the arm's vertical plane; out-of-plane slosh is neglected,
%   a documented single-DOF simplification). The arm itself follows the
%   reference by construction - no feedback tracking loop (spec section 0).
%
%   tau_total (3xN) is then the inverse dynamics of the FULL 4-DOF model
%   (arm + pendulum) along [q_ref; phi], actuated rows only.
%
%   Outputs: tau_total (3xN), peak_slosh = max|phi| (rad, spill proxy),
%   slosh struct with the time series phi, phid and the surrogate
%   parameters w_s, m_s, L_s (used by animate_carry).

% Fill-dependent pendulum surrogate
m_liq = p.cont.rho * pi * p.cont.Rc^2 * (f * p.cont.Hc);
m_s   = p.slosh.k_m * m_liq;
L_s   = p.slosh.L_s_factor * p.cont.Rc;   % slosh length scale (default Rc)
w_s   = sqrt(p.g / L_s);
zs    = p.slosh.zeta_s;
pvec  = pack_pvec(p, m_s, L_s);

t = traj.t;  N = numel(t);

% Horizontal in-plane forcing acceleration along the reference
a_h = zeros(1, N);
for k = 1:N
    a_ee = aee_fun(traj.q(:,k), traj.qd(:,k), traj.qdd(:,k), pvec);
    th3  = traj.q(3,k);
    a_h(k) = cos(th3)*a_ee(1) + sin(th3)*a_ee(2);
end

% Slosh forward-simulation (ode45), zero initial state
a_of_t = griddedInterpolant(t, a_h, 'pchip');
ode = @(tt, x) [x(2); -2*zs*w_s*x(2) - w_s^2*x(1) - a_of_t(tt)/L_s];
[~, X] = ode45(ode, t, [0; 0], odeset('RelTol', 1e-8, 'AbsTol', 1e-10));
phi  = X(:,1).';
phid = X(:,2).';
phidd = -2*zs*w_s*phid - w_s^2*phi - a_h/L_s;
peak_slosh = max(abs(phi));

% Inverse dynamics of the full model along [q_ref; phi], actuated rows
tau_total = zeros(3, N);
for k = 1:N
    qf   = [traj.q(:,k);   phi(k)];
    qdf  = [traj.qd(:,k);  phid(k)];
    qddf = [traj.qdd(:,k); phidd(k)];
    tau  = M_fun(qf, pvec)*qddf + C_fun(qf, qdf, pvec)*qdf + G_fun(qf, pvec);
    tau_total(:,k) = tau(1:3);
end

slosh = struct('phi', phi, 'phid', phid, 'w_s', w_s, 'm_s', m_s, 'L_s', L_s);
end
