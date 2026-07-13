function dx = cl_plant_deriv(x, u, pvec, b_s, rigid)
%CL_PLANT_DERIV State derivative of the full nonlinear 4-DOF plant.
%   dx = CL_PLANT_DERIV(x, u, pvec, b_s, rigid)
%
%   x     = [q; qd], q = [th1 th2 th3 phi]  (8x1)
%   u     = actuated joint torque (3x1) - exo + human total
%   pvec  = 15-element parameter vector (pack_pvec)
%   b_s   = slosh damping coefficient (N m s), b_s = 2*zeta_s*w_s*m_s*L_s^2
%           so the phi row linearizes to the same damped surrogate used by
%           compute_required_torque (phi'' + 2*zeta_s*w_s*phi' + ...).
%   rigid = 0/1 flag. 1 = "F_water = 0" case: the liquid is frozen (phi
%           locked at 0 by a constraint, payload mass still carried), so
%           the arm feels the weight but no slosh disturbance.
%
%   Called from the Plant MATLAB Function block in arm_closed_loop.slx
%   (whiteboard: "State Space or MATLAB Function" - the true nonlinear
%   M qdd + C qd + G = [u; F_water] model, not a linearization).
%#codegen
x = x(:);
u = u(:);
pvec = reshape(pvec, 1, []);   % generated funcs index pvec as a row
q  = x(1:4);
qd = x(5:8);
if rigid > 0.5
    % phi constrained to 0: reduced 3-DOF dynamics at phi = 0, phid = 0
    % (constraint torque absorbs the phi row).
    q4  = [q(1:3); 0];
    qd4 = [qd(1:3); 0];
    M = M_fun(q4, pvec);
    C = C_fun(q4, qd4, pvec);
    G = G_fun(q4, pvec);
    qdd3 = M(1:3, 1:3) \ (u - C(1:3, :)*qd4 - G(1:3));
    dx = [qd(1:3); 0; qdd3; 0];
else
    M = M_fun(q, pvec);
    C = C_fun(q, qd, pvec);
    G = G_fun(q, pvec);
    tau4 = [u; -b_s * qd(4)];   % phi is unactuated; only damping acts on it
    qdd  = M \ (tau4 - C*qd - G);
    dx = [qd; qdd];
end
end
