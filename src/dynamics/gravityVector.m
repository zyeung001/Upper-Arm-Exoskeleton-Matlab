function G = gravityVector(q, params)
%GRAVITYVECTOR Gravity torque vector G(q) of the arm.
%
%   G = gravityVector(q, params) returns the 3x1 vector of joint torques
%   required to hold the arm against gravity in configuration q, i.e. the
%   G(q) term in  M*qddot + C*qdot + G = tau.
%
%   Derived from the potential energy  P = sum_i m_i * g * z_ci  (z up):
%       G_i = dP/dq_i = sum_link  m_link * g * (dz_ci/dq_i)
%   The vertical (z) row of each COM linear-velocity Jacobian is exactly
%   dz_ci/dq, so G reduces to a weighted sum of those rows.

kin = computeKinematics(q, params);
[Jv1, Jv2] = linkJacobians(kin);

g = params.g;
G = params.m1 * g * Jv1(3, :)' + params.m2 * g * Jv2(3, :)';
end
