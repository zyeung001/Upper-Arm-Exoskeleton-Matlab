function M = massMatrix(q, params)
%MASSMATRIX Joint-space inertia (mass) matrix M(q) of the arm.
%
%   M = massMatrix(q, params) returns the 3x3 symmetric positive-definite
%   inertia matrix appearing in the manipulator equation
%
%       M(q)*qddot + C(q,qdot)*qdot + G(q) = tau
%
%   It is assembled from the link centre-of-mass Jacobians (Lagrangian /
%   kinetic-energy formulation):
%
%       M = sum_i [ Jvi' * m_i * Jvi  +  Jwi' * Ic_i * Jwi ]
%
%   where Ic_i is each link's inertia tensor rotated into the world frame.

kin = computeKinematics(q, params);
[Jv1, Jv2, Jw1, Jw2] = linkJacobians(kin);

% Link inertia tensors expressed in the world frame.
Ic1 = kin.R1 * params.I1 * kin.R1';
Ic2 = kin.R2 * params.I2 * kin.R2';

M = Jv1' * params.m1 * Jv1 + Jw1' * Ic1 * Jw1 ...
  + Jv2' * params.m2 * Jv2 + Jw2' * Ic2 * Jw2;

M = 0.5 * (M + M');   % clean up numerical asymmetry
end
