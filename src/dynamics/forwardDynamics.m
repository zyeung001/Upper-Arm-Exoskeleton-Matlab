function qddot = forwardDynamics(q, qdot, tau, params)
%FORWARDDYNAMICS Joint accelerations given applied torques.
%
%   qddot = forwardDynamics(q, qdot, tau, params) solves the manipulator
%   equation for the accelerations:
%
%       qddot = M(q) \ ( tau - C(q,qdot)*qdot - G(q) )
%
%   This is the plant that the simulation integrates: given the controller
%   torque tau (plus gravity/Coriolis coupling), it returns how the arm
%   actually accelerates.

M = massMatrix(q, params);
C = coriolisMatrix(q, qdot, params);
G = gravityVector(q, params);

qddot = M \ (tau(:) - C * qdot(:) - G);
end
