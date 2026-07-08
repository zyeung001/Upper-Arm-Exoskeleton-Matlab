function tau = inverseDynamics(q, qdot, qddot, params)
%INVERSEDYNAMICS Joint torques required to produce a desired acceleration.
%
%   tau = inverseDynamics(q, qdot, qddot, params) evaluates the manipulator
%   equation directly:
%
%       tau = M(q)*qddot + C(q,qdot)*qdot + G(q)
%
%   Useful for computed-torque control, feedforward terms, and for measuring
%   the actuator effort implied by a reference trajectory.

M = massMatrix(q, params);
C = coriolisMatrix(q, qdot, params);
G = gravityVector(q, params);

tau = M * qddot(:) + C * qdot(:) + G;
end
