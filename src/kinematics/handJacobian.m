function [Jp, kin] = handJacobian(q, params)
%HANDJACOBIAN Position Jacobian of the hand (end effector).
%
%   Jp = handJacobian(q, params) returns the 3x3 Jacobian mapping joint
%   velocities to hand linear velocity:   v_hand = Jp * qdot.
%
%   [Jp, kin] = handJacobian(q, params) also returns the kinematics struct.
%
%   Column i is  z_i x (p_hand - o_i)  for each revolute joint -- the same
%   geometric-Jacobian construction used for the link centres of mass in the
%   dynamics.  This Jacobian is what relates joint torques to end-effector
%   forces, and is used by the numerical inverse kinematics.

kin = computeKinematics(q, params);
Jp = zeros(3, 3);
for i = 1:3
    Jp(:, i) = cross(kin.z(:, i), kin.p_hand - kin.o(:, i));
end
end
