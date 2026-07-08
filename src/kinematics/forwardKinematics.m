function [pHand, pElbow, kin] = forwardKinematics(q, params)
%FORWARDKINEMATICS Hand (and elbow) position for a joint configuration.
%
%   pHand = forwardKinematics(q, params) returns the 3-D world position of
%   the hand (end effector) for joint angles q = [q1; q2; q3].
%
%   [pHand, pElbow, kin] = forwardKinematics(q, params) also returns the
%   elbow position and the full kinematics struct (see computeKinematics).
%
%   Forward kinematics starts at the shoulder and walks out to the end
%   effector, accumulating each joint rotation -- this is what converts the
%   measured joint angles into the 3-D hand position we track against a target.

kin = computeKinematics(q, params);
pHand  = kin.p_hand;
pElbow = kin.p_elbow;
end
