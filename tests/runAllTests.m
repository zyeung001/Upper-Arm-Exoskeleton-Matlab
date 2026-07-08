%RUNALLTESTS  Sanity checks for the model (stub).
%
%   Run `setupPath` first, then `runAllTests`. These are lightweight checks to
%   catch mistakes in the kinematics/dynamics as you edit them. Fill in more
%   assertions as the model grows.
%
%   TODO ideas:
%     - forwardKinematics: known q -> known hand position; reach <= L1+L2
%     - inverseKinematics: IK(FK(q)) recovers q
%     - massMatrix: symmetric, positive definite for random q
%     - gravityVector: matches numerical gradient of potential energy
%     - fittsIndex: log2(D/W + 1) for a couple of hand values

setupPath;
params = armParameters();

% Example check: IK inverts FK.
q     = [0.3; 0.4; 0.8];
pHand = forwardKinematics(q, params);
qBack = inverseKinematics(pHand, params, params.qHome);
assert(norm(forwardKinematics(qBack, params) - pHand) < 1e-3, ...
       'IK did not recover the FK target position.');

fprintf('runAllTests: basic FK/IK check passed. (Add more tests -- TODO.)\n');
