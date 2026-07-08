function M = trackingError(t, actualHand, desiredHand, qErr)
%TRACKINGERROR Tracking-accuracy measures for a reach.
%
%   M = trackingError(t, actualHand, desiredHand) compares the actual hand
%   path to the desired hand path (both Nx3 world positions) and returns:
%
%     .rmsCartesian    RMS Euclidean hand-position error over the trial (m)
%     .maxCartesian    worst-case hand-position error (m)
%     .finalCartesian  hand-position error at the final sample (m)
%
%   M = trackingError(t, actualHand, desiredHand, qErr) additionally reports
%     .rmsJoint        RMS joint-angle error (rad), from qErr (Nx3 = q - qref)
%
%   These quantify tracking accuracy: how closely the exoskeleton kept the
%   hand on the intended path to the target.

d   = actualHand - desiredHand;     % Nx3 position error
nrm = sqrt(sum(d.^2, 2));           % Euclidean error per sample

M.rmsCartesian   = sqrt(mean(nrm.^2));
M.maxCartesian   = max(nrm);
M.finalCartesian = nrm(end);

if nargin > 3 && ~isempty(qErr)
    M.rmsJoint = sqrt(mean(sum(qErr.^2, 2)));
end
end
