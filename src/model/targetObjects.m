function targets = targetObjects(params, defs)
%TARGETOBJECTS Build reach targets ("objects") with Fitts difficulty.
%
%   targets = targetObjects(params) returns a struct array describing a small
%   default set of graspable objects, each modelled as a box/target at a 3-D
%   world position with a width W (the tolerance the hand must land within).
%
%   targets = targetObjects(params, defs) builds targets from a custom cell
%   array with rows { name, [x;y;z], W }.
%
%   Each element has fields:
%       .name   label
%       .pos    3x1 target-centre position (m), reachable by the arm
%       .W      target width / tolerance (m)  -> Fitts' W
%       .D      distance from the home hand position (m) -> Fitts' D
%       .ID     Fitts index of difficulty = log2(D/W + 1)
%       .size   [x y z] box dimensions for visualisation (defaults to W cube)
%
%   D is measured from the hand position at params.qHome, so ID reflects the
%   reach the arm actually has to perform. Distances and widths are the two
%   knobs the experiment sweeps.

if nargin < 2 || isempty(defs)
    % name              position [x;y;z] (m)     width W (m)
    defs = {
        'mug',          [0.34;  0.06; -0.05],    0.09
        'phone',        [0.30; -0.10;  0.02],    0.06
        'pen',          [0.28;  0.14;  0.04],    0.02
        'bottle',       [0.40;  0.00; -0.02],    0.07
        'small_switch', [0.25;  0.18;  0.10],    0.015
        'far_box',      [0.46; -0.05;  0.05],    0.05
    };
end

home = forwardKinematics(params.qHome, params);

n = size(defs, 1);
targets = repmat(struct('name', '', 'pos', [], 'W', [], 'D', [], ...
                        'ID', [], 'size', []), 1, n);

for k = 1:n
    pos = defs{k, 2}(:);
    W   = defs{k, 3};
    D   = norm(pos - home);

    targets(k).name = defs{k, 1};
    targets(k).pos  = pos;
    targets(k).W    = W;
    targets(k).D    = D;
    targets(k).ID   = fittsIndex(D, W);
    targets(k).size = [W W W];        % simple cube for plotting
end
end
