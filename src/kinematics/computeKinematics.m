function kin = computeKinematics(q, params)
%COMPUTEKINEMATICS Full forward kinematics of the 3-DOF arm.
%
%   kin = computeKinematics(q, params) returns a struct describing the pose
%   of the whole kinematic chain for joint configuration q = [q1; q2; q3].
%
%   Joint / frame convention (world frame: x forward, y left, z up; the arm
%   hangs down the -z axis at the zero configuration):
%       q1  shoulder flexion / extension   (rotation about world y-axis)
%       q2  shoulder abduction / adduction (rotation about the q1-rotated x-axis)
%       q3  elbow flexion                  (rotation about the upper-arm y-axis)
%
%   Two links:
%       link 1 = upper arm  (length L1), link 2 = forearm+hand (length L2)
%
%   Output fields:
%       .R1, .R2      orientation of upper arm / forearm in world frame (3x3)
%       .p_elbow      elbow position                                    (3x1)
%       .p_hand       hand (end-effector) position                     (3x1)
%       .pc1, .pc2    centre-of-mass positions of the two links        (3x1)
%       .z            [z1 z2 z3] joint axes in world frame              (3x3)
%       .o            [o1 o2 o3] joint origins in world frame           (3x3)
%
%   The joint axes .z and origins .o are all that is needed to build any
%   geometric Jacobian:  Jcol_i = z_i x (p - o_i) for the joints upstream
%   of a point p.

q1 = q(1); q2 = q(2); q3 = q(3);

Ry1 = rotY(q1);
Rx2 = rotX(q2);

Rs = Ry1 * Rx2;      % shoulder orientation  -> upper-arm frame
Re = rotY(q3);       % elbow rotation (within upper-arm frame)

R1 = Rs;             % upper-arm orientation in world
R2 = Rs * Re;        % forearm orientation in world

% Link vectors point down the local -z axis.
uUpper = [0; 0; -params.L1];
vFore  = [0; 0; -params.L2];

p_elbow = R1 * uUpper;
p_hand  = p_elbow + R2 * vFore;

% Centre-of-mass positions (lc measured from the proximal joint).
pc1 = R1 * [0; 0; -params.lc1];
pc2 = p_elbow + R2 * [0; 0; -params.lc2];

% Joint axes in world frame.
z1 = [0; 1; 0];          % joint 1 about world y
z2 = Ry1 * [1; 0; 0];    % joint 2 about x, after q1 rotation
z3 = R1 * [0; 1; 0];     % joint 3 about upper-arm y

% Joint origins in world frame.
o1 = [0; 0; 0];
o2 = [0; 0; 0];
o3 = p_elbow;

kin.R1 = R1;  kin.R2 = R2;
kin.p_elbow = p_elbow;  kin.p_hand = p_hand;
kin.pc1 = pc1;  kin.pc2 = pc2;
kin.z = [z1 z2 z3];
kin.o = [o1 o2 o3];
end
