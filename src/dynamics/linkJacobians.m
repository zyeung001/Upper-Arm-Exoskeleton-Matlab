function [Jv1, Jv2, Jw1, Jw2] = linkJacobians(kin)
%LINKJACOBIANS Centre-of-mass velocity Jacobians for the two links.
%
%   [Jv1, Jv2, Jw1, Jw2] = linkJacobians(kin) takes a kinematics struct from
%   computeKinematics and returns the linear (Jv) and angular (Jw) velocity
%   Jacobians of each link's centre of mass:
%
%       v_ci   = Jvi * qdot        (3x3)
%       omega_i = Jwi * qdot       (3x3)
%
%   These feed the Lagrangian mass matrix and gravity vector. Link 1 (upper
%   arm) is driven only by the two shoulder joints; link 2 (forearm) by all
%   three. Linear columns use the geometric form  z_i x (p_ci - o_i);
%   angular columns are simply the joint axes z_i.

z = kin.z;
o = kin.o;

% --- Linear velocity Jacobians of the COMs ---
Jv1 = zeros(3, 3);
for i = 1:2                                   % upper-arm COM: shoulder joints
    Jv1(:, i) = cross(z(:, i), kin.pc1 - o(:, i));
end

Jv2 = zeros(3, 3);
for i = 1:3                                   % forearm COM: all joints
    Jv2(:, i) = cross(z(:, i), kin.pc2 - o(:, i));
end

% --- Angular velocity Jacobians ---
Jw1 = [z(:, 1), z(:, 2), [0; 0; 0]];
Jw2 = [z(:, 1), z(:, 2), z(:, 3)];
end
