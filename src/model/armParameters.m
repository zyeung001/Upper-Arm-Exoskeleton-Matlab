function params = armParameters()
%ARMPARAMETERS Physical parameters of the 3-DOF upper-arm + exoskeleton model.
%
%   params = armParameters() returns a struct holding link lengths, masses,
%   inertias, gravity, the home configuration and joint limits. Every other
%   function in the project takes this struct so the model is defined in one
%   place -- edit here to retune the arm.
%
%   The default numbers are rough adult-arm segment values (de Leva / Winter
%   anthropometry, upper arm + forearm-and-hand lumped) combined with a light
%   exoskeleton. Treat them as a starting point and adjust for your subject.

params.g = 9.81;                 % gravitational acceleration (m/s^2)

% ---- Link lengths (m) --------------------------------------------------
params.L1 = 0.30;                % upper arm (shoulder -> elbow)
params.L2 = 0.26;                % forearm + hand (elbow -> hand point)

% ---- Centre-of-mass distances from the proximal joint (m) --------------
params.lc1 = 0.15;
params.lc2 = 0.13;

% ---- Segment masses (kg), arm + worn exoskeleton lumped per link -------
params.m1 = 2.2;                 % upper arm + cuff
params.m2 = 1.7;                 % forearm + hand + cuff

% ---- Segment inertia tensors about their own COM (kg*m^2) --------------
% Slender-rod approximation about the local frame (z along the link).
r1 = 0.045; r2 = 0.040;          % effective segment radii (m)
params.I1 = rodInertia(params.m1, params.L1, r1);
params.I2 = rodInertia(params.m2, params.L2, r2);

% ---- Home configuration (rad): relaxed arm, slight flexion -------------
params.qHome = [0.20; 0.10; 0.50];

% ---- Joint limits (rad) ------------------------------------------------
params.qMin = [-1.50; -0.30; 0.00];   % [flex, abduct, elbow]
params.qMax = [ 2.50;  2.50; 2.60];

% ---- Actuator torque limit per joint (N*m) ----------------------------
params.tauMax = [40; 40; 25];
end


function I = rodInertia(m, L, r)
%RODINERTIA Inertia tensor of a uniform cylinder about its COM, axis along z.
Ixx = (1/12) * m * L^2 + (1/4) * m * r^2;   % transverse axes
Izz = 0.5 * m * r^2;                        % longitudinal axis
I = diag([Ixx, Ixx, Izz]);
end
