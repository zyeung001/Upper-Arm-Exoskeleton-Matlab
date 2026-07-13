function cl = cl_params()
%CL_PARAMS All constants for the closed-loop Simulink validation study.
%   cl = CL_PARAMS() returns a struct. Same convention as params.m:
%   nothing downstream hard-codes numbers.
%
%   This module is SEPARATE from the open-loop study pipeline (main.m):
%   the sweep's inverse-dynamics/no-tracking-loop design is unchanged.
%   Here a PD + gravity-compensation tracking controller closes the loop
%   around the full nonlinear 4-DOF plant (arm + slosh pendulum), per the
%   mentor's block diagram, to validate that the assistance-split results
%   survive realistic tracking with the slosh acting as a true two-way
%   disturbance.

% ---- Tracking controller (PD + gravity feedforward) ----------------------
% Gains give ~10 rad/s bandwidth / near-critical damping against the
% arm+payload inertia scale (~0.5-0.9 kg m^2 seen at the shoulder):
% wn ~ sqrt(Kp/M), zeta ~ Kd / (2*sqrt(Kp*M)).
cl.Kp = diag([90 90 90]);    % N m / rad
cl.Kd = diag([14 14 14]);    % N m s / rad

% ---- Simulation window ----------------------------------------------------
% The carry itself lasts p.sim.T_move = 1 s; the reference then HOLDS the
% target pose so the settling phase (slosh ring-down, residual tracking
% error) is part of every run and of the time-to-settle metric.
cl.Tend = 5.0;               % s (slosh decay time 1/(zeta_s*w_s) ~ 1.4 s)

% ---- Metrics ---------------------------------------------------------------
cl.tol_ee = 0.01;            % settle tolerance on end-effector error (m)

% ---- Solver ----------------------------------------------------------------
cl.solver  = 'ode45';
cl.RelTol  = 1e-6;
cl.AbsTol  = 1e-9;
cl.MaxStep = 0.005;          % s - resolve the ~14 rad/s slosh mode densely

% ---- Default task (matches animate_carry's default: fairly hard) ---------
cl.f_default = 0.8;          % fill level
cl.d_default = 0.4;          % reach distance (m)
end
