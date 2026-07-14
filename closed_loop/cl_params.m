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

% ---- Human torque-development lag -----------------------------------------
% The human delivers their share through first-order dynamics
%   tau_h * u_hum_dot = u_hum_cmd - u_hum
% (surrogate for neuromuscular activation / torque-development dynamics,
% effective time constants ~40-150 ms in the literature, cf. Zajac 1989);
% the exo responds instantly by comparison. This is what makes the alpha
% split DYNAMICALLY consequential in closed loop: laws that leave more
% share on the human track worse. Fixed A PRIORI - never tuned to results.
% The ideal-human run (lag bypassed) is also simulated as the cross-check
% against the open-loop sweep.
cl.tau_h = 0.10;             % s

% ---- Human strength cap (what makes the alpha law change the TASK) ---------
% The human is not an unlimited torque source: they can deliver at most
% u_h_max per joint. Beyond it the demanded share is simply not produced,
% the applied total falls short of what the task needs, and tracking
% degrades - so under-support stops being merely "tiring" and becomes a
% performance failure. Implemented as a saturation on the lag integrator's
% STATE (muscle activation cannot exceed 1), which bounds the delivered
% torque without integrator windup; the exo is unbounded by comparison.
%
% MVC = healthy maximum voluntary isometric torque, in the joint order of
% params.m: th1 shoulder flexion, th2 elbow flexion, th3 shoulder rotation
% about the vertical axis. Order-of-magnitude adult values.
cl.MVC = [50; 45; 30];       % N m
%
% kappa = the user's strength as a fraction of MVC - a SCENARIO parameter
% describing WHO wears the exo (a weakened user), never a tuned one. The
% headline is the kappa SWEEP, not any single value; cl.kappa is only the
% representative scenario for the single-task run and the grid table.
cl.kappa   = 0.08;                    % ~8% MVC: a severely weakened user
cl.u_h_max = cl.kappa * cl.MVC;       % N m per joint (3x1)
%
% Sweep range, set by a rule on CALIBRATION-GRID data only (the train/test
% split holds: the evaluation grid is never consulted to choose it). A law's
% A-PRIORI CRITICAL KAPPA - the weakest user it never over-demands on any
% calibration task, max_task max_joint |(1-alpha)*tau_j| / MVC_j - is 9.1%
% (fixed), 8.5% (fill), 6.4% (difficulty). The sweep straddles that window,
% from "the cap binds every law" to "it binds none". Those critical kappas
% are themselves the tuning-free result: the difficulty law never
% over-demands a user ~30% weaker than the fixed law can serve.
cl.kappa_sweep = [0.05, 0.065, 0.08, 0.095, 0.11];
%
% A joint counts as saturated within this relative tolerance of the cap.
cl.sat_tol = 1e-3;

% ---- Simulation window ----------------------------------------------------
% The carry itself lasts p.sim.T_move = 1 s; the reference then HOLDS the
% target pose so the settling phase (slosh ring-down, residual tracking
% error) is part of every run and of the time-to-settle metric.
cl.Tend = 5.0;               % s (slosh decay time 1/(zeta_s*w_s) ~ 1.4 s)

% ---- Metrics ---------------------------------------------------------------
cl.tol_ee = 0.01;            % settle tolerance on end-effector error (m)

% ---- Closed-loop grid evaluation (run_closed_loop_grid) --------------------
% A coarse grid SPANNING THE EXTREMES of the sweep's task space - the laws
% only disagree at the corners (fixed over-assists easy tasks, under-
% supports hard ones), so a single mid-range task cannot separate them.
% One closed-loop simulation per task per law; band status is the metric,
% mirroring the sweep's primary question.
cl.grid_f = [0.1, 0.55, 1.0];    % fill levels
cl.grid_d = [0.1, 0.3, 0.5];     % reach distances (m)
cl.Tend_grid = 2.0;              % s - carry + enough hold for band metrics

% ---- Solver ----------------------------------------------------------------
cl.solver  = 'ode45';
cl.RelTol  = 1e-6;
cl.AbsTol  = 1e-9;
cl.MaxStep = 0.005;          % s - resolve the ~14 rad/s slosh mode densely

% ---- Default task (matches animate_carry's default: fairly hard) ---------
cl.f_default = 0.8;          % fill level
cl.d_default = 0.4;          % reach distance (m)
end
