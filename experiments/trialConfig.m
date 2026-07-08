function cfg = trialConfig()
%TRIALCONFIG  Experiment design: the distances, widths and controllers to test.
%
%   cfg = trialConfig() returns the configuration for the trial sweep. This is
%   where the study is defined -- edit it to change which reaches are tested.
%
%   TODO: tune these values for your protocol.

% Distances (m) and widths (m) to sweep -> each (D,W) pair is a task with a
% Fitts index of difficulty ID = log2(D/W + 1).
cfg.distances = [0.20, 0.30, 0.40];      % TODO
cfg.widths    = [0.02, 0.05, 0.09];      % TODO

% Controllers to compare: 'fixed' = control group, 'adaptive' = experimental.
cfg.controllers = {'fixed', 'adaptive'};

% Repetitions per condition (for averaging / noise, once noise is added).
cfg.reps = 1;                            % TODO

% Simulation options passed through to simulateReach.
cfg.sim = struct('dt', 0.002, 'T', 2.5, 'moveTime', 1.2);
end
