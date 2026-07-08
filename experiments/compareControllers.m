%COMPARECONTROLLERS  Aggregate trial results and compare fixed vs adaptive.
%
%   Run after runTrials has produced results/trials.mat. Loads the per-trial
%   metrics and compares the control group (fixed PID) against the
%   experimental group (adaptive PID) as a function of task difficulty (ID).
%
%   TODO:
%     - load results/trials.mat
%     - plot actuator effort vs ID for both controllers
%     - plot tracking accuracy (RMS / final error) vs ID for both controllers
%     - report the win/lose summary (which controller lower effort, better
%       accuracy, and at which difficulty levels)

setupPath;

% TODO: S = load(fullfile('results','trials.mat'));
% TODO: group by controller, plot vs ID, save figures to results/figures/

fprintf('compareControllers: TODO -- load results and plot effort/accuracy vs ID.\n');
