%MAIN Ties everything together: derive dynamics once, run the sweep, plot.
%   Task-difficulty-aware adaptive assistance for an upper-limb exoskeleton
%   carrying a sloshing liquid. See README.md and the build spec.

clear; clc;
root = fileparts(mfilename('fullpath'));

% Derive symbolic dynamics once (skipped if already exported).
% Delete generated/ or run derive_dynamics() manually to force re-derivation.
if exist(fullfile(root, 'generated', 'M_fun.m'), 'file') ~= 2
    derive_dynamics();   % also adds generated/ to the path
end
addpath(fullfile(root, 'generated'));

p = params();
p = calibrate_controller(p);   % tune ramp + band floor on the held-out grid
R = run_sweep(p);
make_figures(R);
