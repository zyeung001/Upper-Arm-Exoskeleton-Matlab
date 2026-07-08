%DEMOSINGLEREACH  Minimal example: one reach, fixed vs adaptive controller.
%
%   Run `setupPath` first, then run this script. It is intended as a starting
%   point / sanity check -- flesh it out as the project develops.
%
%   TODO:
%     - pick a target and simulate with both controllers
%     - plot hand path vs desired, and torque histories
%     - print effort + tracking metrics side by side

setupPath;
params  = armParameters();
targets = targetObjects(params);

target = targets(1);   % TODO: choose / loop over targets

% --- Fixed PID ---
ctrlFixed.type  = 'fixed';
ctrlFixed.gains = controllerGains('fixed');
resFixed = simulateReach(target, params, ctrlFixed);

% --- Adaptive PID ---
ctrlAdapt.type  = 'adaptive';
ctrlAdapt.gains = controllerGains('adaptive');
resAdapt = simulateReach(target, params, ctrlAdapt);

% --- TODO: plotting + comparison printout ---
fprintf('Target "%s":  D=%.2f m  W=%.3f m  ID=%.2f bits\n', ...
        target.name, target.D, target.W, target.ID);
fprintf('  fixed    : final err %.4f m | effort %.1f\n', ...
        resFixed.finalError, resFixed.effort.integralSqTorque);
fprintf('  adaptive : final err %.4f m | effort %.1f\n', ...
        resAdapt.finalError, resAdapt.effort.integralSqTorque);
