%RUNTRIALS  Sweep the trial matrix and collect effort + accuracy per task.
%
%   Run `setupPath` first. This script loops over the (distance x width x
%   controller) grid from trialConfig, simulates each reach, and records the
%   metrics so the two controllers can be compared across difficulty.
%
%   TODO:
%     - build a real target for each (D, W) pair (place it in the workspace at
%       the required distance, give it width W)
%     - run simulateReach for each controller
%     - store ID, effort, tracking accuracy, hit/miss into a table
%     - save the table to results/ for compareControllers to plot

setupPath;
params = armParameters();
cfg    = trialConfig();

results = [];   % TODO: accumulate a table/struct array of per-trial metrics

for iD = 1:numel(cfg.distances)
    for iW = 1:numel(cfg.widths)
        D  = cfg.distances(iD);
        W  = cfg.widths(iW);
        ID = fittsIndex(D, W);

        % TODO: create a target at distance D with width W, then:
        %   for each controller in cfg.controllers
        %       res = simulateReach(target, params, controller, cfg.sim);
        %       append (D, W, ID, controller, res.effort..., res.tracking...)

        fprintf('D=%.2f W=%.3f -> ID=%.2f  (TODO: simulate)\n', D, W, ID);
    end
end

% TODO: save(fullfile('results','trials.mat'), 'results');
