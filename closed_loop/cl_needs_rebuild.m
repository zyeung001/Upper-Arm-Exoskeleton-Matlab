function tf = cl_needs_rebuild(mdlfile)
%CL_NEEDS_REBUILD Is the gitignored .slx missing or older than its source?
%   tf = CL_NEEDS_REBUILD(mdlfile)
%
%   arm_closed_loop.slx is a build artifact; build_closed_loop_model.m is
%   its source of truth. Editing the builder and forgetting Rebuild=true
%   would silently simulate the OLD model - the same stale-cache trap the
%   repo already guards against for generated/ and results/calibration.mat.
%   cl_params.m counts as source too: the model embeds its solver settings.

% isfile, not exist(...,'file'): exist returns 4 (not 2) for a .slx, so an
% exist == 2 test would report "missing" for a model that is right there.
if ~isfile(mdlfile)
    tf = true;
    return
end
here = fileparts(mfilename('fullpath'));
src  = {fullfile(here, 'build_closed_loop_model.m'), ...
        fullfile(here, 'cl_params.m')};
d_mdl = dir(mdlfile);
tf = false;
for k = 1:numel(src)
    d = dir(src{k});
    if ~isempty(d) && d.datenum > d_mdl.datenum
        tf = true;
        return
    end
end
end
