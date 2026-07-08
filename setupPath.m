function setupPath()
%SETUPPATH Add all project source folders to the MATLAB path.
%   Run this once at the start of a session:
%       >> setupPath
%
%   It adds src/ (and every subfolder) plus experiments/ and tests/ so that
%   every function in the project is callable from anywhere.

thisDir = fileparts(mfilename('fullpath'));

addpath(genpath(fullfile(thisDir, 'src')));
addpath(fullfile(thisDir, 'experiments'));
addpath(fullfile(thisDir, 'tests'));

% Make sure output folders exist.
if ~exist(fullfile(thisDir, 'results'), 'dir')
    mkdir(fullfile(thisDir, 'results'));
end
if ~exist(fullfile(thisDir, 'results', 'figures'), 'dir')
    mkdir(fullfile(thisDir, 'results', 'figures'));
end

fprintf('Upper-arm exoskeleton project on path. Try: demoSingleReach\n');
end
