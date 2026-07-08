function [ID, MT] = fittsIndex(D, W, a, b)
%FITTSINDEX Fitts' index of difficulty (and optional predicted movement time).
%
%   ID = fittsIndex(D, W) returns the Shannon-formulation index of difficulty
%
%       ID = log2( D / W + 1 )              [bits]
%
%   where D is the movement distance (amplitude) to the target and W is the
%   target width. Larger distance and/or smaller width => higher ID => harder
%   task. This is the single scalar "difficulty measure" that drives the
%   adaptive controller and organises the experimental trials.
%
%   [ID, MT] = fittsIndex(D, W, a, b) also returns the predicted movement
%   time from Fitts' law,  MT = a + b*ID  (defaults a = 0, b = 1). Fit a and b
%   to your own data to make MT meaningful; with the defaults MT == ID.
%
%   D and W may be scalars or matching-size arrays (element-wise).

if nargin < 3 || isempty(a), a = 0; end
if nargin < 4 || isempty(b), b = 1; end

if any(W(:) <= 0)
    error('fittsIndex:badWidth', 'Target width W must be positive.');
end

ID = log2(D ./ W + 1);
MT = a + b .* ID;
end
