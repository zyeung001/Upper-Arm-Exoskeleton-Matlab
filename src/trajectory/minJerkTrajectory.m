function [q, qd, qdd] = minJerkTrajectory(q0, qf, t, T)
%MINJERKTRAJECTORY Minimum-jerk reference trajectory between two points.
%
%   [q, qd, qdd] = minJerkTrajectory(q0, qf, t, T) generates a smooth
%   minimum-jerk path from q0 to qf over duration T, evaluated at the times
%   in t. It works per coordinate, so q0/qf may be joint vectors (any length
%   n) or Cartesian points -- the reference for the reach.
%
%   Inputs
%     q0, qf   start / end configuration (length-n vectors)
%     t        Nx1 vector of evaluation times (s)
%     T        movement duration (s)
%
%   Outputs (each Nxn)
%     q        position, qd velocity, qdd acceleration
%
%   For t <= 0 the trajectory sits at q0; for t >= T it holds at qf (so the
%   arm dwells on the target after the move completes). The normalized
%   minimum-jerk profile is s(u) = 10u^3 - 15u^4 + 6u^5.

t  = t(:);
q0 = q0(:)';        % 1xn
qf = qf(:)';        % 1xn

u  = min(max(t / T, 0), 1);              % Nx1 normalized time, clamped

s   = 10*u.^3 - 15*u.^4 + 6*u.^5;                 % position shape
sd  = (30*u.^2 - 60*u.^3 + 30*u.^4) / T;          % velocity shape
sdd = (60*u    - 180*u.^2 + 120*u.^3) / T^2;      % acceleration shape

% Zero out derivatives outside the move window (held endpoints).
held = (t <= 0) | (t >= T);
sd(held)  = 0;
sdd(held) = 0;

delta = qf - q0;                          % 1xn
q   = q0 + s   .* delta;                   % Nxn via implicit expansion
qd  =        sd  .* delta;
qdd =        sdd .* delta;
end
