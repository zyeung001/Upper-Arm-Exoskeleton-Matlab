function [Xref, VelRef, traj, tt, qref, qdref] = cl_reference(d, Tend, p)
%CL_REFERENCE Reference timeseries for the closed-loop model.
%   [Xref, VelRef, traj, tt, qref, qdref] = CL_REFERENCE(d, Tend, p)
%
%   Reuses make_trajectory (same quintic joint-space reference, elbow-down
%   IK, fixed T_move as the sweep - Decision A untouched), then HOLDS the
%   final pose from T_move to Tend so every run contains a settling phase.
%   Returns From Workspace-ready timeseries (whiteboard: Xref = timeseries
%   [time x xdot] from 0 to Tend).

traj = make_trajectory(d, p.sim.T_move, p.sim.dt, p);
t_hold = (traj.t(end) + p.sim.dt):p.sim.dt:Tend;
tt    = [traj.t, t_hold];
qref  = [traj.q,  repmat(traj.q(:, end), 1, numel(t_hold))];
qdref = [traj.qd, zeros(3, numel(t_hold))];

Xref   = timeseries(qref.',  tt.', 'Name', 'qref');
VelRef = timeseries(qdref.', tt.', 'Name', 'qdref');
end
