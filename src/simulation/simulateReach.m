function res = simulateReach(target, params, controller, opts)
%SIMULATEREACH Simulate one reaching trial toward a target.
%
%   res = simulateReach(target, params, controller, opts) closes the loop
%   between the Lagrangian arm dynamics and a joint-space controller, drives
%   the hand from its home position to a target, and returns the recorded
%   trajectory together with effort and tracking-accuracy metrics.
%
%   Inputs
%     target      struct from targetObjects (needs .pos, .W, .ID)
%     params      arm parameters (armParameters)
%     controller  struct:
%                   .type   'fixed' | 'adaptive'
%                   .gains  gain struct for the chosen controller
%     opts        (optional) struct:
%                   .q0        initial joint angles      (default params.qHome)
%                   .dt        integration step (s)      (default 0.002)
%                   .T         total sim time (s)        (default 2.5)
%                   .moveTime  reach duration (s)        (default 1.2)
%
%   Output res fields
%     .t .q .qdot .tau        time and state / torque histories (N x .)
%     .hand .desHand          actual / desired hand paths       (N x 3)
%     .qref                   joint reference                    (N x 3)
%     .effort                 struct from actuatorEffort
%     .tracking               struct from trackingError
%     .finalError             final hand-to-target distance (m)
%     .hitTarget              logical: landed within W/2 of centre
%     .target .controllerType echoed metadata
%
%   The plant is integrated with fixed-step RK4 and a zero-order hold on the
%   controller torque, which is robust and matches a digital control loop.

if nargin < 4, opts = struct(); end
opts = withDefaults(opts, struct('q0', params.qHome, 'dt', 0.002, ...
                                 'T', 2.5, 'moveTime', 1.2));

dt = opts.dt;
t  = (0:dt:opts.T)';
N  = numel(t);

q0 = opts.q0(:);

% --- Reference: min-jerk in joint space from home to the target's IK ---
qTarget = inverseKinematics(target.pos, params, q0);
[qref, qdref] = minJerkTrajectory(q0, qTarget, t, opts.moveTime);

% --- Desired hand path (FK of the joint reference) for tracking error ---
desHand = zeros(N, 3);
for k = 1:N
    kk = computeKinematics(qref(k, :)', params);
    desHand(k, :) = kk.p_hand';
end

% --- Select controller ---
switch lower(controller.type)
    case 'fixed'
        ctrlFun = @fixedPIDController;
    case 'adaptive'
        ctrlFun = @adaptivePIDController;
    otherwise
        error('simulateReach:badController', ...
              'Unknown controller type "%s".', controller.type);
end
gains = controller.gains;

% --- Controller state ---
st.eint   = zeros(3, 1);
st.dt     = dt;
st.ID     = target.ID;      % used by the adaptive controller
st.kAdapt = 1.0;

% --- Allocate logs ---
Q    = zeros(N, 3);
Qd   = zeros(N, 3);
TAU  = zeros(N, 3);
HAND = zeros(N, 3);

q  = q0;
qd = zeros(3, 1);

for k = 1:N
    qr  = qref(k, :)';
    qdr = qdref(k, :)';

    [tau, st] = ctrlFun(q, qd, qr, qdr, st, params, gains);

    % log current state (before stepping)
    Q(k, :)    = q';
    Qd(k, :)   = qd';
    TAU(k, :)  = tau';
    HAND(k, :) = computeKinematics(q, params).p_hand';

    % RK4 step of the plant with torque held constant (ZOH)
    [q, qd] = rk4Step(q, qd, tau, params, dt);
end

% --- Metrics ---
res.t          = t;
res.q          = Q;
res.qdot       = Qd;
res.tau        = TAU;
res.hand       = HAND;
res.desHand    = desHand;
res.qref       = qref;
res.effort     = actuatorEffort(t, TAU);
res.tracking   = trackingError(t, HAND, desHand, Q - qref);
res.finalError = norm(HAND(end, :)' - target.pos(:));
res.hitTarget  = res.finalError <= target.W / 2;
res.target     = target;
res.controllerType = controller.type;
end


function [qNext, qdNext] = rk4Step(q, qd, tau, params, dt)
%RK4STEP One fixed-step RK4 integration of the arm dynamics (tau held).
    f = @(qq, qv) forwardDynamics(qq, qv, tau, params);

    k1q = qd;                 k1v = f(q, qd);
    k2q = qd + 0.5*dt*k1v;    k2v = f(q + 0.5*dt*k1q, qd + 0.5*dt*k1v);
    k3q = qd + 0.5*dt*k2v;    k3v = f(q + 0.5*dt*k2q, qd + 0.5*dt*k2v);
    k4q = qd + dt*k3v;        k4v = f(q + dt*k3q,      qd + dt*k3v);

    qNext  = q  + (dt/6)*(k1q + 2*k2q + 2*k3q + k4q);
    qdNext = qd + (dt/6)*(k1v + 2*k2v + 2*k3v + k4v);
end


function s = withDefaults(s, def)
%WITHDEFAULTS Fill missing fields of s from def.
    fn = fieldnames(def);
    for i = 1:numel(fn)
        if ~isfield(s, fn{i}) || isempty(s.(fn{i}))
            s.(fn{i}) = def.(fn{i});
        end
    end
end
