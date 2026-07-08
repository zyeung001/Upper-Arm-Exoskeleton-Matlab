function [tau, st] = adaptivePIDController(q, qdot, qref, qdotref, st, params, gains)
%ADAPTIVEPIDCONTROLLER Difficulty-adaptive joint-space PID.
%
%   [tau, st] = adaptivePIDController(q, qdot, qref, qdotref, st, params, gains)
%   is the EXPERIMENTAL condition: the PID gains are scheduled by the Fitts'
%   index of difficulty (ID) of the current reaching task, so that harder
%   tasks (far and/or narrow targets) are tracked more stiffly and with more
%   damping than easy ones.
%
%   Gain schedule (relative to a reference difficulty ID0):
%       s   = 1 + beta * (ID - ID0)          (clamped at sMin > 0)
%       Kp' = Kp * s
%       Kd' = Kd * sqrt(s)                    (keeps damping ratio ~constant)
%       Ki' = Ki
%
%   An optional online term nudges the proportional gain up while a large
%   tracking error persists, then relaxes it -- a light-weight adaptation on
%   top of the difficulty scheduling.
%
%   st must carry (in addition to .eint and .dt):
%       .ID     Fitts index of difficulty of the current task
%       .kAdapt online adaptive multiplier (initialise to 1)
%
%   gains adds to the fixed-PID fields:
%       .ID0    reference difficulty            (default 3)
%       .beta   difficulty sensitivity          (default 0.35)
%       .sMin   minimum schedule multiplier     (default 0.5)
%       .gamma  online adaptation rate          (default 0, i.e. off)
%       .kAdaptMax  cap on the online multiplier (default 1.5)

e    = qref(:)  - q(:);
edot = qdotref(:) - qdot(:);

st.eint = st.eint + e * st.dt;

% ---- Difficulty-based gain scheduling ----
ID0  = getfielddef(gains, 'ID0',  3.0);
beta = getfielddef(gains, 'beta', 0.35);
sMin = getfielddef(gains, 'sMin', 0.50);
s = max(1 + beta * (st.ID - ID0), sMin);

% ---- Optional online adaptation on Kp ----
gamma     = getfielddef(gains, 'gamma', 0.0);
kAdaptMax = getfielddef(gains, 'kAdaptMax', 1.5);
if ~isfield(st, 'kAdapt') || isempty(st.kAdapt)
    st.kAdapt = 1.0;
end
if gamma > 0
    errMag   = norm(e);
    st.kAdapt = st.kAdapt + gamma * st.dt * (errMag - 0.02);  % 2 cm deadband
    st.kAdapt = min(max(st.kAdapt, 1.0), kAdaptMax);
end

Kp = gains.Kp(:) * s * st.kAdapt;
Kd = gains.Kd(:) * sqrt(s);
Ki = gains.Ki(:);

tau = Kp .* e + Ki .* st.eint + Kd .* edot;

if isfield(gains, 'gravComp') && gains.gravComp
    tau = tau + gravityVector(q, params);
end

tau = saturateTorque(tau, gains, params);
end


function v = getfielddef(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
