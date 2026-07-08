function [tau, st] = fixedPIDController(q, qdot, qref, qdotref, st, params, gains)
%FIXEDPIDCONTROLLER Baseline joint-space PID with constant gains.
%
%   [tau, st] = fixedPIDController(q, qdot, qref, qdotref, st, params, gains)
%   computes the joint torque for the current step and returns the updated
%   controller state st.
%
%   Control law (per joint):
%       tau = Kp.*e + Ki.*integral(e) + Kd.*edot  [ + gravity compensation ]
%   with  e = qref - q,  edot = qdotref - qdot.
%
%   This is the CONTROL condition in the study: one fixed set of gains used
%   for every task, regardless of how hard the reach is.
%
%   Inputs
%     q, qdot        current joint angles / velocities        (3x1)
%     qref, qdotref  reference angles / velocities            (3x1)
%     st             state struct with fields:
%                      .eint  integral of error (3x1)
%                      .dt    timestep (s)
%     gains          struct with fields:
%                      .Kp .Ki .Kd   gains (scalar or 3x1)
%                      .gravComp     (logical) add gravity feedforward
%                      .tauMax       (3x1, optional) torque saturation

e    = qref(:)  - q(:);
edot = qdotref(:) - qdot(:);

st.eint = st.eint + e * st.dt;

Kp = gains.Kp(:); Ki = gains.Ki(:); Kd = gains.Kd(:);
tau = Kp .* e + Ki .* st.eint + Kd .* edot;

if isfield(gains, 'gravComp') && gains.gravComp
    tau = tau + gravityVector(q, params);
end

tau = saturateTorque(tau, gains, params);
end
