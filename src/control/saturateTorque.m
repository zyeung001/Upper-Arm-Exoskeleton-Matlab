function tau = saturateTorque(tau, gains, params)
%SATURATETORQUE Clamp joint torques to the actuator limits.
%
%   tau = saturateTorque(tau, gains, params) limits each element of tau to
%   +/- the per-joint torque limit. The limit is taken from gains.tauMax if
%   present, otherwise from params.tauMax; if neither exists, tau is returned
%   unchanged. Modelling saturation matters because a controller that only
%   "wins" by commanding unrealistic torque is not a fair comparison.

if isfield(gains, 'tauMax') && ~isempty(gains.tauMax)
    lim = gains.tauMax(:);
elseif isfield(params, 'tauMax') && ~isempty(params.tauMax)
    lim = params.tauMax(:);
else
    return;
end

tau = max(min(tau, lim), -lim);
end
