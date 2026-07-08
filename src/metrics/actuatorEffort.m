function E = actuatorEffort(t, tau)
%ACTUATOREFFORT Summary measures of actuator effort over a trial.
%
%   E = actuatorEffort(t, tau) takes a time vector t (Nx1) and the joint
%   torque history tau (Nx3, one row per timestep) and returns a struct of
%   effort metrics used to compare controllers:
%
%     .integralSqTorque  integral over time of sum-of-squared torque
%                        (proportional to energy dissipated in the actuators;
%                         the primary effort metric)
%     .rmsTorque         root-mean-square of the total torque magnitude
%     .peakTorque        peak instantaneous torque magnitude
%     .meanAbsTorque     mean of the summed absolute joint torques
%
%   Lower values mean the controller achieved the reach with less actuator
%   demand -- i.e. less effort asked of the exoskeleton (and the human).

t   = t(:);
sq  = sum(tau.^2, 2);                 % sum of squared torque at each step
mag = sqrt(sum(tau.^2, 2));           % torque vector magnitude at each step

E.integralSqTorque = trapz(t, sq);
E.rmsTorque        = sqrt(mean(sq));
E.peakTorque       = max(mag);
E.meanAbsTorque    = mean(sum(abs(tau), 2));
end
