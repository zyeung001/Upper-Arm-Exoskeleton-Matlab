function gains = controllerGains(type)
%CONTROLLERGAINS Default gain sets for the two controllers.
%
%   gains = controllerGains('fixed')     returns the baseline PID gains.
%   gains = controllerGains('adaptive')  returns the difficulty-scheduled PID
%                                        gains (base gains + schedule params).
%
%   Both include gravity compensation and use the actuator torque limits from
%   armParameters (via saturateTorque). The two controllers share the SAME
%   base Kp/Ki/Kd so the comparison isolates the effect of the difficulty
%   adaptation, not a lucky retune. Adjust these once and both experiments and
%   the demo pick them up.

base.Kp = [120; 120; 80];
base.Ki = [ 15;  15; 10];
base.Kd = [ 18;  18; 12];
base.gravComp = true;

switch lower(type)
    case 'fixed'
        gains = base;

    case 'adaptive'
        gains = base;
        gains.ID0       = 3.0;    % reference difficulty (bits)
        gains.beta      = 0.35;   % stiffness sensitivity to ID
        gains.sMin      = 0.50;   % floor on the schedule multiplier
        gains.gamma     = 0.0;    % online adaptation rate (0 = schedule only)
        gains.kAdaptMax = 1.5;    % cap on the online multiplier

    otherwise
        error('controllerGains:badType', 'Unknown controller type "%s".', type);
end
end
