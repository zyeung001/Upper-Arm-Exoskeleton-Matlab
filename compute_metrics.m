function m = compute_metrics(t, tau_human, tau_exo, p)
%COMPUTE_METRICS Per-task, per-controller effort metrics.
%   m = COMPUTE_METRICS(t, tau_human, tau_exo, p) returns:
%     m.E_human  integrated |human torque| summed over joints (primary)
%     m.E_exo    integrated |exo torque| summed over joints (secondary)
%     m.status   -1 below band (over-assisted / wasteful)
%                 0 in band
%                +1 above band (under-supported)
m.E_human = trapz(t, sum(abs(tau_human), 1));
m.E_exo   = trapz(t, sum(abs(tau_exo), 1));
if m.E_human < p.band.E_low
    m.status = -1;
elseif m.E_human > p.band.E_high
    m.status = 1;
else
    m.status = 0;
end
end
