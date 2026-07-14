function o = cl_overload(t, uhum, uhumc, win, cl)
%CL_OVERLOAD Did the human hit their strength cap, and what did it cost?
%   o = CL_OVERLOAD(t, uhum, uhumc, win, cl) with t (1xN), uhum (3xN, the
%   torque the human actually DELIVERED, after lag + cap), uhumc (3xN, the
%   torque the law DEMANDED of them), win a logical mask for the carry
%   window, cl = cl_params().
%
%   Overload is assessed over the WHOLE run, not just the carry: the human
%   still has to HOLD the full container at the target pose afterwards, and
%   that static gravity share - not the dynamic peak during the carry - is
%   usually the binding constraint. A human pinned at their cap during the
%   hold can never null the error, so the arm droops for good (t_settle =
%   NaN). Effort integrals stay on the carry window, where they are
%   comparable to the sweep's band.
%
%   o.overload    true if any joint sat at its cap at any point
%   o.duty        fraction of the RUN with at least one joint capped
%   o.duty_carry  the same over the carry window only
%   o.hold_capped true if still capped at the final instant (permanent droop)
%   o.deficit     peak undelivered torque, max_t sum_j|uhumc - uhum| (N m):
%                 torque the task needed and nobody supplied
%   o.E_demanded  integral of sum|uhumc| over the carry (N m s) - the effort
%                 the law ASKED for. Once the cap binds this exceeds the
%                 measured E_human, and that gap IS the failure: a capped
%                 human's deflated effort must never be read as "in band".
%   o.joints      (3x1 logical) which joints saturated

cap  = cl.u_h_max(:);
sat  = abs(uhum) >= (1 - cl.sat_tol) * cap;   % 3xN
hit  = any(sat, 1);                           % 1xN

o.joints      = any(sat, 2);
o.overload    = any(hit);
o.hold_capped = hit(end);
o.duty        = duty_frac(t, hit);
o.duty_carry  = duty_frac(t(win), hit(win));
o.deficit     = max(sum(abs(uhumc - uhum), 1));

tw = t(win);
o.E_demanded = trapz(tw, sum(abs(uhumc(:, win)), 1));
end

% ------------------------------------------------------------------------
function f = duty_frac(t, hit)
% Time-weighted, not sample-counted: the variable-step solver clusters
% output points around the limit's zero crossings, so a sample fraction
% would overstate the time actually spent at the cap.
if numel(t) < 2 || t(end) <= t(1)
    f = double(any(hit));
    return
end
f = trapz(t, double(hit)) / (t(end) - t(1));
end
