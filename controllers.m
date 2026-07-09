function [tau_exo, tau_human, alpha] = controllers(law, tau_total, f, D, p)
%CONTROLLERS Load-sharing split: three alpha laws + torque split.
%   [tau_exo, tau_human, alpha] = CONTROLLERS(law, tau_total, f, D, p)
%   law: 'fixed' | 'fill' | 'difficulty'
%
%   tau_exo   = alpha * tau_total       (exoskeleton share)
%   tau_human = (1 - alpha) * tau_total (human share)
%
%   All laws are parameterized in p.ctrl (Decision B - nothing hard-coded
%   here). alpha_min > 0 and alpha_max < 1 keep the human engaged and the
%   exo below full takeover; these bounds make the problem non-trivial.

c = p.ctrl;
switch law
    case 'fixed'
        alpha = c.alpha_fixed;
    case 'fill'
        s_fill = f;   % fill level already normalized to [0,1]
        alpha = c.alpha_min + (c.alpha_max - c.alpha_min) * s_fill;
    case 'difficulty'
        s = (D - c.D_lo) / (c.D_hi - c.D_lo);
        alpha = c.alpha_min + (c.alpha_max - c.alpha_min) * s;
    otherwise
        error('controllers: unknown law "%s"', law);
end
alpha = max(c.alpha_min, min(c.alpha_max, alpha));

tau_exo   = alpha * tau_total;
tau_human = (1 - alpha) * tau_total;
end
