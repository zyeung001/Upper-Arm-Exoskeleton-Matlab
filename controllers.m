function [tau_exo, tau_human, alpha] = controllers(law, tau_total, f, D, p)
%CONTROLLERS Load-sharing split: three alpha laws + torque split.
%   [tau_exo, tau_human, alpha] = CONTROLLERS(law, tau_total, f, D, p)
%   law: 'fixed' | 'fill' | 'difficulty' | 'difficulty-linear'
%
%   tau_exo   = alpha * tau_total       (exoskeleton share)
%   tau_human = (1 - alpha) * tau_total (human share)
%
%   'difficulty' (the adopted law) is CLOSED-FORM, not a tuned ramp:
%   human effort is E_human = (1 - alpha) * D exactly, so holding it at a
%   target E* gives  alpha = 1 - E*/D.  E* is the midpoint of the target
%   band, which is fixed before any evaluation data is seen (E_high a
%   priori, E_low by rule on calibration data). The only free parameters
%   are the clamps.
%
%   'difficulty-linear' is the REJECTED baseline: a linear ramp from
%   alpha_min at D_lo to alpha_max at D_hi. It matches the closed-form law
%   on the nominal model but collapses when the difficulty range widens
%   (k_m = 0.7 sensitivity case): its mid-range effort overshoots the band
%   ceiling. Kept for the ablation comparison only.
%
%   All laws are parameterized in p.ctrl / p.band (Decision B - nothing
%   hard-coded here). alpha_min > 0 and alpha_max < 1 keep the human
%   engaged and the exo below full takeover; these bounds make the problem
%   non-trivial (and are where the closed-form law can leave the band -
%   run_sweep reports where the clamps bind).

c = p.ctrl;
switch law
    case 'fixed'
        alpha = c.alpha_fixed;
    case 'fill'
        s_fill = f;   % fill level already normalized to [0,1]
        alpha = c.alpha_min + (c.alpha_max - c.alpha_min) * s_fill;
    case 'difficulty'
        E_star = (p.band.E_low + p.band.E_high) / 2;
        alpha = 1 - E_star / D;
    case 'difficulty-linear'
        s = (D - c.D_lo) / (c.D_hi - c.D_lo);
        alpha = c.alpha_min + (c.alpha_max - c.alpha_min) * s;
    otherwise
        error('controllers: unknown law "%s"', law);
end
alpha = max(c.alpha_min, min(c.alpha_max, alpha));

tau_exo   = alpha * tau_total;
tau_human = (1 - alpha) * tau_total;
end
