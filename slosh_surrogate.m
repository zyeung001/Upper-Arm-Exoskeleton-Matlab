function s = slosh_surrogate(f, p)
%SLOSH_SURROGATE Fill-dependent pendulum-equivalent slosh parameters.
%   s = SLOSH_SURROGATE(f, p) evaluates the surrogate rules (Bai et al.
%   2025 form, constants in params.m) at fill level f in [0,1]:
%     s.m_liq  liquid mass rho*pi*Rc^2*(f*Hc)            (kg)
%     s.m_s    participating (sloshing) mass k_m*m_liq   (kg)
%     s.L_s    pendulum length L_s_factor*Rc             (m)
%     s.w_s    natural frequency sqrt(g/L_s)             (rad/s)
%     s.b_s    damping 2*zeta_s*w_s*m_s*L_s^2 (N m s) - the coefficient
%              that makes the nonlinear plant's phi row (cl_plant_deriv)
%              linearize to the same damped surrogate integrated by
%              compute_required_torque
%     s.pvec   15-element parameter vector at this fill (pack_pvec)
%   One definition shared by the sweep pipeline and closed_loop/ so the
%   two studies cannot drift apart on the physics.
s.m_liq = p.cont.rho * pi * p.cont.Rc^2 * (f * p.cont.Hc);
s.m_s   = p.slosh.k_m * s.m_liq;
s.L_s   = p.slosh.L_s_factor * p.cont.Rc;
s.w_s   = sqrt(p.g / s.L_s);
s.b_s   = 2 * p.slosh.zeta_s * s.w_s * s.m_s * s.L_s^2;
s.pvec  = pack_pvec(p, s.m_s, s.L_s);
end
