function pvec = pack_pvec(p, m_s, L_s)
%PACK_PVEC Pack parameters into the vector expected by the generated
%   dynamics functions (M_fun, C_fun, G_fun, ee_fun, aee_fun).
%   m_s, L_s are the fill-dependent slosh pendulum mass and length.
a = p.arm;
pvec = [a.m1, a.m2, a.m3, a.L1, a.L2, a.L3, ...
        a.lc1, a.lc2, a.lc3, a.I1, a.I2, a.I3, p.g, m_s, L_s];
end
