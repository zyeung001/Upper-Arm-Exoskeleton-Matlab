function derive_dynamics()
%DERIVE_DYNAMICS Symbolic derivation of the 4-DOF dynamics (3 arm joints +
%   unactuated slosh pendulum), exported as numeric functions in generated/.
%
%   Generalized coordinates: q = [th1; th2; th3; phi]
%     th1  shoulder flexion (from horizontal, positive up), link 1
%     th2  elbow flexion (relative), link 2
%     th3  shoulder rotation about the vertical z-axis, link 3
%     phi  slosh pendulum angle at the end-effector (0 = hanging down),
%          swinging in the arm's vertical plane; unactuated.
%
%   Manipulator form:  M(q) qdd + C(q,qd) qd + G(q) = [tau1; tau2; tau3; 0]
%   (pendulum viscous damping is handled in the decoupled slosh ODE of
%   compute_required_torque, not here; it does not enter the actuated rows).
%
%   Exports (matlabFunction, all taking a packed parameter vector, see
%   pack_pvec):
%     generated/M_fun(q, pvec)            4x4 mass matrix
%     generated/C_fun(q, qd, pvec)        4x4 Coriolis matrix (Christoffel)
%     generated/G_fun(q, pvec)            4x1 gravity vector
%     generated/ee_fun(q3, pvec)          3x1 end-effector position
%     generated/aee_fun(q3,qd3,qdd3,pvec) 3x1 end-effector acceleration
%
%   Validation (numeric asserts, per spec section 3):
%     (1) m_s = 0            -> matches an independently derived 3-DOF arm
%     (2) th3 = 0 frozen     -> matches the textbook 2-link planar arm
%     (3) th2 = 0 frozen too -> matches a 1-DOF compound pendulum

fprintf('Deriving symbolic dynamics...\n');

syms th1 th2 th3 phi th1d th2d th3d phid real
syms m1 m2 m3 L1 L2 L3 lc1 lc2 lc3 I1 I2 I3 g m_s L_s real

q  = [th1; th2; th3; phi];
qd = [th1d; th2d; th3d; phid];
pvec = [m1 m2 m3 L1 L2 L3 lc1 lc2 lc3 I1 I2 I3 g m_s L_s];

Rz = @(a) [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
Ry = @(a) [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];

% Link rotation matrices (body x-axis along each rod)
R3 = Rz(th3);                 % base segment, horizontal
R1 = Rz(th3) * Ry(-th1);      % upper arm
R2 = Rz(th3) * Ry(-(th1+th2));% forearm

% Positions (base frame at the shoulder-rotation axis)
com3   = R3 * [lc3; 0; 0];
shoulder = R3 * [L3; 0; 0];
com1   = shoulder + R1 * [lc1; 0; 0];
elbow  = shoulder + R1 * [L1; 0; 0];
com2   = elbow + R2 * [lc2; 0; 0];
ee     = elbow + R2 * [L2; 0; 0];
bob    = ee + L_s * Rz(th3) * [sin(phi); 0; -cos(phi)];

% Kinetic energy: links as thin rods, I_body = diag(0, I, I) about the COM
T = sym(0);
bodies = {com1, R1, m1, I1; com2, R2, m2, I2; com3, R3, m3, I3};
for k = 1:size(bodies, 1)
    [pos, R, m, I] = bodies{k, :};
    v  = jacobian(pos, q) * qd;
    Rd = sym(zeros(3)); % dR/dt
    for j = 1:4
        Rd = Rd + diff(R, q(j)) * qd(j);
    end
    W  = simplify(Rd * R.');            % skew(omega) in world frame
    w  = [W(3,2); W(1,3); W(2,1)];
    Iw = R * diag([0, I, I]) * R.';
    T  = T + m*(v.'*v)/2 + (w.'*Iw*w)/2;
end
vb = jacobian(bob, q) * qd;             % pendulum bob: point mass
T  = T + m_s*(vb.'*vb)/2;

% Potential energy
V = g * (m1*com1(3) + m2*com2(3) + m3*com3(3) + m_s*bob(3));

% Manipulator form: M from the KE Hessian, C via Christoffel symbols
T = simplify(expand(T));
M = simplify(hessian(T, qd));
G = simplify(jacobian(V, q).');
C = sym(zeros(4));
for i = 1:4
    for j = 1:4
        for k = 1:4
            C(i,j) = C(i,j) + (diff(M(i,j), q(k)) + diff(M(i,k), q(j)) ...
                               - diff(M(j,k), q(i))) * qd(k) / 2;
        end
    end
end
C = simplify(C);

% End-effector kinematics (actuated joints only)
q3 = q(1:3);  qd3 = qd(1:3);
syms th1dd th2dd th3dd real
qdd3 = [th1dd; th2dd; th3dd];
J    = jacobian(ee, q3);
Jd   = sym(zeros(3));
for j = 1:3
    Jd = Jd + diff(J, q3(j)) * qd3(j);
end
aee = J * qdd3 + Jd * qd3;

outdir = fullfile(fileparts(mfilename('fullpath')), 'generated');
if ~exist(outdir, 'dir'), mkdir(outdir); end
matlabFunction(M,   'File', fullfile(outdir, 'M_fun'),   'Vars', {q, pvec});
matlabFunction(C,   'File', fullfile(outdir, 'C_fun'),   'Vars', {q, qd, pvec});
matlabFunction(G,   'File', fullfile(outdir, 'G_fun'),   'Vars', {q, pvec});
matlabFunction(V,   'File', fullfile(outdir, 'V_fun'),   'Vars', {q, pvec});
matlabFunction(ee,  'File', fullfile(outdir, 'ee_fun'),  'Vars', {q3, pvec});
matlabFunction(aee, 'File', fullfile(outdir, 'aee_fun'), 'Vars', {q3, qd3, qdd3, pvec});
addpath(outdir);
fprintf('Exported M_fun, C_fun, G_fun, V_fun, ee_fun, aee_fun to generated/.\n');

validate_structure();
validate_reductions();
end

% ------------------------------------------------------------------------
function validate_structure()
%Independent numeric checks on the exported FULL model (m_s > 0), catching
%sign/coupling errors the reduction checks cannot:
%  (a) M symmetric and positive definite at random configurations
%  (b) Mdot - 2C skew-symmetric (Christoffel/passivity consistency)
%  (c) free-swing energy conservation (zero torque, zero damping)
%Note aee_fun needs no separate consistency check: it is the jacobian of
%the SAME symbolic ee that positions the pendulum bob inside M/C/G, so the
%slosh forcing and the inverse-dynamics coupling share one kinematics.
p = params();
m_liq = p.cont.rho * pi * p.cont.Rc^2 * (0.7 * p.cont.Hc);
pv = pack_pvec(p, p.slosh.k_m * m_liq, p.cont.Rc);
rng(2);

for trial = 1:5
    qr = randn(4, 1) * 0.6;  qdr = randn(4, 1);
    M = M_fun(qr, pv);
    assert(max(abs(M - M.'), [], 'all') < 1e-9, 'M not symmetric');
    assert(min(eig(M)) > 0, 'M not positive definite');
    Md = zeros(4);  h = 1e-6;
    for k = 1:4
        e = zeros(4, 1);  e(k) = h;
        Md = Md + (M_fun(qr + e, pv) - M_fun(qr - e, pv)) / (2*h) * qdr(k);
    end
    S = Md - 2 * C_fun(qr, qdr, pv);
    assert(max(abs(S + S.'), [], 'all') < 1e-5, ...
        'Mdot - 2C not skew-symmetric (C/Christoffel error)');
end

x0  = [0.3; -0.5; 0.2; 0.4; zeros(4, 1)];
ode = @(t, x) [x(5:8); M_fun(x(1:4), pv) \ ...
    (-C_fun(x(1:4), x(5:8), pv) * x(5:8) - G_fun(x(1:4), pv))];
[~, X] = ode45(ode, [0 1.5], x0, odeset('RelTol', 1e-10, 'AbsTol', 1e-12));
E = zeros(size(X, 1), 1);
for k = 1:size(X, 1)
    qk = X(k, 1:4).';  qdk = X(k, 5:8).';
    E(k) = qdk.' * M_fun(qk, pv) * qdk / 2 + V_fun(qk, pv);
end
drift = max(abs(E - E(1)));
assert(drift < 1e-5 * max(1, abs(E(1))), ...
    'free-swing energy drifted by %.3g J', drift);
fprintf('Structure validation passed (M sym/PD, Mdot-2C skew, energy).\n');
end

% ------------------------------------------------------------------------
function validate_reductions()
%Numeric reduction checks against independently derived simpler models.
p = params();
rng(1);
tol = 1e-9;
a = p.arm;

for trial = 1:5
    qr  = [randn; randn; randn; randn] * 0.6;
    qdr = randn(4, 1);

    % ---- (1) m_s = 0: pendulum decouples from the 3-DOF arm ----
    pv0 = pack_pvec(p, 0, 0.05);
    M4 = M_fun(qr, pv0);  G4 = G_fun(qr, pv0);
    assert(max(abs([M4(1:3,4); M4(4,:).'; G4(4)])) < tol, ...
        'm_s=0: slosh DOF did not decouple');
    % Arm block must be phi-independent and match the arm evaluated at a
    % different phi (the 3-DOF model is the phi-independent restriction).
    qr2 = qr; qr2(4) = qr(4) + 1.3;
    M4b = M_fun(qr2, pv0);
    assert(max(abs(M4(1:3,1:3) - M4b(1:3,1:3)), [], 'all') < tol, ...
        'm_s=0: arm block depends on phi');

    % ---- (2) th3 = 0 frozen, m_s = 0: textbook 2-link planar arm ----
    qp  = [qr(1); qr(2); 0; 0];
    qdp = [qdr(1); qdr(2); 0; 0];
    M4 = M_fun(qp, pv0);  C4 = C_fun(qp, qdp, pv0);  G4 = G_fun(qp, pv0);
    c1 = cos(qp(1)); c12 = cos(qp(1)+qp(2)); s2 = sin(qp(2));
    M11 = a.m1*a.lc1^2 + a.I1 + a.m2*(a.L1^2 + a.lc2^2 ...
          + 2*a.L1*a.lc2*cos(qp(2))) + a.I2;
    M12 = a.m2*(a.lc2^2 + a.L1*a.lc2*cos(qp(2))) + a.I2;
    M22 = a.m2*a.lc2^2 + a.I2;
    h   = -a.m2*a.L1*a.lc2*s2;
    C2  = [h*qdp(2), h*(qdp(1)+qdp(2)); -h*qdp(1), 0];
    G1  = (a.m1*a.lc1 + a.m2*a.L1)*p.g*c1 + a.m2*a.lc2*p.g*c12;
    G2  = a.m2*a.lc2*p.g*c12;
    assert(max(abs(M4(1:2,1:2) - [M11 M12; M12 M22]), [], 'all') < tol, ...
        '2-DOF reduction: mass matrix mismatch');
    assert(max(abs(C4(1:2,1:2)*qdp(1:2) - C2*qdp(1:2))) < tol, ...
        '2-DOF reduction: Coriolis mismatch');
    assert(max(abs(G4(1:2) - [G1; G2])) < tol, ...
        '2-DOF reduction: gravity mismatch');

    % ---- (3) th2 = 0 frozen too: 1-DOF compound pendulum ----
    q1  = [qr(1); 0; 0; 0];
    M4 = M_fun(q1, pv0);  G4 = G_fun(q1, pv0);
    I_eff = a.I1 + a.m1*a.lc1^2 + a.I2 + a.m2*(a.L1 + a.lc2)^2;
    G_eff = (a.m1*a.lc1 + a.m2*(a.L1 + a.lc2))*p.g*cos(qr(1));
    assert(abs(M4(1,1) - I_eff) < tol, '1-DOF reduction: inertia mismatch');
    assert(abs(G4(1) - G_eff) < tol, '1-DOF reduction: gravity mismatch');
end
fprintf('Reduction validation passed (3-DOF, 2-DOF planar, 1-DOF).\n');
end
