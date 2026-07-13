function m = cl_metrics(t, q, err, u, phi, tt, qref, pvec, cl)
%CL_METRICS Closed-loop tracking metrics (whiteboard: Accuracy / Time / Energy).
%   m = CL_METRICS(t, q, err, u, phi, tt, qref, pvec, cl)
%
%   t (1xN, possibly non-uniform solver steps), q/err/u (3xN), phi (1xN),
%   tt/qref = reference time base and joint reference (for the end-effector
%   error), pvec for ee_fun, cl = cl_params().
%
%   m.rmse_q     time-integral RMS of the joint error norm (rad)
%   m.rmse_ee    time-integral RMS of the end-effector position error (m)
%   m.max_ee     peak end-effector error (m)
%   m.t_settle   first time after which ||ee - ee_ref|| stays < cl.tol_ee
%                (s); NaN if it never settles
%   m.energy_u   control energy integral(||u||^2) dt  (N^2 m^2 s)
%   m.effort_u   integral(sum|u|) dt (N m s) - comparable to the sweep's
%                difficulty/effort units
%   m.peak_slosh max |phi| (rad)
%   m.ee_err     (1xN) end-effector error norm time series (for figures)

N = numel(t);
T = t(end) - t(1);

% End-effector error vs the (interpolated) reference
qr = interp1(tt.', qref.', t(:), 'linear', 'extrap').';   % 3xN
ee_err = zeros(1, N);
for k = 1:N
    ee_sim = ee_fun([q(:, k);      0], pvec);
    ee_ref = ee_fun([qr(:, k);     0], pvec);
    ee_err(k) = norm(ee_sim - ee_ref);
end

m.rmse_q  = sqrt(trapz(t, sum(err.^2, 1)) / T);
m.rmse_ee = sqrt(trapz(t, ee_err.^2) / T);
m.max_ee  = max(ee_err);

% Time to settle: last excursion above tolerance
idx = find(ee_err > cl.tol_ee, 1, 'last');
if isempty(idx)
    m.t_settle = t(1);
elseif idx == N
    m.t_settle = NaN;                 % never settles inside the window
else
    m.t_settle = t(idx + 1);
end

m.energy_u   = trapz(t, sum(u.^2, 1));
m.effort_u   = trapz(t, sum(abs(u), 1));
m.peak_slosh = max(abs(phi));
m.ee_err     = ee_err;
end
