function cl_validate_surface()
% closed loop with a controller and plant without lag + cap is validated against
% the open loop by comparing the percent difference of the human effort of
% both

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(root, fullfile(root, 'generated'), here);

p = calibrate_controller(params());
cl = cl_params();

fill_arr = [p.sweep.f_min, p.sweep.f_max];
distance_arr = [p.sweep.d_min, p.sweep.d_max];
task_arr = [fill_arr(1) distance_arr(1);       % small fill, close reach
            fill_arr(1) distance_arr(2);       % small fill, far reach
            fill_arr(2) distance_arr(1);       % full fill, close reach
            fill_arr(2) distance_arr(2);       % full fill, far reach
            mean(fill_arr) mean(distance_arr); % center
            cl.f_default cl.d_default];        % default fill = 80% reach = 0.4m

win_fun = @(t) t <= p.sim.T_move;
laws = {'fixed', 'fill', 'difficulty'};
nl = numel(laws);

mdl = 'arm_closed_loop';
mdlfile = fullfile(here, [mdl '.slx']);
if cl_needs_rebuild(mdlfile)
    build_closed_loop_model();
end

fprintf('\nIdeal-human cross-validation across the task space\n');
fprintf('(closed-loop E_human vs open-loop (1-alpha)*D, carry window)\n\n');
fprintf('  %-5s %-5s %-12s %8s %8s %7s\n', 'f', 'd', 'law', 'CL', 'OL', 'err%');

worst = 0;  worst_str = '';
for k = 1:size(task_arr, 1)
    f = task_arr(k, 1);
    d = task_arr(k, 2);

    % open-loop side: reference, required torque, difficulty, alphas
    ss   = slosh_surrogate(f, p);
    b_s  = ss.b_s;
    pvec = ss.pvec;

    [Xref, VelRef, traj, tt, qref] = cl_reference(d, cl.Tend, p);
    [tau_total, ~, ~] = compute_required_torque(traj, f, p);
    D = compute_difficulty(traj.t, tau_total);

    alpha = zeros(1, nl);
    for L = 1:nl
        [~, ~, alpha(L)] = controllers(laws{L}, tau_total, f, D, p);
    end

    % closed-loop side: ideal human (no lag or cap)
    q0 = traj.q(:, 1);
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('cl_Xref',   Xref);
    in = in.setVariable('cl_VelRef', VelRef);
    in = in.setVariable('cl_x0',    [q0; 0; zeros(4, 1)]);
    in = in.setVariable('cl_Kp',    cl.Kp);
    in = in.setVariable('cl_Kd',    cl.Kd);
    in = in.setVariable('cl_pvec',  pvec);
    in = in.setVariable('cl_bs',    b_s);
    in = in.setVariable('cl_rigid', 0);
    in = in.setVariable('cl_tauh',  cl.tau_h);
    in = in.setVariable('cl_uhmax', cl.u_h_max(:));
    in = in.setVariable('cl_Tend',  cl.Tend);
    in = in.setVariable('cl_alpha', alpha(3));
    in = in.setVariable('cl_ideal', 1);
    in = in.setVariable('cl_uh0',   zeros(3, 1));

    out = sim(in);
    t = out.y.Time.';
    u = out.u.Data.';
    win = win_fun(t);

    % compare open loop to closed loop
    for L = 1:nl
        a  = alpha(L);
        bm = compute_metrics(t(win), (1 - a) * u(:, win), a * u(:, win), p);
        CL = bm.E_human;
        OL = (1 - a) * D;
        err = 100 * abs(CL - OL) / max(1e-9, OL);
        fprintf('  %-5.2f %-5.2f %-12s %8.3f %8.3f %6.2f\n', f, d, laws{L}, CL, OL, err);
        if err > worst
            worst = err;
            worst_str = sprintf('%s at (f=%.2f, d=%.2f): CL %.3f vs OL %.3f', laws{L}, f, d, CL, OL);
        end
    end
    fprintf('\n');
end

fprintf('WORST-CASE discrepancy: %.2f%%  [%s]\n\n', worst, worst_str);
end
