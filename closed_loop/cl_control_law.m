function [u, u_exo, u_hum] = cl_control_law(e, edot, x, Kp, Kd, alpha, pvec)
%CL_CONTROL_LAW PD + gravity-compensation tracking torque and alpha split.
%   [u, u_exo, u_hum] = CL_CONTROL_LAW(e, edot, x, Kp, Kd, alpha, pvec)
%
%   e, edot = joint position / velocity tracking error (3x1), e = qref - q
%   x       = plant state [q; qd] (8x1)
%   alpha   = assistance share, CONSTANT per task, computed a priori by
%             controllers.m from the reference trajectory (the alpha laws
%             depend only on fill f and difficulty D, never on closed-loop
%             results - same a-priori discipline as the sweep).
%
%   u       = total tracking torque (whiteboard controller block)
%   u_exo   = alpha * u        (exoskeleton share)
%   u_hum   = (1 - alpha) * u  (human share)
%
%   Gravity feedforward uses G at the MEASURED arm angles with phi = 0
%   (liquid assumed at rest): the exo/human pair compensates the static
%   arm + payload weight it can know about, but NOT the instantaneous
%   slosh state - the slosh remains a genuine disturbance.
%#codegen
e = e(:);
edot = edot(:);
x = x(:);
pvec = reshape(pvec, 1, []);   % generated funcs index pvec as a row
q0 = [x(1:3); 0];
G = G_fun(q0, pvec);
u = Kp*e + Kd*edot + G(1:3);
u_exo = alpha * u;
u_hum = (1 - alpha) * u;
end
