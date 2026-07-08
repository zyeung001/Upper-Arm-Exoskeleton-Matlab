function C = coriolisMatrix(q, qdot, params)
%CORIOLISMATRIX Coriolis / centripetal matrix C(q,qdot) of the arm.
%
%   C = coriolisMatrix(q, qdot, params) returns the 3x3 matrix such that
%   C*qdot captures the Coriolis and centripetal torques in
%
%       M*qddot + C*qdot + G = tau
%
%   It is built from the Christoffel symbols of the first kind:
%
%       C_kj = sum_i c_ijk * qdot_i
%       c_ijk = 1/2 ( dM_kj/dq_i + dM_ki/dq_j - dM_ij/dq_k )
%
%   The partial derivatives of the mass matrix are obtained by finite
%   differences, which keeps this fully general for the 3-DOF model without
%   a hand-derived symbolic expression.

n = numel(q);
h = 1e-6;

M0 = massMatrix(q, params);

% dM(:,:,i) = partial derivative of M with respect to q_i
dM = zeros(n, n, n);
for i = 1:n
    qp = q;
    qp(i) = qp(i) + h;
    dM(:, :, i) = (massMatrix(qp, params) - M0) / h;
end

C = zeros(n, n);
for k = 1:n
    for j = 1:n
        s = 0;
        for i = 1:n
            cijk = 0.5 * (dM(k, j, i) + dM(k, i, j) - dM(i, j, k));
            s = s + cijk * qdot(i);
        end
        C(k, j) = s;
    end
end
end
