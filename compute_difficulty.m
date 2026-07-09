function D = compute_difficulty(t, tau_total)
%COMPUTE_DIFFICULTY Scalar task difficulty from the required torque.
%   D = COMPUTE_DIFFICULTY(t, tau_total) = integrated absolute torque over
%   the movement, summed across joints (N m s). Computed once per task from
%   the reference trajectory only (a-priori, controller-independent) -
%   never from post-split results.
D = trapz(t, sum(abs(tau_total), 1));
end
