function [x, converged] = solve_travel_step(x_guess, dz, susp, tol, max_iter, h)
% Solve for upright state [dy; alpha] at a prescribed vertical travel using Newton-Raphson.
%
% INPUTS
%   x_guess  - [2x1] initial guess [dy; alpha] (mm, rad)
%   dz       - scalar, prescribed vertical wheel travel (mm)
%   susp     - struct, suspension geometry (see main script for fields)
%   tol      - scalar, convergence tolerance on norm(f)
%   max_iter - scalar, maximum Newton-Raphson iterations
%   h        - [2x1] finite-difference step sizes [h_dy; h_alpha] (mm, rad)
%
% OUTPUTS
%   x         - [2x1] converged state [dy; alpha] (mm, rad)
%   converged - logical, true if norm(f) < tol was achieved

x = x_guess;
converged = false;

for i = 1:max_iter
    f = constraints(x, dz, susp);
    if norm(f) < tol
        converged = true;
        break
    end
    J = numerical_jacobian(x, dz, h, susp);
    dx = J \ (-f);
    x = x + dx;
end

% Re-check after loop in case max_iter was reached without breaking
f = constraints(x, dz, susp);
converged = norm(f) < tol;
end