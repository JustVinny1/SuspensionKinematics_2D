function [x, converged] = solve_travel_step(x_guess, dz, susp, tol, max_iter, h)
% x_guess = [dy; alpha] initial guess
% dz = prescribed vertical travel
% Returns converged x = [dy; alpha] and a boolean flag

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

f = constraints(x, dz, susp);
converged = norm(f) < tol;
end