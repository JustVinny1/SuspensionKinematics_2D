function J = numerical_jacobian(x, dz, h, susp)
% Compute the 2x2 Jacobian of the constraint residuals using central finite differences.
%
% INPUTS
%   x    - [2x1] state vector [dy; alpha] (mm, rad)
%   dz   - scalar, prescribed vertical wheel travel (mm)
%   h    - [2x1] finite-difference step sizes [h_dy; h_alpha] (mm, rad)
%   susp - struct, suspension geometry (see main script for fields)
%
% OUTPUTS
%   J - [2x2] Jacobian matrix, J(i,j) = df_i / dx_j

dy = x(1);
alpha = x(2);

dy_plus = [dy + h(1); alpha];
dy_minus = [dy - h(1); alpha];

alpha_plus = [dy; alpha + h(2)];
alpha_minus = [dy; alpha - h(2)];

J(:,1) = (constraints(dy_plus, dz, susp) - constraints(dy_minus, dz, susp)) / (2 * h(1));
J(:,2) = (constraints(alpha_plus, dz, susp) - constraints(alpha_minus, dz, susp)) / (2 * h(2));
end