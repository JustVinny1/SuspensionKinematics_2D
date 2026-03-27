function J = numerical_jacobian(x, dz, h, susp)
% x = [dy; alpha]
% Returns J = 2x2 matrix of partial derivatives of constraints w.r.t. x

dy = x(1);
alpha = x(2);

dy_plus = [dy + h(1); alpha];
dy_minus = [dy - h(1); alpha];

alpha_plus = [dy; alpha + h(2)];
alpha_minus = [dy; alpha - h(2)];

J(:,1) = (constraints(dy_plus, dz, susp) - constraints(dy_minus, dz, susp)) / (2 * h(1));
J(:,2) = (constraints(alpha_plus, dz, susp) - constraints(alpha_minus, dz, susp)) / (2 * h(2));
end