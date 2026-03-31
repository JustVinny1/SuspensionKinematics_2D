function f = constraints(x, dz, susp)
% Evaluate link-length constraint residuals for the double-wishbone suspension.
%
% INPUTS
%   x    - [2x1] state vector [dy; alpha] (mm, rad)
%   dz   - scalar, prescribed vertical wheel travel (mm)
%   susp - struct, suspension geometry (see main script for fields)
%
% OUTPUTS
%   f - [2x1] residual vector; each entry is (actual link length^2 - nominal link length^2)

q = [x(1) dz x(2)];

[P1, P2, ~, ~] = upright_points(q, susp);

% Squared-length residuals avoid a sqrt and keep the function smooth
f1 = (P1 - susp.Q1)' * (P1 - susp.Q1) - susp.L1^2;
f2 = (P2 - susp.Q2)' * (P2 - susp.Q2) - susp.L2^2;

f = [f1; f2];
end