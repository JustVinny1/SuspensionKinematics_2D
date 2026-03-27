function f = constraints(x, dz, susp)
% x = [dy; alpha]  (the two unknowns)
% dz = prescribed vertical travel
% susp = geometry struct
% Returns f = [f1; f2] residual vector

q = [x(1) dz x(2)];

[P1, P2, ~] = upright_points(q, susp);

f1 = (P1 - susp.Q1)' * (P1 - susp.Q1) - susp.L1^2;
f2 = (P2 - susp.Q2)' * (P2 - susp.Q2) - susp.L2^2;

f = [f1; f2];
end