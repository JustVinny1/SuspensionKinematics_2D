function [P1, P2, WC] = upright_points(q, susp)
dy = q(1);
dz = q(2);
alpha = q(3);

R = [cos(alpha) -sin(alpha);
     sin(alpha) cos(alpha)];

WC = susp.WC + [dy; dz];
P1 = susp.WC + [dy; dz] + R * susp.r1;
P2 = susp.WC + [dy; dz] + R * susp.r2;
end