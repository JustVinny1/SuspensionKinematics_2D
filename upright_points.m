function [P1, P2, WC, CP] = upright_points(q, susp)
% Compute world-frame positions of upright points for a given kinematic state.
%
% INPUTS
%   q    - [3x1] generalized coordinates [dy; dz; alpha] (mm, mm, rad)
%   susp - struct, suspension geometry with fields:
%            WC   - [2x1] nominal wheel-centre position (mm)
%            r1   - [2x1] body-fixed vector from WC to upper ball joint (mm)
%            r2   - [2x1] body-fixed vector from WC to lower ball joint (mm)
%            r_cp - [2x1] body-fixed vector from WC to contact patch (mm)
%
% OUTPUTS
%   P1 - [2x1] upper ball joint position (mm)
%   P2 - [2x1] lower ball joint position (mm)
%   WC - [2x1] wheel-centre position (mm)
%   CP - [2x1] contact-patch position (mm)

dy = q(1);
dz = q(2);
alpha = q(3);

R = [cos(alpha) -sin(alpha);
     sin(alpha) cos(alpha)];

WC = susp.WC + [dy; dz];
P1 = susp.WC + [dy; dz] + R * susp.r1;
P2 = susp.WC + [dy; dz] + R * susp.r2;
CP = susp.WC + [dy; dz] + R * susp.r_cp;
end