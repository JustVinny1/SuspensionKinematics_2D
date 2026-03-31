function [RC_height, IC] = find_roll_centre(Q1, P1, Q2, P2, CP)
% Find the roll-centre height by intersecting the wishbone lines at the instant centre.
%
% INPUTS
%   Q1 - [2x1] upper wishbone inboard (chassis) point (mm)
%   P1 - [2x1] upper wishbone outboard (ball joint) point (mm)
%   Q2 - [2x1] lower wishbone inboard (chassis) point (mm)
%   P2 - [2x1] lower wishbone outboard (ball joint) point (mm)
%   CP - [2x1] tyre contact-patch position (mm)
%
% OUTPUTS
%   RC_height - scalar, roll-centre height above ground (mm)
%   IC        - [2x1] instant-centre position [y; z] (mm)

d1 = P1 - Q1;
d2 = P2 - Q2;

% Solve for the intersection parameter of the two wishbone lines
t = [d1, -d2] \ (Q2 - Q1);

IC = Q1 + t(1)*d1;

% Roll centre lies where the line from CP through IC crosses the vehicle centreline (y=0)
RC_height = IC(2) * (-CP(1) / (IC(1) - CP(1)));
end