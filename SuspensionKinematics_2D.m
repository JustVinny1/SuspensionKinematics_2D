%% SuspensionKinematics_2D
% 2D Double-Wishbone Suspension Kinematics Solver
%
% Computes camber angle, track width change, and roll-centre height
% over a range of vertical wheel travel for a planar double-wishbone
% suspension. Includes an animated visualisation of the mechanism.
%
% UNITS: mm and radians throughout (angles displayed in degrees on plots).
%
% HOW TO USE:
%   1. Edit the geometry and sweep parameters in the USER INPUTS section.
%   2. Run the script.
%
% Dependencies: constraints.m, upright_points.m, numerical_jacobian.m,
%               solve_travel_step.m, find_roll_centre.m

%% ======================== USER INPUTS ========================
% Edit this section to change the suspension geometry or sweep range.
% All coordinates are [y; z] in mm, measured from the vehicle centreline.

% --- Chassis hardpoints ---
susp.Q1 = [400; 420];   % Upper wishbone inboard pickup
susp.Q2 = [300; 150];   % Lower wishbone inboard pickup

% --- Upright ball joints (design position) ---
susp.P1 = [720; 430];   % Upper ball joint
susp.P2 = [750; 120];   % Lower ball joint
susp.WC = [750; 310];   % Wheel centre

% --- Contact patch offset from wheel centre (body-fixed) ---
susp.r_cp = [0; -310];

% --- Sweep parameters ---
dz_range = linspace(-80, 80, 161);    % Vertical travel range (mm), 1 mm steps
tol      = 1e-9;                      % Newton-Raphson convergence tolerance
max_iter = 20;                        % Maximum Newton-Raphson iterations
h        = [1e-6; 1e-8];             % Finite-difference steps [dy (mm); alpha (rad)]

% --- Animation ---
anim_pause = 0.02;                    % Pause between frames (s), smaller = faster
%% ==================== END USER INPUTS ========================

%% Derived quantities (do not edit)
susp.L1 = norm(susp.Q1 - susp.P1);   % Upper link length
susp.L2 = norm(susp.Q2 - susp.P2);   % Lower link length

susp.r1 = susp.P1 - susp.WC;    % Body-fixed vector WC to UBJ
susp.r2 = susp.P2 - susp.WC;    % Body-fixed vector WC to LBJ

%% Pre-allocate output arrays
n       = length(dz_range);
dy_out  = zeros(1, n);
alpha_out = zeros(1, n);
P1_out  = zeros(2, n);
P2_out  = zeros(2, n);
WC_out  = zeros(2, n);
CP_out  = zeros(2, n);
RC_height = zeros(1, n);
IC_out = zeros(2, n);           % Instant centre (available in workspace after run)

%% Find design index (dz = 0)
[~, idx_design] = min(abs(dz_range));

%% Solve kinematics over travel range
% Two sweeps from design position: bump (increasing dz), then droop (decreasing dz).
% Each sweep resets the initial guess so the solver always starts near the solution.
sweep_ranges = {idx_design:n, idx_design-1:-1:1};
for s = 1:2
    x = [0; 0];
    for k = sweep_ranges{s}
        [x, converged] = solve_travel_step(x, dz_range(k), susp, tol, max_iter, h);
        if ~converged
            warning('No convergence at dz = %.1f mm', dz_range(k));
        end
        dy_out(k)    = x(1);
        alpha_out(k) = x(2);
        q = [x(1); dz_range(k); x(2)];
        [P1_out(:,k), P2_out(:,k), WC_out(:,k), CP_out(:,k)] = upright_points(q, susp);
        [RC_height(k), IC_out(:,k)] = find_roll_centre(susp.Q1, P1_out(:,k), susp.Q2, P2_out(:,k), CP_out(:,k));
    end
end

%% Derived results
track_change = 2 * (WC_out(1,:) - susp.WC(1));

%% Summary
fprintf('\n--- Suspension Kinematics Summary ---\n');
fprintf('Travel range:         %.0f to %.0f mm\n', dz_range(1), dz_range(end));
fprintf('Camber at design:     %.2f deg\n', rad2deg(alpha_out(idx_design)));
fprintf('Camber range:         %.2f to %.2f deg\n', rad2deg(min(alpha_out)), rad2deg(max(alpha_out)));
fprintf('Track width change:   %.2f to %.2f mm\n', min(track_change), max(track_change));
fprintf('RC height at design:  %.1f mm\n', RC_height(idx_design));
fprintf('RC height range:      %.1f to %.1f mm\n', min(RC_height), max(RC_height));
fprintf('-------------------------------------\n\n');

%% Plot camber vs travel
figure;
plot(dz_range, rad2deg(alpha_out), 'LineWidth', 1.5);
xlabel('Vertical travel (mm)');
ylabel('Camber angle (deg)');
title('Camber vs Wheel Travel');
grid on;

%% Plot track width change vs travel
figure;
plot(dz_range, track_change, 'LineWidth', 1.5);
xlabel('Vertical travel (mm)');
ylabel('Track width change (mm)');
title('Track Width Change vs Wheel Travel');
grid on;

%% Plot roll centre height vs travel
figure;
plot(dz_range, RC_height, 'LineWidth', 1.5);
xlabel('Vertical travel (mm)');
ylabel('Roll centre height (mm)');
title('Roll centre height vs Wheel Travel');
grid on;

%% Animation
figure; axis equal; hold on;

% Compute axis limits from trajectory data with padding
all_y = [0, susp.Q1(1), susp.Q2(1), P1_out(1,:), P2_out(1,:), WC_out(1,:), CP_out(1,:)];
all_z = [0, susp.Q1(2), susp.Q2(2), P1_out(2,:), P2_out(2,:), WC_out(2,:), CP_out(2,:)];
margin = 50;
xlim([min(all_y) - margin, max(all_y) + margin]);
ylim([min(all_z) - margin, max(all_z) + margin]);

% Create objects
h_upper  = plot([0 0], [0 0], 'b-', 'LineWidth', 2);
h_lower  = plot([0 0], [0 0], 'r-', 'LineWidth', 2);
h_upright = plot([0 0 0 0], [0 0 0 0], 'w-', 'LineWidth', 2.5);
h_ground = yline(0, '-', 'Color', [0.5 0.5 0.5]);
h_title  = title('');
h_cp_line = plot([0 0], [0 0], '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
h_cp      = plot(0, 0, 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
h_rc      = plot(0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'y');

% Static labels for chassis pickups
text(susp.Q1(1)+10, susp.Q1(2), 'Q1', 'Color', 'w');
text(susp.Q2(1)+10, susp.Q2(2), 'Q2', 'Color', 'w');

% Dynamic labels
h_label_P1 = text(0, 0, 'P1', 'Color', 'w');
h_label_P2 = text(0, 0, 'P2', 'Color', 'w');
h_label_WC = text(0, 0, 'WC', 'Color', 'w');
h_label_CP = text(0, 0, 'CP', 'Color', 'g');
h_label_RC = text(0, 0, 'RC', 'Color', 'y');

for k = 1:n
    set(h_upper, 'XData', [susp.Q1(1), P1_out(1,k)], ...
                 'YData', [susp.Q1(2), P1_out(2,k)]);
    set(h_lower, 'XData', [susp.Q2(1), P2_out(1,k)], ...
                 'YData', [susp.Q2(2), P2_out(2,k)]);
    set(h_upright, 'XData', [P2_out(1,k), P1_out(1,k), WC_out(1,k), P2_out(1,k)], ...
                   'YData', [P2_out(2,k), P1_out(2,k), WC_out(2,k), P2_out(2,k)]);
    set(h_title, 'String', sprintf('Wheel travel: %.0f mm', dz_range(k)));

    % CP and RC markers
    set(h_cp_line, 'XData', [WC_out(1,k), CP_out(1,k)], ...
                   'YData', [WC_out(2,k), CP_out(2,k)]);
    set(h_cp, 'XData', CP_out(1,k), 'YData', CP_out(2,k));
    set(h_rc, 'XData', 0, 'YData', RC_height(k));

    % Moving labels (offset by 10 mm so they don't sit on top of the marker)
    set(h_label_P1, 'Position', [P1_out(1,k)+10, P1_out(2,k)]);
    set(h_label_P2, 'Position', [P2_out(1,k)+10, P2_out(2,k)]);
    set(h_label_WC, 'Position', [WC_out(1,k)+10, WC_out(2,k)]);
    set(h_label_CP, 'Position', [CP_out(1,k)+10, CP_out(2,k)-20]);
    set(h_label_RC, 'Position', [15, RC_height(k)]);

    drawnow;
    pause(anim_pause);
end