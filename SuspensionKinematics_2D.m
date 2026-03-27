%% Hardpoints positions and lengths

susp.Q1 = [400; 420];   % Upper hardpoint
susp.Q2 = [300; 150];   % Lower hardpoint

susp.P1 = [720; 430];   % Upper ball joint
susp.P2 = [750; 120];   % Lower ball joint
susp.WC = [750; 310];   % Wheel centre

susp.L1 = norm(susp.Q1 - susp.P1);   % Upper link length
susp.L2 = norm(susp.Q2 - susp.P2);   % Lower link length

susp.r1 = susp.P1 - susp.WC;    % body-fixed vector WC to UBJ
susp.r2 = susp.P2 - susp.WC;    % body-fixed vector WC to LBJ

%% Sweep parameters
dz_range = linspace(-80, 80, 161);    % 1 mm steps, droop to bump
tol      = 1e-9;
max_iter = 20;
h        = [1e-6; 1e-8];

%% Pre-allocate
n       = length(dz_range);
dy_out  = zeros(1, n);
alpha_out = zeros(1, n);
P1_out  = zeros(2, n);
P2_out  = zeros(2, n);
WC_out  = zeros(2, n);

%% Find design index (dz = 0)
[~, idx_design] = min(abs(dz_range));

%% Sweep bump (design → full bump)
x = [0; 0];
for k = idx_design:n
    [x, converged] = solve_travel_step(x, dz_range(k), susp, tol, max_iter, h);
    if ~converged
        warning('No convergence at dz = %.1f mm', dz_range(k));
    end
    dy_out(k)    = x(1);
    alpha_out(k) = x(2);
    q = [x(1); dz_range(k); x(2)];
    [P1_out(:,k), P2_out(:,k), WC_out(:,k)] = upright_points(q, susp);
end

%% Sweep droop (design → full droop)
x = [0; 0];
for k = idx_design:-1:1
    [x, converged] = solve_travel_step(x, dz_range(k), susp, tol, max_iter, h);
    if ~converged
        warning('No convergence at dz = %.1f mm', dz_range(k));
    end
    dy_out(k)    = x(1);
    alpha_out(k) = x(2);
    q = [x(1); dz_range(k); x(2)];
    [P1_out(:,k), P2_out(:,k), WC_out(:,k)] = upright_points(q, susp);
end

%% Plot camber vs travel
figure;
plot(dz_range, rad2deg(alpha_out), 'LineWidth', 1.5);
xlabel('Vertical travel (mm)');
ylabel('Camber angle (deg)');
title('Camber vs Wheel Travel');
grid on;

%% Plot track width change vs travel
track_change = 2 * (WC_out(1,:) - susp.WC(1));
figure;
plot(dz_range, track_change, 'LineWidth', 1.5);
xlabel('Vertical travel (mm)');
ylabel('Track width change (mm)');
title('Track Width Change vs Wheel Travel');
grid on;

