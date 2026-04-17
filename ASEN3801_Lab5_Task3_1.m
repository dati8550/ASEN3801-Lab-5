%% Lab 5 Task 3.1
clear; clc; close all;

%% Load aircraft parameters
ttwistor;   % creates aircraft_parameters in workspace

%% Setup
wind_inertial = [0; 0; 0];
options = odeset('RelTol', 1e-3, 'AbsTol', 1e-6, 'MaxStep', 0.1);

%% Trim initial conditions (from Problem 2.2)
x0 = [0;        % x_E (m)
      0;        % y_E (m)
      -1800;    % z_E (m)
      0;        % phi (rad)
      0.02780;  % theta (rad)
      0;        % psi (rad)
      20.99;    % u (m/s)
      0;        % v (m/s)
      0.5837;   % w (m/s)
      0;        % p (rad/s)
      0;        % q (rad/s)
      0];       % r (rad/s)

%% Trim control inputs
u0 = [0.1079;   % delta_e (rad)
      0;        % delta_a (rad)
      0;        % delta_r (rad)
      0.3182];  % delta_t (fraction)

%% Doublet parameters
doublet_size = deg2rad(15);  % 15 degrees in radians
doublet_time = 0.25;         % seconds

%% Run simulation 3 seconds
tspan = [0 3]; % Hardcode this for different duration

[t, X] = ode45(@(t,x) AircraftEOMDoublet(t, x, u0, doublet_size, doublet_time, ...
               wind_inertial, aircraft_parameters), tspan, x0, options);

%% Build control input array for plotting
control_array = repmat(u0, 1, length(t));
for i = 1:length(t)
    if t(i) > 0 && t(i) <= doublet_time
        control_array(1,i) = u0(1) + doublet_size;
    elseif t(i) > doublet_time && t(i) <= 2*doublet_time
        control_array(1,i) = u0(1) - doublet_size;
    else
        control_array(1,i) = u0(1);
    end
end

%% Plot results
PlotAircraftSim(t', X', control_array, [1,2,3,4,5,6], 'b');

%% Estimate long period natural frequency and damping ratio
theta = X(:, 5);  % pitch angle in radians

% Plot theta 
figure(7);
plot(t, rad2deg(theta), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('\theta (deg)');
title('Pitch Angle - Short Period Response');
grid on;
xline(0.5, 'r--', 'Start of SP response');
xline(1.5, 'g--', 'End of SP response');

%% Print values 3s (Find A and t on Pitch plot)

% Read from plot
theta_trim  = 0.02780;               % rad - trim pitch angle
peak_value  = 0.0305892;                % rad - height of first peak (read from plot)
settling_time = 2.5;                 % sec - when theta looks settled (read from plot)

% Amplitude above trim
A1 = peak_value - theta_trim;        % peak amplitude above trim

% Estimate wn and zeta from settling time
% Peak occurs at: t_peak = pi / wd
t_peak = 0.70;                       % sec - time of first peak (read from plot)
wd_sp  = pi / t_peak;                % damped natural frequency

% Estimate zeta from settling time
% ts ≈ 4 / (zeta * wn) → zeta*wn = 4/ts
zeta_wn = 4 / settling_time;         % zeta * wn product
wn_sp   = sqrt(wd_sp^2 + zeta_wn^2);% natural frequency
zeta_sp = zeta_wn / wn_sp;           % damping ratio
T_sp    = 2*pi / wd_sp;              % period

fprintf('\nShort Period Results (Settling Time Method):\n');
fprintf('  Period           = %.4f sec\n', T_sp);
fprintf('  Damped freq (wd) = %.4f rad/s\n', wd_sp);
fprintf('  Damping ratio    = %.4f\n', zeta_sp);
fprintf('  Natural freq(wn) = %.4f rad/s\n', wn_sp);