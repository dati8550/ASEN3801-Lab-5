%% Lab 5 Task 3.2
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

%% Run simulation 
tspan = [0 100]; % Hardcode this for different duration

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
title('Pitch Angle - Long Period Response');
grid on;
xline(0.5, 'r--', 'Start of LP response');
xline(1.5, 'g--', 'End of LP response');

%% Print values 100s (Find A and t on Body Velocity plot)

u_trim = 20.99;

A1 = 21.0084 - u_trim;                     % amplitude of first peak (m/s)
A2 = 21.0001 - u_trim;                     % amplitude of second peak (m/s)
t1 = 10.7;                                 % time of first peak (s)
t2 = 22.38;                                % time of second peak (s)

T_ph    = t2 - t1;                         % period
wd_ph   = 2*pi / T_ph;                     % damped natural frequency
delta   = log(A1 / A2);                    % logarithmic decrement
zeta_ph = delta / sqrt(4*pi^2 + delta^2);  % damping ratio
wn_ph   = wd_ph / sqrt(1 - zeta_ph^2);     % natural frequency

fprintf('Phugoid Mode Results:\n');
fprintf('  Period           = %.4f sec\n', T_ph);
fprintf('  Damped freq (wd) = %.4f rad/s\n', wd_ph);
fprintf('  Damping ratio    = %.4f\n', zeta_ph);
fprintf('  Natural freq(wn) = %.4f rad/s\n', wn_ph);