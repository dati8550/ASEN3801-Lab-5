%% Lab 5 - Problems 1 and 2
clear; clc; close all;

%% Load aircraft parameters
ttwistor;

%% Setup
wind_inertial = [0; 0; 0];
options = odeset('RelTol', 1e-3, 'AbsTol', 1e-5, 'MaxStep', 0.1);

%% Problem 2.1 - Zero state except u and height


x0_1 = [0;          % x_E (m)
         0;          % y_E (m)
         -1609.34;   % z_E (m) - 1 mile altitude
         0;          % phi (rad)
         0;          % theta (rad)
         0;          % psi (rad)
         21;         % u (m/s)
         0;          % v (m/s)
         0;          % w (m/s)
         0;          % p (rad/s)
         0;          % q (rad/s)
         0];         % r (rad/s)

u0_1 = [0; 0; 0; 0];   % all control inputs zero

[t1, X1] = ode45(@(t,x) AircraftEOM(t, x, u0_1, wind_inertial, aircraft_parameters), ...
                 [0 30], x0_1, options);

ctrl1 = repmat(u0_1, 1, length(t1));
PlotAircraftSim(t1', X1', ctrl1, [1,2,3,4,5,6], 'b');


%% Problem 2.2 - Trim initial conditions

x0_2 = [0;          % x_E (m)
         0;          % y_E (m)
         -1800;      % z_E (m)
         0;          % phi (rad)
         0.02780;    % theta (rad)
         0;          % psi (rad)
         20.99;      % u (m/s)
         0;          % v (m/s)
         0.5837;     % w (m/s)
         0;          % p (rad/s)
         0;          % q (rad/s)
         0];         % r (rad/s)

u0_2 = [0.1079; 0; 0; 0.3182];

[t2, X2] = ode45(@(t,x) AircraftEOM(t, x, u0_2, wind_inertial, aircraft_parameters), ...
                 [0 60], x0_2, options);

ctrl2 = repmat(u0_2, 1, length(t2));
PlotAircraftSim(t2', X2', ctrl2, [7,8,9,10,11,12], 'b');

%% Problem 2.3 - Off trim initial conditions

x0_3 = [0;               % x_E (m)
         0;               % y_E (m)
         -1800;           % z_E (m)
         deg2rad(15);     % phi (rad)
         deg2rad(-12);    % theta (rad)
         deg2rad(270);    % psi (rad)
         19;              % u (m/s)
         3;               % v (m/s)
         -2;              % w (m/s)
         deg2rad(0.08);   % p (rad/s)
         deg2rad(-0.2);   % q (rad/s)
         0];              % r (rad/s)

u0_3 = [deg2rad(5); deg2rad(2); deg2rad(-13); 0.3];

[t3, X3] = ode45(@(t,x) AircraftEOM(t, x, u0_3, wind_inertial, aircraft_parameters), ...
                 [0 60], x0_3, options);

ctrl3 = repmat(u0_3, 1, length(t3));
PlotAircraftSim(t3', X3', ctrl3, [13,14,15,16,17,18], 'b');
