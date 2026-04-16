clear;
close all;
clc;

% From Task 2 Problem 2

x_trim_0 = [ ...
    0;                      % x_e (m)
    0;                      % y_e (m)
    -1800;                  % z_e (m) -> Altitude is 1800m
    0 * (pi/180);           % phi (roll)   -> Converted from 0 deg
    0.02780;                % theta (pitch)-> ALREADY IN RAD
    0 * (pi/180);           % psi (yaw)    -> Converted from 0 deg
    20.99;                  % u (m/s)
    0;                      % v (m/s)
    0.5837;                 % w (m/s)
    0 * (pi/180);           % p (roll rate) -> Converted from 0 deg/s
    0 * (pi/180);           % q (pitch rate)-> Converted from 0 deg/s
    0 * (pi/180)            % r (yaw rate)  -> Converted from 0 deg/s
];

u_trim_0 = [0.1079; 
            0; 
            0; 
            0.3182];

wind_0 = [0;0;0];             % No wind

% Givens
doublet_time = 0.25;         % seconds
doublet_size_deg = 15;      % degrees
doublet_size_rad = doublet_size_deg * (pi/180); % Switch to radians

% Function Call to get aircraft aparameters
aircraft_parameters = ttwistor();

h = 1609;
[~, ~, ~, rho] = stdatmo(h);
aircraft_parameters.rho = rho;

tspan = 0:0.01:3;      % Simulate over 3 seconds

%% Run ODE 45
options = odeset('MaxStep', 0.01, 'RelTol', 1e-3, 'AbsTol', 1e-3);
[t_out, x_out] = ode45(@(time, aircraft_state) AircraftEOMDoublet(time, aircraft_state, u_trim_0, doublet_size_rad, doublet_time, wind_0, aircraft_parameters), tspan, x_trim_0, options);n = length(t_out);

for i = 1:n;
    t = t_out(i);
    de_trim = u_trim_0(1);
    if 0 < t && t <= doublet_time
        de = de_trim + doublet_size_rad;
    elseif t > doublet_time && t <= 2*doublet_time
        de =  de_trim + doublet_size_rad;
    else
        de = de_trim;
    end

    u_out(i,:) = [de, u_trim_0(2), u_trim_0(3), u_trim_0(4)];
end

fig_num = 1;        
PlotAircraftSim(t_out, x_out, u_out, 139:144, '-b');
