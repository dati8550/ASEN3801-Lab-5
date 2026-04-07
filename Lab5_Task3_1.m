clear;
close all;
clc;

% From Task 2 Problem 2

x_trim_0 = [0;      0;      0;
            0;  0.02780;    0;
            20.99;  0;   0.5837;
            0;      0;      0];
u_trim_0 = [0.1079; 0; 0; 0.3182];
wind_0 = [0;0;0];             % No wind

% Givens
doublet_time = 0.25;         % seconds
doublet_size_deg = 15;      % degrees
doublet_size_rad = doublet_size_deg * (pi/180);

% Function Calls
aircraft_parameters = ttwistor();


tspan = [0 3];      % Simulate over 3 seconds

%% Run ODE 45
[t_out, x_out] = ode45(@(time, aircraft_state) AircraftEOMDoublet(time, aircraft_state, u_trim_0, doublet_size_rad, doublet_time, wind_0, aircraft_parameters), tspan, x_trim_0);

n = length(t_out);

for i = 1:n;
    t = t_oout(i);
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
PlotAircraftSim(t_out, x_out, u_out, fig_num, 'b');
