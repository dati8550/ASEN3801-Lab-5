function xdot = AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, doublet_size, doublet_time, wind_inertial, aircraft_parameters)

% Constants
 h = -aircraft_state(3); % Dynamically calculate altitude from z-position
 g = 9.81;
 m = aircraft_parameters.m;
 Ix = aircraft_parameters.Ix; Iy = aircraft_parameters.Iy; Iz = aircraft_parameters.Iz;
 Ixz = aircraft_parameters.Ixz;
 Gamma = Ix*Iz - Ixz^2;
 [T, a, P, rho] = stdatmo(h);    % Get Density dynamically

 de_trim = aircraft_surfaces(1); % trim elevator
    

% Evaluate what the elevator should be doing at THIS specific instant in time
if time <= doublet_time
    d_e_effective = de_trim + doublet_size;
elseif time <= 2 * doublet_time
    d_e_effective = de_trim - doublet_size;
else
    d_e_effective = de_trim;
end

% Update the surface vector for this specific time step
current_surfaces = aircraft_surfaces;
current_surfaces(1) = d_e_effective;

% Get values
    phi = aircraft_state(4);
    theta = aircraft_state(5);
    psi = aircraft_state(6);
    u = aircraft_state(7);
    v = aircraft_state(8);
    w = aircraft_state(9);
    p = aircraft_state(10);
    q = aircraft_state(11);
    r = aircraft_state(12);

% Use the given function to find the forces and moments
    [f_aero_body, m_aero_body] = AeroForcesAndMoments(aircraft_state, current_surfaces, wind_inertial, rho, aircraft_parameters);
    
    X = f_aero_body(1);
    Y = f_aero_body(2);
    Z = f_aero_body(3);

    L = m_aero_body(1);
    M = m_aero_body(2);
    N = m_aero_body(3);

% Navigation Equations
cos_the = cos(theta);
sin_the = sin(theta);
cos_phi = cos(phi);
sin_phi = sin(phi);
cos_psi = cos(psi);
sin_psi = sin(psi);

% Body to Inertial frame
dx_e = u*(cos_the*cos_psi) + v*(sin_phi*sin_the*cos_psi - cos_phi*sin_psi) + w*(cos_phi*sin_the*cos_psi + sin_phi*sin_psi);
dy_e = u*(cos_the*sin_psi) + v*(sin_phi*sin_the*sin_psi + cos_phi*cos_psi) + w*(cos_phi*sin_the*sin_psi - sin_phi*cos_psi);
dz_e = -u*sin_the + v*sin_phi*cos_the + w*cos_phi*cos_the;

% Attitude Equations
dphi   = p + q*sin_phi*tan(theta) + r*cos_phi*tan(theta);
dtheta = q*cos_phi - r*sin_phi;
dpsi   = (q*sin_phi + r*cos_phi)/cos_the;

% Translational Dynamics
du = r*v - q*w - g*sin_the + X/m;
dv = p*w - r*u + g*sin_phi*cos_the + Y/m;
dw = q*u - p*v + g*cos_phi*cos_the + Z/m;

% Rotational Dynamics
dp = (Ixz*(Ix-Iy+Iz)*p*q - (Iz*(Iz-Iy)+Ixz^2)*q*r + Iz*L + Ixz*N) / Gamma;
dq = ((Iz-Ix)*p*r - Ixz*(p^2-r^2) + M) / Iy;
dr = (((Ix-Iy)*Ix+Ixz^2)*p*q - Ixz*(Ix-Iy+Iz)*q*r + Ixz*L + Ix*N) / Gamma;

xdot = [dx_e; dy_e; dz_e; dphi; dtheta; dpsi; du; dv; dw; dp; dq; dr];
end
