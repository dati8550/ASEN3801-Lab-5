function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)

%% Constants
g   = aircraft_parameters.g;
m   = aircraft_parameters.m;
Ix  = aircraft_parameters.Ix;
Iy  = aircraft_parameters.Iy;
Iz  = aircraft_parameters.Iz;
Ixz = aircraft_parameters.Ixz;
Gamma = Ix*Iz - Ixz^2;

%% Atmosphere
h = -aircraft_state(3);
h = max(h, 0);
[rho, ~, ~, ~] = stdatmo(h);

%% Unpack state
phi   = aircraft_state(4);
theta = aircraft_state(5);
psi   = aircraft_state(6);
u     = aircraft_state(7);
v     = aircraft_state(8);
w     = aircraft_state(9);
p     = aircraft_state(10);
q     = aircraft_state(11);
r     = aircraft_state(12);

%% Aero forces and moments
[f_aero_body, m_aero_body] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, rho, aircraft_parameters);

X = f_aero_body(1);
Y = f_aero_body(2);
Z = f_aero_body(3);
L = m_aero_body(1);
M = m_aero_body(2);
N = m_aero_body(3);

%% Trig
cos_the = cos(theta); sin_the = sin(theta);
cos_phi = cos(phi);   sin_phi = sin(phi);
cos_psi = cos(psi);   sin_psi = sin(psi);

%% 1) Navigation equations
dx_e = u*(cos_the*cos_psi) + v*(sin_phi*sin_the*cos_psi - cos_phi*sin_psi) + w*(cos_phi*sin_the*cos_psi + sin_phi*sin_psi);
dy_e = u*(cos_the*sin_psi) + v*(sin_phi*sin_the*sin_psi + cos_phi*cos_psi) + w*(cos_phi*sin_the*sin_psi - sin_phi*cos_psi);
dz_e = -u*sin_the + v*sin_phi*cos_the + w*cos_phi*cos_the;

%% 2) Attitude kinematics
dphi   = p + q*sin_phi*tan(theta) + r*cos_phi*tan(theta);
dtheta = q*cos_phi - r*sin_phi;
dpsi   = (q*sin_phi + r*cos_phi) / cos_the;

%% 3) Translational dynamics
du = r*v - q*w - g*sin_the         + X/m;
dv = p*w - r*u + g*cos_the*sin_phi + Y/m;
dw = q*u - p*v + g*cos_the*cos_phi + Z/m;

%% 4) Rotational dynamics
dp = (Iz*L + Ixz*N - (Iz*(Iz-Iy) + Ixz^2)*q*r + Ixz*(Ix-Iy+Iz)*p*q) / Gamma;
dq = ((Iz-Ix)*p*r - Ixz*(p^2-r^2) + M) / Iy;
dr = (Ixz*L + Ix*N - ((Ix-Iy)*Ix + Ixz^2)*p*q - Ixz*(Ix-Iy+Iz)*q*r) / Gamma;

%% Assemble xdot
xdot = [dx_e; dy_e; dz_e; dphi; dtheta; dpsi; du; dv; dw; dp; dq; dr];

end