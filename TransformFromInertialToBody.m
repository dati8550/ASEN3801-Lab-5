function v_body = TransformFromInertialToBody(v_inertial, euler_angles)
% TransformFromInertialToBody
% Performs a 3-2-1 (ZYX) rotation from inertial frame to body frame
%
% Inputs:
%   v_inertial  - 3x1 vector in inertial frame
%   euler_angles - [phi; theta; psi]
%
% Output:
%   v_body - 3x1 vector in body frame

phi   = euler_angles(1);
theta = euler_angles(2);
psi   = euler_angles(3);

%% Trig
cphi = cos(phi);   sphi = sin(phi);
cth  = cos(theta); sth  = sin(theta);
cpsi = cos(psi);   spsi = sin(psi);

%% 3-2-1 (ZYX) rotation matrix: Inertial → Body
R = [ ...
    cpsi*cth, ...
    spsi*cth, ...
   -sth;

    cpsi*sth*sphi - spsi*cphi, ...
    spsi*sth*sphi + cpsi*cphi, ...
    cth*sphi;

    cpsi*sth*cphi + spsi*sphi, ...
    spsi*sth*cphi - cpsi*sphi, ...
    cth*cphi ];

%% Transform
v_body = R * v_inertial;

end
