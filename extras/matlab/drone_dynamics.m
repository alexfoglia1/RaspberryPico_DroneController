function [d_state] = drone_dynamics(state, pwm, I, m, d, kf, km)
% DRONE_DYNAMICS Calculates the dynamics of a quadcopter
%
% INPUTS:
%   state - Current state vector [x; y; z; roll; pitch; yaw; dx; dy; dz; droll; dpitch; dyaw]
%   pwm - Motor speeds [omega1; omega2; omega3; omega4] (pwm period us)
%   I - Moments of inertia [Ixx; Iyy; Izz] (kg*m^2)
%   m - Mass of the drone (kg)
%   d - Distance from the center to the motor (m)
%   kf - Thrust coefficient (N/(rad/s)^2)
%   km - Drag coefficient (Nm/(rad/s)^2)
%
% OUTPUT:
%   d_state - Time derivative of the state vector

% Unpack state variables
x = state(1); y = state(2); z = state(3);
roll = state(4); pitch = state(5); yaw = state(6);
dx = state(7); dy = state(8); dz = state(9);
droll = state(10); dpitch = state(11); dyaw = state(12);

% Moments of inertia

Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);

A = 65.01599999999989165644;
B = 2079.60000000000763975549;
omega_rpm = A * (100*((pwm - 1000)/1000)) + B;
omega = (omega_rpm / 60) * 2 * pi;

% Motor forces
F1 = kf * omega(1)^2;
F2 = kf * omega(2)^2;
F3 = kf * omega(3)^2;
F4 = kf * omega(4)^2;

% Total thrust and torques
Tz = F1 + F2 + F3 + F4;
Tx = d * (F1 - F4);
Ty = d * (F2 - F3);
Tyaw = km * (omega(1)^2 - omega(2)^2 + omega(3)^2 - omega(4)^2);

% Rotation matrix (body to world frame)
R = [
    cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
    sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
    -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)
];

% Linear accelerations
acc = R * [0; 0; Tz / m] - [0; 0; 9.81]; % Subtract gravity

% Angular accelerations
ang_acc = [
    Tx / Ixx;
    Ty / Iyy;
    Tyaw / Izz
];

% Output derivatives of state
% [dx; dy; dz; droll; dpitch; dyaw; ddx; ddy; ddz; ddroll; ddpitch; ddyaw]
d_state = [
    dx;
    dy;
    dz;
    droll;
    dpitch;
    dyaw;
    acc(1);
    acc(2);
    acc(3);
    ang_acc(1);
    ang_acc(2);
    ang_acc(3);
];
end

