% Parametri del drone
L = 0.225;  % Distanza dal centro ai motori (m)
g = 9.81;  % Accelerazione di gravità (m/s^2)

% Costanti del motore e pale
k_f = 1.2259232154527175e-05;
k_m = k_f * 0.1;

% Matrice di inerzia aggiornata per configurazione a +
m_motor = 0.048;  % Massa di ciascun motore (kg)
m_ESC = 0.025;    % Massa di ciascun ESC (kg)
m_braccio = 0.055; % Massa di ciascun braccio del frame (kg)
m_frame_centro = 0.070; % Massa centrale del frame (kg)
m_batteria = 0.404; % Massa della batteria (kg)
m = m_motor * 4 + m_ESC * 4 + m_braccio * 4 + m_frame_centro + m_batteria;  % Massa del drone (kg)

% Rollio (I_roll) e Beccheggio (I_pitch)
I_roll = 2 * m_motor * L^2 + 2 * m_ESC * (L/2)^2 + 2 * m_braccio * L^2;
I_pitch = I_roll;

% Yaw (I_yaw)
I_yaw = 4 * (m_motor * L^2 + m_ESC * (L/2)^2 + m_braccio * L^2) + (m_frame_centro + m_batteria) * (0.1)^2;

% Matrice di inerzia
I = diag([I_roll, I_pitch, I_yaw]);

% PID Parameters
Kp_roll = 1.0; Ki_roll = 0.0; Kd_roll = 0.00;
Kp_pitch = 0; Ki_pitch = 0.0; Kd_pitch = 0.00;

% Setpoint dinamici

function out = roll_control(t)
  if t > 2.5
    out = pi/32;
  else
    out = 0;
  endif
end

function out = pitch_control(t)
  if t > 2.5
    out = 0;
  else
    out = 0;
  endif
end

roll_setpoint = @(t) roll_control(t); % Oscillazione sinusoidale per roll
pitch_setpoint = @(t) pitch_control(t); % Oscillazione sinusoidale per pitch

% Funzione per la dinamica del drone
function dxdt = drone_dynamics(t, x, params, rpm)
% Estrai parametri
    if (t == round(t))
      disp(num2str(t));
    endif
    m = params.m; g = params.g; I = params.I;
    k_f = params.k_f; k_m = params.k_m; L = params.L;
    Kp_roll = params.Kp_roll; Ki_roll = params.Ki_roll; Kd_roll = params.Kd_roll;
    Kp_pitch = params.Kp_pitch; Ki_pitch = params.Ki_pitch; Kd_pitch = params.Kd_pitch;

    % Stato attuale
    pos = x(1:3);  % Posizione [x, y, z]
    vel = x(4:6);  % Velocità [vx, vy, vz]
    angles = x(7:9);  % Angoli [phi, theta, psi]
    omega = x(10:12);  % Velocità angolare [p, q, r]

    % Aggiungi rumore di misura agli angoli
    noise_level = deg2rad(0.1); % Livello di rumore massimo ±0.1°
    measured_angles = angles + noise_level * (2 * rand(3, 1) - 1);

    % PID integrale e errori persistenti
    persistent roll_integral pitch_integral roll_prev_error pitch_prev_error
    if isempty(roll_integral)
        roll_integral = 0; pitch_integral = 0;
        roll_prev_error = 0; pitch_prev_error = 0;
    end

    % Setpoint e errore
    roll_error = params.roll_setpoint(t) - measured_angles(1);
    pitch_error = params.pitch_setpoint(t) - measured_angles(2);

    % PID roll
    roll_integral = roll_integral + roll_error * params.dt;
    roll_derivative = (roll_error - roll_prev_error) / params.dt;
    tau_roll = Kp_roll * roll_error + Ki_roll * roll_integral + Kd_roll * roll_derivative;
    roll_prev_error = roll_error;

    % PID pitch
    pitch_integral = pitch_integral + pitch_error * params.dt;
    pitch_derivative = (pitch_error - pitch_prev_error) / params.dt;
    tau_pitch = Kp_pitch * pitch_error + Ki_pitch * pitch_integral + Kd_pitch * pitch_derivative;
    pitch_prev_error = pitch_error;

    % RPM totali per ogni motore
    motor_rpms = rpm + [ tau_pitch;  % Incremento da PID per Pitch
                         tau_roll;   % Incremento da PID per Roll
                        -tau_roll;   % Opposto per Roll
                        -tau_pitch]; % Opposto per Pitch

    % Converti RPM in rad/s
    motor_speeds = motor_rpms / 60 * 2 * pi;

    % Momenti e forze
    Fz = sum(k_f * motor_speeds.^2);
    tau = [L * (k_f * motor_speeds(2)^2 - k_f * motor_speeds(3)^2); % Roll
           L * (k_f * motor_speeds(1)^2 - k_f * motor_speeds(4)^2); % Pitch
           k_m * (motor_speeds(1)^2 - motor_speeds(2)^2 + motor_speeds(3)^2 - motor_speeds(4)^2)]; % Yaw

    % Rotazione attuale
    phi = angles(1); theta = angles(2); psi = angles(3);
    R = [cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta);
         sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), ...
         sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), sin(phi)*cos(theta);
         cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi), ...
         cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi), cos(phi)*cos(theta)];

    % Equazioni di moto
    accel = (1/m) * (R * [0; 0; Fz] - [0; 0; m*g]);  % Accelerazione traslazionale
    omegadot = I \ (tau - cross(omega, I*omega));     % Accelerazione angolare

    % Derivate dello stato
    dxdt = zeros(12, 1);
    dxdt(1:3) = vel;        % Derivata posizione
    dxdt(4:6) = accel;      % Derivata velocità
    dxdt(7:9) = omega;      % Derivata angoli
    dxdt(10:12) = omegadot; % Derivata velocità angolare
end

% Esempio di simulazione
params.m = m;
params.g = g;
params.I = I;
params.k_f = k_f;
params.k_m = k_m;
params.L = L;
params.Kp_roll = Kp_roll; params.Ki_roll = Ki_roll; params.Kd_roll = Kd_roll;
params.Kp_pitch = Kp_pitch; params.Ki_pitch = Ki_pitch; params.Kd_pitch = Kd_pitch;
params.dt = 0.005; % Passo temporale
params.roll_setpoint = roll_setpoint;
params.pitch_setpoint = pitch_setpoint;

% Stato iniziale
x0 = zeros(12, 1);  % [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, phi, theta, psi, p, q, r]

% Tempo di simulazione
tspan = linspace(0, 10, 10 / params.dt);

% Simulazione
% Runge-Kutta 4° ordine a passo fisso
tspan = linspace(0, 30, 30 / params.dt + 1); % Definisci i punti temporali
num_steps = length(tspan);
x = zeros(num_steps, length(x0));
x(1, :) = x0'; % Condizioni iniziali

RPM_SETPOINT = 7200;
for i = 1:num_steps-1
    t = tspan(i);
    h = params.dt; % Passo temporale
    k1 = drone_dynamics(t, x(i, :)', params, RPM_SETPOINT);
    k2 = drone_dynamics(t + h/2, x(i, :)' + h/2 * k1, params, RPM_SETPOINT);
    k3 = drone_dynamics(t + h/2, x(i, :)' + h/2 * k2, params, RPM_SETPOINT);
    k4 = drone_dynamics(t + h, x(i, :)' + h * k3, params, RPM_SETPOINT);
    x(i+1, :) = x(i, :) + (h/6) * (k1 + 2*k2 + 2*k3 + k4)';
end

% Visualizzazione del risultato
figure;
% Traiettoria 3D del centro di massa
subplot(2, 1, 1);
plot3(x(:, 1), x(:, 2), x(:, 3), 'b-');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Traiettoria 3D del Centro di Massa');
grid on;
axis equal;

subplot(2, 1, 2);
plot(tspan, rad2deg(x(:, 7)), 'r-', tspan, rad2deg(arrayfun(roll_setpoint, tspan)), 'g-', ...
     tspan, rad2deg(x(:, 8)), 'b-', tspan, rad2deg(arrayfun(pitch_setpoint, tspan)), 'm-');
xlabel('Tempo (s)');
ylabel('Angoli e Setpoint (deg)');
title('Angoli e Setpoint - Roll e Pitch');
legend('Roll', 'Roll Setpoint', 'Pitch', 'Pitch Setpoint');
grid on;

