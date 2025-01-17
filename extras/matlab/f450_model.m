motor_saturation = @(in, min_val, max_val) max(min(in, max_val), min_val);
% Script principale per testare la funzione

% Parametri del drone
L = 0.225;  % Distanza dal centro ai motori (m)
g = 9.81;  % Accelerazione di gravità (m/s^2)

% Costanti del motore e pale
k_m = 9.08e-7;
k_f = 7.15e-6;

% Matrice di inerzia aggiornata per configurazione a +
m_motor = 0.0527;  % Massa di ciascun motore (kg)
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

% Simula la dinamica del drone
Kp = 10.0;
Ki = 0.0;
Kd = 1.5;

t0 = 0;
tf = 10;
dt = 0.005;
N = (tf-t0)/dt;
tspan = linspace(t0, tf, N); % Tempo di simulazione (10 secondi)

pwm_setpoint = 1550;
pwm = [pwm_setpoint, pwm_setpoint, pwm_setpoint, pwm_setpoint];

roll_setpoint = zeros(1, N);
for i=1:length(roll_setpoint)
  if i > length(roll_setpoint)/2
    roll_setpoint(i) = 10;
  endif
endfor
roll = zeros(1, N);
pitch = zeros(1, N);
z_drone = zeros(1, N);
pid_err = 0;
pid_integral = 0;

state = zeros(12, 1);

it = 1;
while it <= N
  % get state variation
  d_state = drone_dynamics(state, pwm, I, m, L, k_f, k_m);

  % integrate variation over time to obtain current state
  state += d_state * dt;
  z_drone(it) = state(3);
  roll(it) = state(4) * 180 / pi;
  pitch(it) = state(5) * 180 / pi;

  % execute PID controller
  [pid_u, pid_err, pid_integral] = pid_controller (roll_setpoint(it), roll(it), Kp, Ki, Kd, dt, 1000, pid_integral, pid_err);
  m1 = motor_saturation(pwm_setpoint + pid_u, 1000, 2000);
  m4 = motor_saturation(pwm_setpoint - pid_u, 1000, 2000);

  % Correct inputs to follow setpoint
  pwm = [m1, pwm(2), pwm(3), m4];

  % increase iteration counter
  it += 1;
end

% Visualizza i risultati
figure;
subplot(3, 1, 1);
plot(tspan, roll, 'r');
hold on;
plot(tspan, roll_setpoint, 'm');
xlabel('Tempo (s)'); ylabel('Roll (deg)'); title('Roll');
grid on;

subplot(3, 1, 2);
plot(tspan, pitch, 'g');
xlabel('Tempo (s)'); ylabel('Pitch (deg)'); title('Pitch');
grid on;

subplot(3, 1, 3);
plot(tspan, z_drone, 'b');
xlabel('Tempo (s)'); ylabel('Altitude (m)'); title('Altitude');
grid on;
