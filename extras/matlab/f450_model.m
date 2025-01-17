
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

%k_f = 1.884e-5; % Coefficiente di forza [N/(rad/s)^2]
%k_m = 2.6e-7;   % Coefficiente di momento torcente [Nm/(rad/s)^2]

T = [60, 60, 60, 60]; % Throttle percentage per i 4 motori (60%)

tspan = linspace(0, 10, 1000); % Tempo di simulazione (10 secondi)

% Simula la dinamica del drone
[roll, pitch, yaw, x, y, z] = drone_dynamics(k_f, k_m, m, I, T, g, tspan);

% Visualizza i risultati
figure;
subplot(3, 1, 1);
plot(tspan, roll * 180 / pi, 'r');
xlabel('Tempo (s)'); ylabel('Roll (deg)'); title('Roll');

subplot(3, 1, 2);
plot(tspan, pitch * 180 / pi, 'g');
xlabel('Tempo (s)'); ylabel('Pitch (deg)'); title('Pitch');

subplot(3, 1, 3);
plot(tspan, z, 'b');
xlabel('Tempo (s)'); ylabel('Altitudine (m)'); title('Altitudine');
