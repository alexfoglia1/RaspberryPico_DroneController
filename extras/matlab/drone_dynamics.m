function [roll, pitch, yaw, x, y, z] = drone_dynamics(k_f, k_m, m, I, T, g, tspan)
    % Simula la dinamica del drone in configurazione "a +"
    %
    % Input:
    % k_f: Coefficiente di forza [N/(rad/s)^2]
    % k_m: Coefficiente di momento torcente [Nm/(rad/s)^2]
    % m: Massa del drone [kg]
    % I: Momenti di inerzia [3x3 matrice]
    % T: Percentuali di throttle per i 4 motori [0-1]
    % max_rpm: Numero massimo di RPM
    % g: Accelerazione gravitazionale [m/s^2]
    % tspan: Vettore dei tempi di simulazione
    %
    % Output:
    % roll, pitch, yaw: Angoli di orientamento (rad)
    % x, y, z: Posizioni (m)

    % Calcolo delle velocità angolari dei motori (rad/s)
    %omega_rpm = 65.016 * T + 2079.6

    %omega = (omega_rpm / 60) * 2 * pi

    roll_setpoint = pi/4;
    kp = 0.6;
    ki = 0.0;
    kd = 0.8;
    sat = 1000;

    persistent ek;
    persistent ik;
    persistent roll;
    persistent pitch;
    if isempty(ik)
      ik = 0;
      ek = 0;
      roll = 0;
      pitch = 0;
    endif
    [pid_u, ek, ik] = pid_controller (roll_setpoint, roll, kp, ki, kd, 0.005, sat, ik, ek);

    pwm_setpoint = 1000 + (T / 100) * 1000;
    pwm_m1 = pwm_setpoint(1) + pid_u;
    pwm_m4 = pwm_setpotnt(4) - pid_u;
    pwm_m2 = pwm_setpoint(2);
    pwm_m3 = pwm_setpoint(3);

    if (pwm_m1 < 1000) pwm_m1 = 1000;
    elseif (pwm_m1 > 2000) pwm_m1 = 2000;
    endif

    if (pwm_m2 < 1000) pwm_m2 = 1000;
    elseif (pwm_m2 > 2000) pwm_m2 = 2000;
    endif

    if (pwm_m3 < 1000) pwm_m3 = 1000;
    elseif (pwm_m3 > 2000) pwm_m3 = 2000;
    endif

    if (pwm_m4 < 1000) pwm_m4 = 1000;
    elseif (pwm_m4 > 2000) pwm_m4 = 2000;
    endif

    pwm = [pwm_m1, pwm_m2, pwm_m3, pwm_m4];
    throttle = (pwm - 1000)/1000 * 100;
    omega_rpm = 65.016 * throttle + 2079.6;
    omega = (omega_rpm / 60) * 2 * pi


    % Funzione dinamica del drone
    function dx = dynamics(t, x)
        % Stato:
        % x = [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]

        dx = zeros(12, 1);

        % Forze generate dai motori
        F = k_f * omega.^2;  % Forze [N]
        T_m = k_m * omega.^2;  % Momenti torcenti [Nm]

        % Forza totale lungo Z
        Fz = sum(F);

        % Momenti attorno agli assi in configurazione "a +"
        tau_roll = I(1, 1) * (F(2) - F(3));  % Momento per il roll
        tau_pitch = I(2, 2) * (F(1) - F(4)); % Momento per il pitch
        tau_yaw = T_m(1) - T_m(2) + T_m(3) - T_m(4); % Momento per lo yaw

        % Accelerazioni lineari nel sistema di riferimento globale
        ax = (Fz / m) * (cos(x(7)) * sin(x(8)) * cos(x(9)) + sin(x(7)) * sin(x(9)));
        ay = (Fz / m) * (cos(x(7)) * sin(x(8)) * sin(x(9)) - sin(x(7)) * cos(x(9)));
        az = (Fz / m) * cos(x(8)) - g;

        % Accelerazioni angolari nel sistema di riferimento del drone
        wx_dot = tau_roll / I(1, 1);
        wy_dot = tau_pitch / I(2, 2);
        wz_dot = tau_yaw / I(3, 3);

        % Aggiorna il vettore stato
        dx(1:3) = x(4:6);       % Velocità lineari
        dx(4) = ax;             % Accelerazione x
        dx(5) = ay;             % Accelerazione y
        dx(6) = az;             % Accelerazione z
        dx(7:9) = x(10:12);     % Velocità angolari
        dx(10) = wx_dot;        % Accelerazione angolare roll
        dx(11) = wy_dot;        % Accelerazione angolare pitch
        dx(12) = wz_dot;        % Accelerazione angolare yaw
    end

    % Stato iniziale
    x0 = zeros(12, 1); % [posizioni, velocità lineari, angoli, velocità angolari]

    % Simula la dinamica con ode45
    [T, X] = ode45(@dynamics, tspan, x0);

    % Estrai i risultati
    roll = X(:, 7);   % Roll
    pitch = X(:, 8);  % Pitch
    yaw = X(:, 9);    % Yaw
    x = X(:, 1);      % Posizione x
    y = X(:, 2);      % Posizione y
    z = X(:, 3);      % Posizione z
end

