N = 100;

m_f450 = 0.837; # Kg
g = 9.81; # ms^-2

I_x = 0.0124; # [Kg * m^2]
I_y = 0.0130; # [Kg * m^2]
I_z = 0.0237;  # [Kg * m^2]

vertical_thrust_v = zeros(1, N);
torque_roll_v = zeros(1, N);
torque_pitch_v = zeros(1, N);
torque_yaw_v = zeros(1, N);
vert_vel = zeros(1, N);
roll_vel = zeros(1, N);
pitch_vel = zeros(1, N);
yaw_vel = zeros(1, N);
roll = zeros(1, N);
pitch = zeros(1, N);
yaw = zeros(1, N);

%motor_signals = linspace(1000, 2000, N);
motor_signals = ones(1, N) * 1100; #1217 last value before takeoff
dt = 0.01;


pid_u = 0;
ykm1 = 0;
ikm1 = 0;
dkm1 = 0;
kp = 1.0;
ki = 0;
sat = 50;
ad = 0;
bd = 10;
kt = 0;


ti = zeros(1, N);
for i = 1:N
  motor_signal = motor_signals(i);
  m1_signal = motor_signal - pid_u
  m2_signal = motor_signal + pid_u
  m3_signal = motor_signal + pid_u
  m4_signal = motor_signal - pid_u

  m1 = (m1_signal - 1000)/1000;
  m2 = (m2_signal - 1000)/1000;
  m3 = (m3_signal - 1000)/1000;
  m4 = (m4_signal - 1000)/1000;
  ti(i) = i * dt;

  torque_v = f450_torque(m1, m2, m3, m4);
  vertical_thrust = torque_v(1);
  torque_roll = torque_v(2);
  torque_pitch = torque_v(3);
  torque_yaw = torque_v(4);

  vertical_thrust_v(i) = vertical_thrust;
  torque_roll_v(i) = torque_roll;
  torque_pitch_v(i) = torque_pitch;
  torque_yaw_v(i) = torque_yaw;

  if i > 1
    acc_z = (vertical_thrust - m_f450 * g) / m_f450; # [ms^-2]
    acc = [torque_roll / I_x; torque_pitch / I_y; torque_yaw / I_z];
    vert_vel(i) = vert_vel(i - 1) + dt * acc_z;
    roll_vel(i) = roll_vel(i - 1) + dt * acc(1);
    pitch_vel(i) = pitch_vel(i - 1) + dt * acc(2);
    yaw_vel(i) = yaw_vel(i - 1) + dt * acc(2);
    roll(i) = (roll(i - 1) + dt * roll_vel(i));
    pitch(i) = (pitch(i - 1) + dt * pitch_vel(i));
    yaw(i) = (yaw(i - 1) + dt * yaw_vel(i));

    [pid_u, ykm1, ikm1, dkm1] = pid_controller (0, roll(i), kp, ki, kt, sat, ad, bd, ykm1, ikm1, dkm1);

  endif
endfor

figure();
hold on;
grid on;
%plot(ti, vert_vel, '.-', "DisplayName", "Vertical speed");
plot(ti, roll_vel,'.-', "DisplayName", "Roll vel");
plot(ti, roll,'.-', "DisplayName", "Roll");
%plot(ti, pitch,'.-', "DisplayName", "Pitch");
%plot(ti, yaw, '.-', "DisplayName", "Yaw");
legend(gca, 'show');
