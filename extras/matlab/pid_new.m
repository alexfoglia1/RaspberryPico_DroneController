ysp = 0.0;
y   = 5.0;

ekm1 = (ysp - y);
ikm1 = 0;

kp = 5.0;
ki = 0.0;
kd = 0.0;
sat = 100;
dt = 0.005;

[pid_roll_output, ek, ik] = pid_controller(ysp, y, kp, ki, kd, dt, sat, ikm1, ekm1);
pid_pitch_output = 0.0;

m1_signal = (1250 - pid_roll_output + pid_pitch_output);
m2_signal = (1250 - pid_roll_output + pid_pitch_output);
m3_signal = (1250 + pid_roll_output - pid_pitch_output);
m4_signal = (1250 + pid_roll_output - pid_pitch_output);

