#include "motors.h"
#include "user.h"
#include "attitude.h"
#include "joystick.h"
#include "maint.h"

#include <pico/time.h>


Servo motor1(MOTOR_1);
Servo motor2(MOTOR_2);
Servo motor3(MOTOR_3);
Servo motor4(MOTOR_4);
PID_CONTROL_TAG pid_roll;
PID_CONTROL_TAG pid_pitch;
PID_CONTROL_TAG pid_yaw;

static float pid_roll_gain[PID_PARAMS_SIZE];
static float pid_pitch_gain[PID_PARAMS_SIZE];
static float pid_yaw_gain[PID_PARAMS_SIZE];

static void calib()
{
    motor1.writeMicroseconds(MOTOR_MAX_SIGNAL);
    motor2.writeMicroseconds(MOTOR_MAX_SIGNAL);
    motor3.writeMicroseconds(MOTOR_MAX_SIGNAL);
    motor4.writeMicroseconds(MOTOR_MAX_SIGNAL);
    
    sleep_ms(2000);

    motor1.writeMicroseconds(MOTOR_MIN_SIGNAL);
    motor2.writeMicroseconds(MOTOR_MIN_SIGNAL);
    motor3.writeMicroseconds(MOTOR_MIN_SIGNAL);
    motor4.writeMicroseconds(MOTOR_MIN_SIGNAL);
}

void MOTORS_Init()
{
    pid_reset(&pid_roll);
    pid_reset(&pid_pitch);
    pid_reset(&pid_yaw);

    pid_roll_gain[PID_KP] = 5.0;
    pid_roll_gain[PID_KI] = 0.0;
    pid_roll_gain[PID_KT] = 0.0;
    pid_roll_gain[PID_AD] = 0.0;
    pid_roll_gain[PID_BD] = 0.0;
    pid_roll_gain[PID_SAT] = 50.0f;

    pid_pitch_gain[PID_KP] = 5.0f;
    pid_pitch_gain[PID_KI] = 0.0f;
    pid_pitch_gain[PID_KT] = 0.0f;
    pid_pitch_gain[PID_AD] = 0.0f;
    pid_pitch_gain[PID_BD] = 0.0f;
    pid_pitch_gain[PID_SAT] = 50.0f;

    pid_yaw_gain[PID_KP] = 1.0f;
    pid_yaw_gain[PID_KI] = 0.0f;
    pid_yaw_gain[PID_KT] = 0.0f;
    pid_yaw_gain[PID_AD] = 0.0f;
    pid_yaw_gain[PID_BD] = 0.0f;
    pid_yaw_gain[PID_SAT] = 50.0f;

    motor1.attach();
    motor2.attach();
    motor3.attach();
    motor4.attach();

    calib();    
}


void MOTORS_Handler()
{
    uint32_t m1_signal = motor_parameters[0][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)];
    uint32_t m2_signal = motor_parameters[1][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)];
    uint32_t m3_signal = motor_parameters[2][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)];
    uint32_t m4_signal = motor_parameters[3][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)];

    if (JOYSTICK_MotorsArmed)
    {
        float m1_signal_armed = to_range(JOYSTICK_Throttle, RADIO_MIN_SIGNAL, RADIO_MAX_SIGNAL, m1_signal + MOTOR_ARMED_THRESHOLD, motor_parameters[0][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)]);
        float m2_signal_armed = to_range(JOYSTICK_Throttle, RADIO_MIN_SIGNAL, RADIO_MAX_SIGNAL, m2_signal + MOTOR_ARMED_THRESHOLD, motor_parameters[1][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)]);
        float m3_signal_armed = to_range(JOYSTICK_Throttle, RADIO_MIN_SIGNAL, RADIO_MAX_SIGNAL, m3_signal + MOTOR_ARMED_THRESHOLD, motor_parameters[2][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)]);
        float m4_signal_armed = to_range(JOYSTICK_Throttle, RADIO_MIN_SIGNAL, RADIO_MAX_SIGNAL, m4_signal + MOTOR_ARMED_THRESHOLD, motor_parameters[3][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)]);
        
        /** Position control loop **/
        pid_controller(&pid_roll, &pid_roll_gain[PID_KP], JOYSTICK_Roll, 0);//ATTITUDE_Roll);
        pid_controller(&pid_pitch, &pid_pitch_gain[PID_KP], JOYSTICK_Pitch, 0);//ATTITUDE_Pitch);

        /** Velocity control loop **/
        pid_controller(&pid_yaw, &pid_yaw_gain[PID_KP], 0.0f, gz_flt_tag.filt_k);

        pid_yaw.u = 0x00;

        if (motor_parameters[0][int(MAINT_MOTOR_PARAM::ENABLED)] > 0) m1_signal = uint32_t(m1_signal_armed + pid_roll.u + pid_pitch.u - pid_yaw.u);
        if (motor_parameters[1][int(MAINT_MOTOR_PARAM::ENABLED)] > 0) m2_signal = uint32_t(m2_signal_armed - pid_roll.u + pid_pitch.u + pid_yaw.u);
        if (motor_parameters[2][int(MAINT_MOTOR_PARAM::ENABLED)] > 0) m3_signal = uint32_t(m3_signal_armed - pid_roll.u - pid_pitch.u - pid_yaw.u);
        if (motor_parameters[3][int(MAINT_MOTOR_PARAM::ENABLED)] > 0) m4_signal = uint32_t(m4_signal_armed + pid_roll.u - pid_pitch.u + pid_yaw.u);
    }
    else
    {
        /** m_signals = MOTOR_MIN_SIGNAL as initialized **/
        pid_reset(&pid_roll);
        pid_reset(&pid_pitch);
        pid_reset(&pid_yaw);
    }

    motor1.writeMicroseconds(m1_signal);
    motor2.writeMicroseconds(m2_signal);
    motor3.writeMicroseconds(m3_signal);
    motor4.writeMicroseconds(m4_signal);
}