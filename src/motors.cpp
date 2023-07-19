#include "motors.h"
#include "user.h"
#include "attitude.h"
#include "maint.h"
#include "joystick.h"


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


static uint32_t cmd_vel(float pid_u)
{
    return pid_u < MOTOR_MIN_SIGNAL ? MOTOR_MIN_SIGNAL :
           pid_u > MOTOR_MAX_SIGNAL ? MOTOR_MAX_SIGNAL : pid_u;
}


void MOTORS_Init()
{
    pid_reset(&pid_roll);
    pid_reset(&pid_pitch);
    pid_reset(&pid_yaw);

    pid_roll_gain[PID_KP] = 1.0f;
    pid_roll_gain[PID_KI] = 0.0f;
    pid_roll_gain[PID_KT] = 0.0f;
    pid_roll_gain[PID_AD] = 0.0f;
    pid_roll_gain[PID_BD] = 0.0f;
    pid_roll_gain[PID_SAT] = 50.0f;

    pid_pitch_gain[PID_KP] = 1.0f;
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
}


void MOTORS_Handler()
{
    uint32_t m1_signal = MOTOR_MIN_SIGNAL;
    uint32_t m2_signal = MOTOR_MIN_SIGNAL;
    uint32_t m3_signal = MOTOR_MIN_SIGNAL;
    uint32_t m4_signal = MOTOR_MIN_SIGNAL;

    if (JOYSTICK_MotorsArmed)
    {
        /** Position control loop **/
        //pid_controller(&pid_roll, &pid_roll_gain[PID_KP], JOYSTICK_Roll, ATTITUDE_Roll);
        //pid_controller(&pid_pitch, &pid_pitch_gain[PID_KP], JOYSTICK_Roll, ATTITUDE_Pitch);

        /** Velocity control loop **/
        //pid_controller(&pid_yaw, &pid_yaw_gain[PID_KP], 0.0f, gz_flt_tag.filt_k);

        m1_signal = cmd_vel(JOYSTICK_Throttle + pid_roll.u + pid_pitch.u - pid_yaw.u);
        m2_signal = cmd_vel(JOYSTICK_Throttle + pid_roll.u + pid_pitch.u - pid_yaw.u);
        m3_signal = cmd_vel(JOYSTICK_Throttle + pid_roll.u + pid_pitch.u - pid_yaw.u);
        m4_signal = cmd_vel(JOYSTICK_Throttle + pid_roll.u + pid_pitch.u - pid_yaw.u);


    }
    else if (MAINT_IsPresent())
    {
        m1_signal = motor1.currentSignal();
        m2_signal = motor2.currentSignal();
        m3_signal = motor3.currentSignal();
        m4_signal = motor4.currentSignal();
    }
    else
    {
        /** m_signals = MOTOR_MIN_SIGNAL as initialized **/
        ;
    }

    motor1.writeMicroseconds(m1_signal);
    motor2.writeMicroseconds(m2_signal);
    motor3.writeMicroseconds(m3_signal);
    motor4.writeMicroseconds(m4_signal);
}