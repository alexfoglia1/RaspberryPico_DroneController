#include "motors.h"
#include "user.h"
#include "attitude.h"
#include "joystick.h"
#include "maint.h"

#include <pico/time.h>


Servo motor1(MOTOR_1_GPIO);
Servo motor2(MOTOR_2_GPIO);
Servo motor3(MOTOR_3_GPIO);
Servo motor4(MOTOR_4_GPIO);

PID_CONTROL_TAG pid_roll;
PID_CONTROL_TAG pid_pitch;
PID_CONTROL_TAG pid_yaw;

static bool wereArmed = false;

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

    motor1.attach();
    motor2.attach();
    motor3.attach();
    motor4.attach();

    calib();    
}


void MOTORS_Handler()
{
    uint32_t m1_signal = MOTOR_MIN_SIGNAL;
    uint32_t m2_signal = MOTOR_MIN_SIGNAL; 
    uint32_t m3_signal = MOTOR_MIN_SIGNAL;
    uint32_t m4_signal = MOTOR_MIN_SIGNAL;

    float pid_roll_gain[int(MAINT_PID_PARAM::SIZE)];
    float pid_pitch_gain[int(MAINT_PID_PARAM::SIZE)];
    float pid_yaw_gain[int(MAINT_PID_PARAM::SIZE)];

    pid_roll_gain[int(MAINT_PID_PARAM::PID_KP)]  = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::ROLL)][int(MAINT_PID_PARAM::PID_KP)]);
    pid_roll_gain[int(MAINT_PID_PARAM::PID_KI)]  = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::ROLL)][int(MAINT_PID_PARAM::PID_KI)]);
    pid_roll_gain[int(MAINT_PID_PARAM::PID_KT)]  = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::ROLL)][int(MAINT_PID_PARAM::PID_KT)]);
    pid_roll_gain[int(MAINT_PID_PARAM::PID_SAT)] = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::ROLL)][int(MAINT_PID_PARAM::PID_SAT)]);
    pid_roll_gain[int(MAINT_PID_PARAM::PID_AD)]  = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::ROLL)][int(MAINT_PID_PARAM::PID_AD)]);
    pid_roll_gain[int(MAINT_PID_PARAM::PID_BD)]  = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::ROLL)][int(MAINT_PID_PARAM::PID_BD)]);

    pid_pitch_gain[int(MAINT_PID_PARAM::PID_KP)]  = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::PITCH)][int(MAINT_PID_PARAM::PID_KP)]);
    pid_pitch_gain[int(MAINT_PID_PARAM::PID_KI)]  = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::PITCH)][int(MAINT_PID_PARAM::PID_KI)]);
    pid_pitch_gain[int(MAINT_PID_PARAM::PID_KT)]  = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::PITCH)][int(MAINT_PID_PARAM::PID_KT)]);
    pid_pitch_gain[int(MAINT_PID_PARAM::PID_SAT)] = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::PITCH)][int(MAINT_PID_PARAM::PID_SAT)]);
    pid_pitch_gain[int(MAINT_PID_PARAM::PID_AD)]  = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::PITCH)][int(MAINT_PID_PARAM::PID_AD)]);
    pid_pitch_gain[int(MAINT_PID_PARAM::PID_BD)]  = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::PITCH)][int(MAINT_PID_PARAM::PID_BD)]);

    pid_yaw_gain[int(MAINT_PID_PARAM::PID_KP)]  = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::YAW)][int(MAINT_PID_PARAM::PID_KP)]);
    pid_yaw_gain[int(MAINT_PID_PARAM::PID_KI)]  = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::YAW)][int(MAINT_PID_PARAM::PID_KI)]);
    pid_yaw_gain[int(MAINT_PID_PARAM::PID_KT)]  = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::YAW)][int(MAINT_PID_PARAM::PID_KT)]);
    pid_yaw_gain[int(MAINT_PID_PARAM::PID_SAT)] = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::YAW)][int(MAINT_PID_PARAM::PID_SAT)]);
    pid_yaw_gain[int(MAINT_PID_PARAM::PID_AD)]  = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::YAW)][int(MAINT_PID_PARAM::PID_AD)]);
    pid_yaw_gain[int(MAINT_PID_PARAM::PID_BD)]  = *reinterpret_cast<float*>(&MAINT_PidParameters[int(EULER_ANGLES::YAW)][int(MAINT_PID_PARAM::PID_BD)]);

    pid_controller(&pid_roll, pid_roll_gain, JOYSTICK_Roll, ATTITUDE_Roll);
    pid_controller(&pid_pitch, pid_pitch_gain, JOYSTICK_Pitch, ATTITUDE_Pitch);
    pid_controller(&pid_yaw, pid_yaw_gain, 0.0f, ATTITUDE_Yaw);

    if (JOYSTICK_MotorsArmed)
    {
        float m1_signal_armed = to_range(JOYSTICK_Throttle, RADIO_MIN_SIGNAL, RADIO_MAX_SIGNAL, MAINT_MotorsParameters[int(MOTORS::M1)][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)] + MOTOR_ARMED_THRESHOLD, MAINT_MotorsParameters[int(MOTORS::M1)][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)]);
        float m2_signal_armed = to_range(JOYSTICK_Throttle, RADIO_MIN_SIGNAL, RADIO_MAX_SIGNAL, MAINT_MotorsParameters[int(MOTORS::M2)][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)] + MOTOR_ARMED_THRESHOLD, MAINT_MotorsParameters[int(MOTORS::M2)][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)]);
        float m3_signal_armed = to_range(JOYSTICK_Throttle, RADIO_MIN_SIGNAL, RADIO_MAX_SIGNAL, MAINT_MotorsParameters[int(MOTORS::M3)][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)] + MOTOR_ARMED_THRESHOLD, MAINT_MotorsParameters[int(MOTORS::M3)][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)]);
        float m4_signal_armed = to_range(JOYSTICK_Throttle, RADIO_MIN_SIGNAL, RADIO_MAX_SIGNAL, MAINT_MotorsParameters[int(MOTORS::M4)][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)] + MOTOR_ARMED_THRESHOLD, MAINT_MotorsParameters[int(MOTORS::M4)][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)]);

        if (MAINT_MotorsParameters[int(MOTORS::M1)][int(MAINT_MOTOR_PARAM::ENABLED)] > 0) m1_signal = uint32_t(m1_signal_armed + pid_roll.u - pid_pitch.u - pid_yaw.u);
        if (MAINT_MotorsParameters[int(MOTORS::M2)][int(MAINT_MOTOR_PARAM::ENABLED)] > 0) m2_signal = uint32_t(m2_signal_armed - pid_roll.u - pid_pitch.u + pid_yaw.u);
        if (MAINT_MotorsParameters[int(MOTORS::M3)][int(MAINT_MOTOR_PARAM::ENABLED)] > 0) m3_signal = uint32_t(m3_signal_armed - pid_roll.u + pid_pitch.u - pid_yaw.u);
        if (MAINT_MotorsParameters[int(MOTORS::M4)][int(MAINT_MOTOR_PARAM::ENABLED)] > 0) m4_signal = uint32_t(m4_signal_armed + pid_roll.u + pid_pitch.u + pid_yaw.u);
    }
    
    if (!JOYSTICK_MotorsArmed && wereArmed)
    {
        /** Reset pid when switch from armed to disarmed */
        /** m_signals = MOTOR_MIN_SIGNAL as initialized **/
        pid_reset(&pid_roll);
        pid_reset(&pid_pitch);
        pid_reset(&pid_yaw);
    }
    wereArmed = JOYSTICK_MotorsArmed;

    m1_signal = saturate(m1_signal, MOTOR_MIN_SIGNAL, MOTOR_MAX_SIGNAL);
    m2_signal = saturate(m2_signal, MOTOR_MIN_SIGNAL, MOTOR_MAX_SIGNAL);
    m3_signal = saturate(m3_signal, MOTOR_MIN_SIGNAL, MOTOR_MAX_SIGNAL);
    m4_signal = saturate(m4_signal, MOTOR_MIN_SIGNAL, MOTOR_MAX_SIGNAL);

    motor1.writeMicroseconds(m1_signal);
    motor2.writeMicroseconds(m2_signal);
    motor3.writeMicroseconds(m3_signal);
    motor4.writeMicroseconds(m4_signal);
}