#include "motors.h"
#include "user.h"
#include "attitude.h"
#include "joystick.h"
#include "maint.h"

#include <math.h>
#include <pico/time.h>


Servo motor1(MOTOR_1_GPIO);
Servo motor2(MOTOR_2_GPIO);
Servo motor3(MOTOR_3_GPIO);
Servo motor4(MOTOR_4_GPIO);

PID_CONTROL_TAG pid_roll;
PID_CONTROL_TAG pid_pitch;
PID_CONTROL_TAG pid_yaw;

uint32_t MOTOR_Throttle;


const double RPM_MODEL_A = 7.368210869723635e-9;
const double RPM_MODEL_B = -1.764534157839651e-4;
const double RPM_MODEL_C = 1.436568967923393;
const double RPM_MODEL_D = -2622.828248110786;

static double min_rpm;
static double max_rpm;

static uint32_t rpm_to_pwm(uint32_t rpm)
{
    double x = static_cast<double>(rpm);
    double px = pow(x, 3.0) * RPM_MODEL_A +
                pow(x, 2.0) * RPM_MODEL_B +
                x * RPM_MODEL_C +
                RPM_MODEL_D;

    return static_cast<uint32_t>(round(px));
}


static void init_pwm()
{
    motor1.writeMicroseconds(MOTOR_MIN_SIGNAL);
    motor2.writeMicroseconds(MOTOR_MIN_SIGNAL);
    motor3.writeMicroseconds(MOTOR_MIN_SIGNAL);
    motor4.writeMicroseconds(MOTOR_MIN_SIGNAL);
}


void MOTORS_Init()
{
    MOTOR_Throttle = 0;
    min_rpm = 5700;
    max_rpm = 12000;
    
    pid_reset(&pid_roll);
    pid_reset(&pid_pitch);
    pid_reset(&pid_yaw);

    motor1.attach();
    motor2.attach();
    motor3.attach();
    motor4.attach();

    init_pwm();    
}

static void rotateRollPitch(double roll, double pitch, double& newRoll, double& newPitch)
{
	// Angolo di rotazione in radianti (45°)
	static const double angleZ = M_PI / 4.0;

	// Calcola seno e coseno dell'angolo
	static const double cosZ = std::cos(angleZ);
	static const double sinZ = std::sin(angleZ);

	// Ruota roll e pitch rispetto all'asse Z
	newRoll = roll * cosZ - pitch * sinZ;
	newPitch = roll * sinZ + pitch * cosZ;
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
    
    MOTOR_Throttle = to_range(JOYSTICK_Throttle, RADIO_MIN_SIGNAL, RADIO_MAX_SIGNAL, min_rpm, max_rpm);

    if (JOYSTICK_MotorsArmed)
    {
        double body_roll = ATTITUDE_RelRoll();
        double body_pitch = ATTITUDE_RelPitch();
        double body_roll_rotated = 0.0;
        double body_pitch_rotated = 0.0;

        rotateRollPitch(body_roll, body_pitch, body_roll_rotated, body_pitch_rotated);
        
        pid_controller(&pid_roll, pid_roll_gain, JOYSTICK_Roll, body_roll_rotated);
        pid_controller(&pid_pitch, pid_pitch_gain, JOYSTICK_Pitch, body_pitch_rotated);

        uint32_t m1_rpm = uint32_t(round(MOTOR_Throttle - pid_pitch.output));
        uint32_t m2_rpm = uint32_t(round(MOTOR_Throttle - pid_roll.output));
        uint32_t m3_rpm = uint32_t(round(MOTOR_Throttle + pid_roll.output));
        uint32_t m4_rpm = uint32_t(round(MOTOR_Throttle + pid_pitch.output));
    
        m1_signal = rpm_to_pwm(m1_rpm);
        m2_signal = rpm_to_pwm(m2_rpm);
        m3_signal = rpm_to_pwm(m3_rpm);
        m4_signal = rpm_to_pwm(m4_rpm);

        // Con gli RPM in teoria non dovrei mai modificare le soglie minime e massime di PWM da maintenance
        m1_signal = to_range(m1_signal, MOTOR_MIN_SIGNAL, MOTOR_MAX_SIGNAL, MAINT_MotorsParameters[int(MOTORS::M1)][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)], MAINT_MotorsParameters[int(MOTORS::M1)][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)]);
        m2_signal = to_range(m2_signal, MOTOR_MIN_SIGNAL, MOTOR_MAX_SIGNAL, MAINT_MotorsParameters[int(MOTORS::M2)][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)], MAINT_MotorsParameters[int(MOTORS::M2)][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)]);
        m3_signal = to_range(m3_signal, MOTOR_MIN_SIGNAL, MOTOR_MAX_SIGNAL, MAINT_MotorsParameters[int(MOTORS::M3)][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)], MAINT_MotorsParameters[int(MOTORS::M3)][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)]);
        m4_signal = to_range(m4_signal, MOTOR_MIN_SIGNAL, MOTOR_MAX_SIGNAL, MAINT_MotorsParameters[int(MOTORS::M4)][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)], MAINT_MotorsParameters[int(MOTORS::M4)][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)]);
    }
    else
    {
        pid_reset(&pid_roll);
        pid_reset(&pid_pitch);
        pid_reset(&pid_yaw);
    }

    m1_signal = MAINT_MotorsParameters[int(MOTORS::M1)][int(MAINT_MOTOR_PARAM::ENABLED)] ? m1_signal : MOTOR_MIN_SIGNAL;
    m2_signal = MAINT_MotorsParameters[int(MOTORS::M2)][int(MAINT_MOTOR_PARAM::ENABLED)] ? m2_signal : MOTOR_MIN_SIGNAL;
    m3_signal = MAINT_MotorsParameters[int(MOTORS::M3)][int(MAINT_MOTOR_PARAM::ENABLED)] ? m3_signal : MOTOR_MIN_SIGNAL;
    m4_signal = MAINT_MotorsParameters[int(MOTORS::M4)][int(MAINT_MOTOR_PARAM::ENABLED)] ? m4_signal : MOTOR_MIN_SIGNAL;

    motor1.writeMicroseconds(m1_signal);
    motor4.writeMicroseconds(m4_signal);

    motor2.writeMicroseconds(m2_signal);
    motor3.writeMicroseconds(m3_signal);
}