#ifndef MAINT_H
#define MAINT_H

#include <stdint.h>
#include <hardware/uart.h>

#define MAINT_PAYLOAD_SIZE 512
#define MAINT_SYNC_CHAR    0xFF
#define UART_BUFLEN        1024
#define UART_TX_CHUNK_SIZE 10

typedef union
{
    struct
    {
        uint64_t accel_x          :1;     //0
        uint64_t accel_y          :1;     //1
        uint64_t accel_z          :1;     //2
        uint64_t gyro_x           :1;     //3
        uint64_t gyro_y           :1;     //4
        uint64_t gyro_z           :1;     //5
        uint64_t magn_x           :1;     //6
        uint64_t magn_y           :1;     //7
        uint64_t magn_z           :1;     //8
        uint64_t accel_x_f        :1;     //9
        uint64_t accel_y_f        :1;     //10
        uint64_t accel_z_f        :1;     //11
        uint64_t gyro_x_f         :1;     //12
        uint64_t gyro_y_f         :1;     //13
        uint64_t gyro_z_f         :1;     //14
        uint64_t magn_x_f         :1;     //15
        uint64_t magn_y_f         :1;     //16
        uint64_t magn_z_f         :1;     //17
        uint64_t throttle_sgn     :1;     //18
        uint64_t roll_sgn         :1;     //19
        uint64_t pitch_sgn        :1;     //20
        uint64_t cmd_thr          :1;     //21
        uint64_t cmd_roll         :1;     //22
        uint64_t cmd_pitch        :1;     //23
        uint64_t body_roll        :1;     //24
        uint64_t body_pitch       :1;     //25
        uint64_t body_yaw         :1;     //26
        uint64_t roll_pid_err     :1;     //27
        uint64_t roll_pid_p       :1;     //28
        uint64_t roll_pid_i       :1;     //29
        uint64_t roll_pid_d       :1;     //30
        uint64_t roll_pid_u       :1;     //31
        uint64_t pitch_pid_err    :1;     //32
        uint64_t pitch_pid_p      :1;     //33
        uint64_t pitch_pid_i      :1;     //34
        uint64_t pitch_pid_d      :1;     //35
        uint64_t pitch_pid_u      :1;     //36
        uint64_t yaw_pid_err      :1;     //37
        uint64_t yaw_pid_p        :1;     //38
        uint64_t yaw_pid_i        :1;     //39
        uint64_t yaw_pid_d        :1;     //40
        uint64_t yaw_pid_u        :1;     //41
        uint64_t motor1           :1;     //42
        uint64_t motor2           :1;     //43
        uint64_t motor3           :1;     //44
        uint64_t motor4           :1;     //45
        uint64_t motors_armed     :1;     //46
        uint64_t cbit             :1;     //47
        uint64_t motor_params     :1;     //48
        uint64_t js_params        :1;     //49
        uint64_t pid_params       :1;     //50
        uint64_t ptf1_params      :1;     //51
        uint64_t imu_type         :1;     //52
        uint64_t i2c_read         :1;     //53
        uint64_t sw_ver           :1;     //54
        uint64_t imu_offset       :1;     //55
        uint64_t throttle_params  :1;     //56
        uint64_t maint_cmd_id     :7;     //57, 58, 59, 60, 61, 62, 63
    } Bits;
    
    uint8_t  Bytes[8];
    uint64_t All;
} MAINT_HEADER_T;


typedef struct
{
    uint8_t override_radio;
    uint16_t armed_signal;
    uint16_t roll_signal;
    uint16_t pitch_signal;
    uint16_t throttle_signal;
} __attribute__((packed)) REMOTE_CONTROL_TAG;


typedef struct
{
    MAINT_HEADER_T header;
    uint8_t payload[MAINT_PAYLOAD_SIZE];
}__attribute__((packed)) MAINT_MESSAGE_TAG;


enum class MAINT_STATUS
{
    WAIT_SYNC = 0,
    WAIT_HEADER_BYTE_0,
    WAIT_HEADER_BYTE_1,
    WAIT_HEADER_BYTE_2,
    WAIT_HEADER_BYTE_3,
    WAIT_HEADER_BYTE_4,
    WAIT_HEADER_BYTE_5,
    WAIT_HEADER_BYTE_6,
    WAIT_HEADER_BYTE_7,
    WAIT_PAYLOAD,
};

enum class MAINT_CMD_ID
{
    MAINT_CMD_NONE = 0,
    MAINT_CMD_SET_M1,
    MAINT_CMD_SET_M2,
    MAINT_CMD_SET_M3,
    MAINT_CMD_SET_M4,
    MAINT_CMD_SET_MALL,
    MAINT_CMD_CTRL_MOTORS,
    MAINT_CMD_FLASH_WRITE,
    MAINT_CMD_SET_M1_PARAMS,
    MAINT_CMD_SET_M2_PARAMS,
    MAINT_CMD_SET_M3_PARAMS,
    MAINT_CMD_SET_M4_PARAMS,
    MAINT_CMD_SET_JS_THROTTLE_ALPHA_BETA,
    MAINT_CMD_SET_JS_ROLL_ALPHA_BETA,
    MAINT_CMD_SET_JS_PITCH_ALPHA_BETA,
    MAINT_CMD_SET_ROLL_PID_PARAMS,
    MAINT_CMD_SET_PITCH_PID_PARAMS,
    MAINT_CMD_SET_YAW_PID_PARAMS,
    MAINT_CMD_SET_PTF1_ACC_PARAMS,
    MAINT_CMD_SET_PTF1_GYRO_PARAMS,
    MAINT_CMD_SET_PTF1_MAGN_PARAMS,
    MAINT_CMD_SET_IMU_TYPE,
    MAINT_CMD_I2C_READ,
    MAINT_CMD_I2C_WRITE,
    MAINT_CMD_SET_IMU_OFFSET,
    MAINT_CMD_SET_THROTTLE_PARAMS
};

enum class MAINT_MOTOR_PARAM
{
    FIRST = 0,
    ENABLED = FIRST,
    MIN_SIGNAL,
    MAX_SIGNAL,
    SIZE
};

enum class MAINT_JS_PARAM
{
    FIRST = 0,
    ALPHA = FIRST,
    BETA,
    SIZE
};

enum class MAINT_PID_PARAM
{
    FIRST = 0,
    PID_KP = FIRST,
    PID_KI,
    PID_KT,
    PID_SAT,
    PID_AD,
    PID_BD,
    SIZE
};

enum class MAINT_THROTTLE_PARAM
{
    FIRST = 0,
    DESCEND = FIRST,
    HOVERING,
    CLIMB,
    SIZE
};

enum class JOYSTICK_CHANNEL
{
    FIRST = 0,
    THROTTLE = FIRST,
    ROLL,
    PITCH,
    ARMED,
    SIZE
};

enum class MOTORS
{
    FIRST = 0,
    M1 = FIRST,
    M2,
    M3,
    M4,
    SIZE
};

enum class EULER_ANGLES
{
    FIRST = 0,
    ROLL = FIRST,
    PITCH,
    YAW,
    SIZE
};

enum class SENSOR_SOURCE
{
    FIRST = 0,
    ACCELEROMETER = FIRST,
    GYROSCOPE,
    MAGNETOMETER,
    SIZE
};

enum class EUCLIDEAN_AXES
{
    FIRST = 0,
    X = FIRST,
    Y,
    Z,
    SIZE
};

enum class IMU_TYPE : uint8_t
{
    FIRST = 0,
    LSM9DS1 = FIRST,
    MPU6050,
    BNO055,
    SIZE
};

typedef union 
{
    uint32_t ival;
    float    fval;
} maint_float_t;

extern uint32_t MAINT_MotorsParameters[int(MOTORS::SIZE)][int(MAINT_MOTOR_PARAM::SIZE)];
extern uint32_t MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::SIZE)][int(MAINT_JS_PARAM::SIZE)];
extern uint32_t MAINT_PidParameters[int(EULER_ANGLES::SIZE)][int(MAINT_PID_PARAM::SIZE)];
extern uint32_t MAINT_Ptf1Parameters[int(SENSOR_SOURCE::SIZE)][int(EUCLIDEAN_AXES::SIZE)];
extern IMU_TYPE MAINT_ImuType;
extern uint16_t MAINT_ThrottleParams[int(MAINT_THROTTLE_PARAM::SIZE)];

extern bool MAINT_FlashWriteRequested;

void MAINT_Init();
void MAINT_OnByteReceived(uint8_t byte_rx);
void MAINT_Handler();
bool MAINT_IsPresent();
bool MAINT_IsControllingMotors();
bool MAINT_IsOverridingRadio();
void MAINT_ReadOverridenSignals(uint16_t* armed, uint16_t* roll, uint16_t* pitch, uint16_t* throttle);

#endif //MAINT_H