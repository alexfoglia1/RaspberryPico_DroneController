#ifndef MAINT_H
#define MAINT_H

#include <stdint.h>

#define MAINT_PAYLOAD_SIZE 512
#define MAINT_PAYLOAD_SIZE 512
#define MAINT_SYNC_CHAR    0xFF

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
        uint64_t maint_cmd_id     :16;    //48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63
    } Bits;
    
    uint8_t  Bytes[8];
    uint64_t All;
} MAINT_HEADER_T;


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
    MAINT_CMD_SET_M4_PARAMS
};

enum class MAINT_MOTOR_PARAM
{
    FIRST = 0,
    ENABLED = FIRST,
    MIN_SIGNAL,
    MAX_SIGNAL,
    SIZE
};

extern uint32_t MAINT_MotorsParameters[4][int(MAINT_MOTOR_PARAM::SIZE)];
extern bool MAINT_FlashWriteRequested;

void MAINT_Init();
void MAINT_OnByteReceived(uint8_t byte_rx);
void MAINT_Handler();
bool MAINT_IsPresent();
bool MAINT_IsControllingMotors();

#endif //MAINT_H