#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>

#define SYNC_CHAR 0xFF

typedef union
{
    struct
    {
        uint64_t accel_x : 1;     //0
        uint64_t accel_y : 1;     //1
        uint64_t accel_z : 1;     //2
        uint64_t gyro_x : 1;     //3
        uint64_t gyro_y : 1;     //4
        uint64_t gyro_z : 1;     //5
        uint64_t magn_x : 1;     //6
        uint64_t magn_y : 1;     //7
        uint64_t magn_z : 1;     //8
        uint64_t accel_x_f : 1;     //9
        uint64_t accel_y_f : 1;     //10
        uint64_t accel_z_f : 1;     //11
        uint64_t gyro_x_f : 1;     //12
        uint64_t gyro_y_f : 1;     //13
        uint64_t gyro_z_f : 1;     //14
        uint64_t magn_x_f : 1;     //15
        uint64_t magn_y_f : 1;     //16
        uint64_t magn_z_f : 1;     //17
        uint64_t throttle_sgn : 1;     //18
        uint64_t roll_sgn : 1;     //19
        uint64_t pitch_sgn : 1;     //20
        uint64_t cmd_thr : 1;     //21
        uint64_t cmd_roll : 1;     //22
        uint64_t cmd_pitch : 1;     //23
        uint64_t body_roll : 1;     //24
        uint64_t body_pitch : 1;     //25
        uint64_t body_yaw : 1;     //26
        uint64_t roll_pid_err : 1;     //27
        uint64_t roll_pid_p : 1;     //28
        uint64_t roll_pid_i : 1;     //29
        uint64_t roll_pid_d : 1;     //30
        uint64_t roll_pid_u : 1;     //31
        uint64_t pitch_pid_err : 1;     //32
        uint64_t pitch_pid_p : 1;     //33
        uint64_t pitch_pid_i : 1;     //34
        uint64_t pitch_pid_d : 1;     //35
        uint64_t pitch_pid_u : 1;     //36
        uint64_t yaw_pid_err : 1;     //37
        uint64_t yaw_pid_p : 1;     //38
        uint64_t yaw_pid_i : 1;     //39
        uint64_t yaw_pid_d : 1;     //40
        uint64_t yaw_pid_u : 1;     //41
        uint64_t motor1 : 1;     //42
        uint64_t motor2 : 1;     //43
        uint64_t motor3 : 1;     //44
        uint64_t motor4 : 1;     //45
        uint64_t motors_armed : 1;     //46
        uint64_t cbit : 1;     //47
        uint64_t motor_params : 1;     //48
        uint64_t js_params : 1;     //49
        uint64_t pid_params : 1;     //50
        uint64_t ptf1_params : 1;     //51
        uint64_t imu_type : 1; // 52
        uint64_t i2c_read : 1; // 53
        uint64_t sw_ver : 1; // 54
        uint64_t imu_offset : 1; //55
        uint64_t maint_cmd_id : 8;    //56, 57, 58, 59, 60, 61, 62, 63
    } Bits;

    uint8_t  Bytes[8];
    uint64_t All;
} MaintenanceProtocolHdr;


enum class MaintenanceCommand
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
    MAINT_CMD_SET_OVERRIDE_RADIO,
    MAINT_CMD_SET_ROLL_PITCH_THROTTLE_SIGNAL,
    MAINT_CMD_SET_ARMED_SIGNAL
};

#endif
