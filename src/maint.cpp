#include "maint.h"
#include "motors.h"
#include "attitude.h"
#include "joystick.h"
#include "cbit.h"
#include "user.h"
#include <pico/time.h>
#include <string.h>
#include <stdio.h>

uint32_t motor_parameters[4][int(MAINT_MOTOR_PARAM::SIZE)];

static MAINT_STATUS status;
static uint8_t rx_buf[sizeof(MAINT_MESSAGE_TAG)];
static MAINT_MESSAGE_TAG rx_message;
static MAINT_MESSAGE_TAG tx_message;
static uint32_t expected_bytes;
static uint32_t rx_payload_idx;
static bool shall_tx;
static bool shall_set;
static uint64_t last_msg_us;
static bool controlling_motors;


static void put_packet(uint8_t* buf, uint32_t len)
{
    putchar(MAINT_SYNC_CHAR);
    for (uint32_t i = 0; i < len; i++)
    {
        putchar(buf[i]);
    }
}


static uint8_t checksum(uint8_t* buf, uint32_t size)
{
    uint8_t cks = 0;
    for (uint32_t i = 0; i < size; i++)
    {
        cks ^= buf[i];
    }
    return cks;
}


static void data_ingest(uint8_t rx_cks, uint32_t data_len)
{
    uint8_t local_cks = checksum(&rx_buf[0], data_len - 1);
    if (local_cks == rx_cks)
    {
        last_msg_us = time_us_64();

        memcpy(&rx_message, &rx_buf[0], data_len);
        shall_tx = true;
        shall_set = (static_cast<MAINT_CMD_ID>(rx_message.header.Bits.maint_cmd_id) != MAINT_CMD_ID::MAINT_CMD_NONE);
    }
}


static uint32_t calc_exp_bytes(MAINT_HEADER_T* header)
{
    uint64_t cmd_id = header->Bits.maint_cmd_id;
    
    switch (static_cast<MAINT_CMD_ID>(cmd_id))
    {
        case MAINT_CMD_ID::MAINT_CMD_NONE:
            return 1; /** Checksum only **/
        case MAINT_CMD_ID::MAINT_CMD_SET_M1: 
        case MAINT_CMD_ID::MAINT_CMD_SET_M2:
        case MAINT_CMD_ID::MAINT_CMD_SET_M3:
        case MAINT_CMD_ID::MAINT_CMD_SET_M4:
        case MAINT_CMD_ID::MAINT_CMD_SET_MALL:
        case MAINT_CMD_ID::MAINT_CMD_CTRL_MOTORS:
            return 5; /** 4 data bytes + checksum **/
        case MAINT_CMD_ID::MAINT_CMD_SET_M1_PARAMS:
        case MAINT_CMD_ID::MAINT_CMD_SET_M2_PARAMS:
        case MAINT_CMD_ID::MAINT_CMD_SET_M3_PARAMS:
        case MAINT_CMD_ID::MAINT_CMD_SET_M4_PARAMS:
            return 13; /** 3 * 4 = 12 data bytes + checksum **/
    }

    return 0;
}


static void update_fsm(uint8_t byte_rx)
{
    switch (status)
    {
        case MAINT_STATUS::WAIT_SYNC:
        {
            if (byte_rx == MAINT_SYNC_CHAR)
            {
                status = MAINT_STATUS::WAIT_HEADER_BYTE_0;
            }
            break;
        }
        case MAINT_STATUS::WAIT_HEADER_BYTE_0:
        {
            rx_buf[0] = byte_rx;
            status = MAINT_STATUS::WAIT_HEADER_BYTE_1;
            break;
        }
        case MAINT_STATUS::WAIT_HEADER_BYTE_1:
        {
            rx_buf[1] = byte_rx;
            status = MAINT_STATUS::WAIT_HEADER_BYTE_2;
            break;
        }
        case MAINT_STATUS::WAIT_HEADER_BYTE_2:
        {
            rx_buf[2] = byte_rx;
            status = MAINT_STATUS::WAIT_HEADER_BYTE_3;
            break;
        }
        case MAINT_STATUS::WAIT_HEADER_BYTE_3:
        {
            rx_buf[3] = byte_rx;
            status = MAINT_STATUS::WAIT_HEADER_BYTE_4;
            break;
        }
        case MAINT_STATUS::WAIT_HEADER_BYTE_4:
        {
            rx_buf[4] = byte_rx;
            status = MAINT_STATUS::WAIT_HEADER_BYTE_5;
            break;
        }
        case MAINT_STATUS::WAIT_HEADER_BYTE_5:
        {
            rx_buf[5] = byte_rx;
            status = MAINT_STATUS::WAIT_HEADER_BYTE_6;
            break;
        }
        case MAINT_STATUS::WAIT_HEADER_BYTE_6:
        {
            rx_buf[6] = byte_rx;
            status = MAINT_STATUS::WAIT_HEADER_BYTE_7;
            break;
        }
        case MAINT_STATUS::WAIT_HEADER_BYTE_7:
        {
            rx_buf[7] = byte_rx;
            status = MAINT_STATUS::WAIT_PAYLOAD;
            expected_bytes = calc_exp_bytes(reinterpret_cast<MAINT_HEADER_T*>(&rx_buf[0]));
            rx_payload_idx = 0;
            break;
        }
        case MAINT_STATUS::WAIT_PAYLOAD:
        {
            rx_buf[8 + rx_payload_idx] = byte_rx;
            rx_payload_idx += 1;

            if (rx_payload_idx == expected_bytes)
            {
                data_ingest(byte_rx, 8 + rx_payload_idx);
                status = MAINT_STATUS::WAIT_SYNC;
            }
            break;
        }
    }
}


void MAINT_Init()
{
    last_msg_us = 0;

    memset(rx_buf, sizeof(MAINT_MESSAGE_TAG), 0x00);
    memset(&rx_message, sizeof(MAINT_MESSAGE_TAG), 0x00);
    memset(&tx_message, sizeof(MAINT_MESSAGE_TAG), 0x00);

    for (int i = 0; i < 4; i++)
    {
        motor_parameters[i][int(MAINT_MOTOR_PARAM::ENABLED)] = 1;
        motor_parameters[i][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)] = MOTOR_MIN_SIGNAL;
        motor_parameters[i][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)] = MOTOR_MAX_SIGNAL;
    }

    expected_bytes = 0;
    rx_payload_idx = 0;
    shall_tx = false;
    shall_set = false;

    status = MAINT_STATUS::WAIT_SYNC;
}


void MAINT_OnByteReceived(uint8_t byte_rx)
{
    update_fsm(byte_rx);
}


bool MAINT_IsPresent()
{
    return (time_us_64() - last_msg_us) < MAINT_TIMEOUT_S * SECONDS_TO_MICROSECONDS;
}


void MAINT_Handler()
{
    int byteIn = getchar();
    MAINT_OnByteReceived((uint8_t)byteIn & 0xFF);
    
    if (shall_set)
    {
        switch (static_cast<MAINT_CMD_ID>(rx_message.header.Bits.maint_cmd_id))
        {
            case MAINT_CMD_ID::MAINT_CMD_NONE:
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_M1:
                if (controlling_motors) motor1.writeMicroseconds(*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_M2:
                if (controlling_motors) motor2.writeMicroseconds(*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_M3:
                if (controlling_motors) motor3.writeMicroseconds(*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_M4:
                if (controlling_motors) motor4.writeMicroseconds(*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_MALL:
                if (controlling_motors) 
                {
                    motor1.writeMicroseconds(*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                    motor2.writeMicroseconds(*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                    motor3.writeMicroseconds(*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                    motor4.writeMicroseconds(*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                }
                break;
            case MAINT_CMD_ID::MAINT_CMD_CTRL_MOTORS:
                controlling_motors = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]) == 1);
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_M1_PARAMS:
                motor_parameters[0][int(MAINT_MOTOR_PARAM::ENABLED)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                motor_parameters[0][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                motor_parameters[0][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[8]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_M2_PARAMS:
                motor_parameters[1][int(MAINT_MOTOR_PARAM::ENABLED)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                motor_parameters[1][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                motor_parameters[1][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[8]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_M3_PARAMS:
                motor_parameters[2][int(MAINT_MOTOR_PARAM::ENABLED)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                motor_parameters[2][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                motor_parameters[2][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[8]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_M4_PARAMS:
                motor_parameters[3][int(MAINT_MOTOR_PARAM::ENABLED)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                motor_parameters[3][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                motor_parameters[3][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[8]));
                break;

        }

        shall_set = false;
    }

    if (shall_tx)
    {
        tx_message.header.All = rx_message.header.All;
        
        uint32_t tx_payload_idx = 0;
        if (tx_message.header.Bits.accel_x)
        {
            float fdata = ax_flt_tag.raw_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.accel_y)
        {
            float fdata = ay_flt_tag.raw_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.accel_z)
        {
            float fdata = az_flt_tag.raw_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.gyro_x)
        {
            float fdata = gx_flt_tag.raw_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.gyro_y)
        {
            float fdata = gy_flt_tag.raw_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.gyro_z)
        {
            float fdata = gz_flt_tag.raw_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.magn_x)
        {
            float fdata = mx_flt_tag.raw_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.magn_y)
        {
            float fdata = my_flt_tag.raw_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.magn_z)
        {
            float fdata = mz_flt_tag.raw_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.accel_x_f)
        {
            float fdata = ax_flt_tag.filt_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.accel_y_f)
        {
            float fdata = ay_flt_tag.filt_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.accel_z_f)
        {
            float fdata = az_flt_tag.filt_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.gyro_x_f)
        {
            float fdata = gx_flt_tag.filt_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.gyro_y_f)
        {
            float fdata = gy_flt_tag.filt_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.gyro_z_f)
        {
            float fdata = gz_flt_tag.filt_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.magn_x_f)
        {
            float fdata = mx_flt_tag.filt_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.magn_y_f)
        {
            float fdata = my_flt_tag.filt_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.magn_z_f)
        {
            float fdata = mz_flt_tag.filt_k;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.throttle_sgn)
        {
            uint32_t idata = throttle_signal.pulseIn();
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.roll_sgn)
        {
            uint32_t idata = roll_signal.pulseIn();
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.pitch_sgn)
        {
            uint32_t idata = pitch_signal.pulseIn();
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.cmd_thr)
        {
            float fdata = JOYSTICK_Throttle;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.cmd_roll)
        {
            float fdata = JOYSTICK_Roll;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.cmd_pitch)
        {
            float fdata = JOYSTICK_Pitch;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.body_roll)
        {
            float fdata = ATTITUDE_Roll;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.body_pitch)
        {
            float fdata = ATTITUDE_Pitch;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.body_yaw)
        {
            float fdata = ATTITUDE_Yaw;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.roll_pid_err)
        {
            float fdata = pid_roll.error;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.roll_pid_p)
        {
            float fdata = pid_roll.P;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.roll_pid_i)
        {
            float fdata = pid_roll.I;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.roll_pid_d)
        {
            float fdata = pid_roll.D;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.roll_pid_u)
        {
            float fdata = pid_roll.u;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.pitch_pid_err)
        {
            float fdata = pid_pitch.error;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.pitch_pid_p)
        {
            float fdata = pid_pitch.P;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.pitch_pid_i)
        {
            float fdata = pid_pitch.I;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.pitch_pid_d)
        {
            float fdata = pid_pitch.D;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.pitch_pid_u)
        {
            float fdata = pid_pitch.u;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.yaw_pid_err)
        {
            float fdata = pid_yaw.error;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.yaw_pid_p)
        {
            float fdata = pid_yaw.P;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.yaw_pid_i)
        {
            float fdata = pid_yaw.I;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.yaw_pid_d)
        {
            float fdata = pid_yaw.D;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.yaw_pid_u)
        {
            float fdata = pid_yaw.u;
            uint32_t idata = *(reinterpret_cast<uint32_t*>(&fdata));
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.motor1)
        {
            uint32_t idata = motor1.currentSignal();
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.motor2)
        {
            uint32_t idata = motor2.currentSignal();
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.motor3)
        {
            uint32_t idata = motor3.currentSignal();
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.motor4)
        {
            uint32_t idata = motor4.currentSignal();
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.motors_armed)
        {
            uint32_t idata = JOYSTICK_MotorsArmed;
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.cbit)
        {
            uint32_t idata = cbit_status.Dword;
            memcpy(&tx_message.payload[tx_payload_idx], &idata, sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }

        tx_message.payload[tx_payload_idx] = checksum(reinterpret_cast<uint8_t*>(&tx_message), sizeof(MAINT_HEADER_T) + tx_payload_idx);
        put_packet(reinterpret_cast<uint8_t*>(&tx_message), sizeof(MAINT_HEADER_T) + tx_payload_idx + 1);

        shall_tx = false;
    }

}


bool MAINT_IsControllingMotors()
{
    return controlling_motors;
}