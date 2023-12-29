#include "maint.h"
#include "motors.h"
#include "attitude.h"
#include "timer.h"
#include "joystick.h"
#include "cbit.h"
#include "user.h"
#include "i2c_utils.h"

#include <pico/time.h>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <string.h>
#include <stdio.h>


// We're going to erase and reprogram a region 256k from the start of flash.
// Once done, we can access this at XIP_BASE + 256k.
#define FLASH_TARGET_OFFSET (256 * 1024)
#define FLASH_PARAMS_OFFSET 0
#define FLASH_PARAMS_SIZE   192 // 4 blocks of 3 4-bytes length uint32_t + 
                                // 4 blocks of 2 4-bytes length uint32_t +
                                // 3 blocks of 6 4-bytes length uint32_t +
                                // 3 blocks of 3 4 bytes length uint32_t +
                                // 1 block of  1 4 bytes length uint32_t = 192
#define FLASH_MOTORS_PARAMS_SIZE   48
#define FLASH_JOYSTICK_PARAMS_SIZE 32
#define FLASH_PID_PARAMS_SIZE      72
#define FLASH_PTF1_PARAMS_SIZE     36
#define FLASH_IMU_TYPE_SIZE         4

const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

uint32_t MAINT_MotorsParameters[int(MOTORS::SIZE)][int(MAINT_MOTOR_PARAM::SIZE)];
uint32_t MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::SIZE)][int(MAINT_JS_PARAM::SIZE)];
uint32_t MAINT_PidParameters[int(EULER_ANGLES::SIZE)][int(MAINT_PID_PARAM::SIZE)];
uint32_t MAINT_Ptf1Parameters[int(SENSOR_SOURCE::SIZE)][int(EUCLIDEAN_AXES::SIZE)];
IMU_TYPE MAINT_ImuType;
uint8_t  MAINT_I2CRead;

bool MAINT_FlashWriteRequested;

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
static uint8_t uart_tx_buf[UART_BUFLEN];
static uint8_t uart_ll_tx_buf;


static void put_packet(uint8_t* buf, uint32_t len, MaintClient sender)
{
    if (sender == MaintClient::USB)
    {
        putchar(MAINT_SYNC_CHAR);
        for (uint32_t i = 0; i < len; i++)
        {
            putchar(buf[i]);
        }
    }
    else
    {
        uart_putc_raw(uart0, MAINT_SYNC_CHAR);
        for (uint32_t i = 0; i < len; i++)
        {
            uart_putc_raw(uart0, buf[i]);
        }        
#if 0        
        if (uart_ll_tx_buf + len + 1 < UART_BUFLEN)
        {
            uart_tx_buf[uart_ll_tx_buf] = MAINT_SYNC_CHAR;
            memcpy(&uart_tx_buf[uart_ll_tx_buf + 1], buf, len);
            
            uart_ll_tx_buf += (1 + len);
        }
#endif        
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
        case MAINT_CMD_ID::MAINT_CMD_FLASH_WRITE:
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
        case MAINT_CMD_ID::MAINT_CMD_SET_JS_THROTTLE_ALPHA_BETA:
        case MAINT_CMD_ID::MAINT_CMD_SET_JS_ROLL_ALPHA_BETA:
        case MAINT_CMD_ID::MAINT_CMD_SET_JS_PITCH_ALPHA_BETA:
            return 9; /** 2 * 4 = 8 data bytes + checksum **/
        case MAINT_CMD_ID::MAINT_CMD_SET_ROLL_PID_PARAMS:
        case MAINT_CMD_ID::MAINT_CMD_SET_PITCH_PID_PARAMS:
        case MAINT_CMD_ID::MAINT_CMD_SET_YAW_PID_PARAMS:
            return 25; /** 6 * 4 = 24 data bytes + checksum*/
        case MAINT_CMD_ID::MAINT_CMD_SET_PTF1_ACC_PARAMS:
        case MAINT_CMD_ID::MAINT_CMD_SET_PTF1_GYRO_PARAMS:
        case MAINT_CMD_ID::MAINT_CMD_SET_PTF1_MAGN_PARAMS:
            return 13; /** 3 * 4 = 12 bytes + checksum **/
        case MAINT_CMD_ID::MAINT_CMD_SET_IMU_TYPE:
            return 5;  /** 1 * 4 = 4  bytes + checksum **/
        case MAINT_CMD_ID::MAINT_CMD_I2C_READ:
            return 13;  /** 3 * 4 = 12  bytes + checksum **/
        case MAINT_CMD_ID::MAINT_CMD_I2C_WRITE:
            return 17;  /** 4 * 4 = 16  bytes + checksum */
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


static void flash_write_params()
{
    uint32_t ints = save_and_disable_interrupts();

    uint8_t eeprom_img[FLASH_PAGE_SIZE];
    memset(&eeprom_img, 0x00, FLASH_PAGE_SIZE);

    memcpy(&eeprom_img[0], reinterpret_cast<uint8_t*>(MAINT_MotorsParameters), FLASH_MOTORS_PARAMS_SIZE);
    memcpy(&eeprom_img[FLASH_MOTORS_PARAMS_SIZE], reinterpret_cast<uint8_t*>(MAINT_JoystickParameters), FLASH_JOYSTICK_PARAMS_SIZE);
    memcpy(&eeprom_img[FLASH_MOTORS_PARAMS_SIZE + FLASH_JOYSTICK_PARAMS_SIZE], reinterpret_cast<uint8_t*>(MAINT_PidParameters), FLASH_PID_PARAMS_SIZE);
    memcpy(&eeprom_img[FLASH_MOTORS_PARAMS_SIZE + FLASH_JOYSTICK_PARAMS_SIZE + FLASH_PID_PARAMS_SIZE], reinterpret_cast<uint8_t*>(MAINT_Ptf1Parameters), FLASH_PTF1_PARAMS_SIZE);
    memcpy(&eeprom_img[FLASH_MOTORS_PARAMS_SIZE + FLASH_JOYSTICK_PARAMS_SIZE + FLASH_PID_PARAMS_SIZE + FLASH_PTF1_PARAMS_SIZE], reinterpret_cast<uint8_t*>(&MAINT_ImuType), FLASH_IMU_TYPE_SIZE);
    
    eeprom_img[FLASH_PARAMS_SIZE] = checksum(reinterpret_cast<uint8_t*>(eeprom_img), FLASH_PARAMS_SIZE);

    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, eeprom_img, FLASH_PAGE_SIZE);

    restore_interrupts(ints);
}


void MAINT_Init()
{
    MAINT_FlashWriteRequested = false;
    last_msg_us = 0;
    MAINT_I2CRead = 0;

    memset(rx_buf, 0x00, sizeof(MAINT_MESSAGE_TAG));
    memset(&rx_message, 0x00, sizeof(MAINT_MESSAGE_TAG));
    memset(&tx_message, 0x00, sizeof(MAINT_MESSAGE_TAG));
    memset(&uart_tx_buf, 0x00, sizeof(UART_BUFLEN));
    uart_ll_tx_buf = 0;

    /** Load actual params **/
    uint32_t eeprom_offset = 0;

    for (int i = int(MOTORS::FIRST); i < int(MOTORS::SIZE); i++)
    {
        MAINT_MotorsParameters[i][int(MAINT_MOTOR_PARAM::ENABLED)]    = *reinterpret_cast<const uint32_t*>(&flash_target_contents[eeprom_offset + 0 * sizeof(uint32_t)]);
        MAINT_MotorsParameters[i][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)] = *reinterpret_cast<const uint32_t*>(&flash_target_contents[eeprom_offset + 1 * sizeof(uint32_t)]);
        MAINT_MotorsParameters[i][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)] = *reinterpret_cast<const uint32_t*>(&flash_target_contents[eeprom_offset + 2 * sizeof(uint32_t)]);
        
        eeprom_offset += (int(MAINT_MOTOR_PARAM::SIZE) * sizeof(uint32_t));
    }
    
    for (int i = int(JOYSTICK_CHANNEL::FIRST); i < int(JOYSTICK_CHANNEL::SIZE); i++)
    {
        MAINT_JoystickParameters[i][int(MAINT_JS_PARAM::ALPHA)] = *reinterpret_cast<const uint32_t*>(&flash_target_contents[eeprom_offset + 0 * sizeof(uint32_t)]);
        MAINT_JoystickParameters[i][int(MAINT_JS_PARAM::BETA)]  = *reinterpret_cast<const uint32_t*>(&flash_target_contents[eeprom_offset + 1 * sizeof(uint32_t)]);
        
        eeprom_offset += (int(MAINT_JS_PARAM::SIZE) * sizeof(uint32_t));
    }

    for (int i = int(EULER_ANGLES::FIRST); i < int(EULER_ANGLES::SIZE); i++)
    {
        MAINT_PidParameters[i][int(MAINT_PID_PARAM::PID_KP)]  = *reinterpret_cast<const uint32_t*>(&flash_target_contents[eeprom_offset + 0 * sizeof(uint32_t)]);
        MAINT_PidParameters[i][int(MAINT_PID_PARAM::PID_KI)]  = *reinterpret_cast<const uint32_t*>(&flash_target_contents[eeprom_offset + 1 * sizeof(uint32_t)]);
        MAINT_PidParameters[i][int(MAINT_PID_PARAM::PID_KT)]  = *reinterpret_cast<const uint32_t*>(&flash_target_contents[eeprom_offset + 2 * sizeof(uint32_t)]);
        MAINT_PidParameters[i][int(MAINT_PID_PARAM::PID_SAT)] = *reinterpret_cast<const uint32_t*>(&flash_target_contents[eeprom_offset + 3 * sizeof(uint32_t)]);
        MAINT_PidParameters[i][int(MAINT_PID_PARAM::PID_AD)]  = *reinterpret_cast<const uint32_t*>(&flash_target_contents[eeprom_offset + 4 * sizeof(uint32_t)]);
        MAINT_PidParameters[i][int(MAINT_PID_PARAM::PID_BD)]  = *reinterpret_cast<const uint32_t*>(&flash_target_contents[eeprom_offset + 5 * sizeof(uint32_t)]);
        
        eeprom_offset += (int(MAINT_PID_PARAM::SIZE) * sizeof(uint32_t));
    }

    for (int i = int(SENSOR_SOURCE::FIRST); i < int(SENSOR_SOURCE::SIZE); i++)
    {
        MAINT_Ptf1Parameters[i][int(EUCLIDEAN_AXES::X)] = *reinterpret_cast<const uint32_t*>(&flash_target_contents[eeprom_offset + 0 * sizeof(uint32_t)]);
        MAINT_Ptf1Parameters[i][int(EUCLIDEAN_AXES::Y)] = *reinterpret_cast<const uint32_t*>(&flash_target_contents[eeprom_offset + 1 * sizeof(uint32_t)]);
        MAINT_Ptf1Parameters[i][int(EUCLIDEAN_AXES::Z)] = *reinterpret_cast<const uint32_t*>(&flash_target_contents[eeprom_offset + 2 * sizeof(uint32_t)]);
        
        eeprom_offset += (int(EUCLIDEAN_AXES::SIZE) * sizeof(uint32_t));
    }

    MAINT_ImuType = IMU_TYPE(*reinterpret_cast<const uint32_t*>(&flash_target_contents[eeprom_offset]));
    eeprom_offset += sizeof(uint32_t);

    CBIT_TAG fail_code;
    fail_code.Dword = 0;
    fail_code.Bits.flash_error = 1;

    /** Check actual params **/
    const uint8_t eeprom_cks = *reinterpret_cast<const uint8_t*>(&flash_target_contents[FLASH_PARAMS_SIZE]);

    uint8_t eeprom_img[FLASH_PAGE_SIZE];
    memset(&eeprom_img, 0x00, FLASH_PAGE_SIZE);

    memcpy(&eeprom_img[0], reinterpret_cast<uint8_t*>(MAINT_MotorsParameters), FLASH_MOTORS_PARAMS_SIZE);
    memcpy(&eeprom_img[FLASH_MOTORS_PARAMS_SIZE], reinterpret_cast<uint8_t*>(MAINT_JoystickParameters), FLASH_JOYSTICK_PARAMS_SIZE);
    memcpy(&eeprom_img[FLASH_MOTORS_PARAMS_SIZE + FLASH_JOYSTICK_PARAMS_SIZE], reinterpret_cast<uint8_t*>(MAINT_PidParameters), FLASH_PID_PARAMS_SIZE);
    memcpy(&eeprom_img[FLASH_MOTORS_PARAMS_SIZE + FLASH_JOYSTICK_PARAMS_SIZE + FLASH_PID_PARAMS_SIZE], reinterpret_cast<uint8_t*>(MAINT_Ptf1Parameters), FLASH_PTF1_PARAMS_SIZE);
    memcpy(&eeprom_img[FLASH_MOTORS_PARAMS_SIZE + FLASH_JOYSTICK_PARAMS_SIZE + FLASH_PID_PARAMS_SIZE + FLASH_PTF1_PARAMS_SIZE], reinterpret_cast<uint8_t*>(&MAINT_ImuType), FLASH_IMU_TYPE_SIZE);
    
    uint8_t current_cks = checksum(reinterpret_cast<uint8_t*>(eeprom_img), FLASH_PARAMS_SIZE);
    if (eeprom_cks != current_cks)
    {
        /** Set fail code **/
        CBIT_Set_fail_code(fail_code.Dword, true);
        for (int i = 0; i < 8; i++)
        {
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(125);
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(125);
        }

        /** Flash not programmed, loading default **/
        for (int i = int(MOTORS::FIRST); i < int(MOTORS::SIZE); i++)
        {
            MAINT_MotorsParameters[i][int(MAINT_MOTOR_PARAM::ENABLED)]    = 1;
            MAINT_MotorsParameters[i][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)] = MOTOR_MIN_SIGNAL;
            MAINT_MotorsParameters[i][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)] = MOTOR_MAX_SIGNAL;
        }
        
        for (int i = int(JOYSTICK_CHANNEL::FIRST); i < int(JOYSTICK_CHANNEL::SIZE); i++)
        {
            maint_float_t alpha, beta;

            alpha.fval = 0.0f;
            beta.fval = 1.0f;

            MAINT_JoystickParameters[i][int(MAINT_JS_PARAM::ALPHA)] = alpha.ival;
            MAINT_JoystickParameters[i][int(MAINT_JS_PARAM::BETA)]  = beta.ival;
        }

        for (int i = int(EULER_ANGLES::FIRST); i < int(EULER_ANGLES::SIZE); i++)
        {
            maint_float_t kp, ki, kt, sat, ad, bd;
            kp.fval = 1.0f;
            ki.fval = 0.0f;
            kt.fval = 0.0f;
            sat.fval = 50.0f;
            ad.fval = 0.0f;
            bd.fval = 0.0f;

            MAINT_PidParameters[i][int(MAINT_PID_PARAM::PID_KP)]  = kp.ival;
            MAINT_PidParameters[i][int(MAINT_PID_PARAM::PID_KI)]  = ki.ival;
            MAINT_PidParameters[i][int(MAINT_PID_PARAM::PID_KT)]  = kt.ival;
            MAINT_PidParameters[i][int(MAINT_PID_PARAM::PID_SAT)] = sat.ival;
            MAINT_PidParameters[i][int(MAINT_PID_PARAM::PID_AD)]  = ad.ival;
            MAINT_PidParameters[i][int(MAINT_PID_PARAM::PID_BD)]  = bd.ival;
        }

        for (int i = int(SENSOR_SOURCE::FIRST); i < int(SENSOR_SOURCE::SIZE); i++)
        {
            maint_float_t t_ptf1_s;
            t_ptf1_s.fval = 0.1f;

            MAINT_Ptf1Parameters[i][int(EUCLIDEAN_AXES::X)] = t_ptf1_s.ival;
            MAINT_Ptf1Parameters[i][int(EUCLIDEAN_AXES::Y)] = t_ptf1_s.ival;
            MAINT_Ptf1Parameters[i][int(EUCLIDEAN_AXES::Z)] = t_ptf1_s.ival;
        }

        MAINT_ImuType = IMU_TYPE::MPU6050;

        /** Storing default to flash **/
        flash_write_params();
    }
    else
    {
        CBIT_Set_fail_code(fail_code.Dword, false);
    }


    expected_bytes = 0;
    rx_payload_idx = 0;
    shall_tx = false;
    shall_set = false;

    status = MAINT_STATUS::WAIT_SYNC;
}


void MAINT_OnByteReceived(uint8_t byte_rx, MaintClient sender)
{
    update_fsm(byte_rx);

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
                MAINT_MotorsParameters[int(MOTORS::M1)][int(MAINT_MOTOR_PARAM::ENABLED)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                MAINT_MotorsParameters[int(MOTORS::M1)][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                MAINT_MotorsParameters[int(MOTORS::M1)][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[8]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_M2_PARAMS:
                MAINT_MotorsParameters[int(MOTORS::M2)][int(MAINT_MOTOR_PARAM::ENABLED)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                MAINT_MotorsParameters[int(MOTORS::M2)][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                MAINT_MotorsParameters[int(MOTORS::M2)][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[8]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_M3_PARAMS:
                MAINT_MotorsParameters[int(MOTORS::M3)][int(MAINT_MOTOR_PARAM::ENABLED)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                MAINT_MotorsParameters[int(MOTORS::M3)][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                MAINT_MotorsParameters[int(MOTORS::M3)][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[8]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_M4_PARAMS:
                MAINT_MotorsParameters[int(MOTORS::M4)][int(MAINT_MOTOR_PARAM::ENABLED)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                MAINT_MotorsParameters[int(MOTORS::M4)][int(MAINT_MOTOR_PARAM::MIN_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                MAINT_MotorsParameters[int(MOTORS::M4)][int(MAINT_MOTOR_PARAM::MAX_SIGNAL)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[8]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_FLASH_WRITE:
                MAINT_FlashWriteRequested = true;
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_JS_THROTTLE_ALPHA_BETA:
                MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::THROTTLE)][int(MAINT_JS_PARAM::ALPHA)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::THROTTLE)][int(MAINT_JS_PARAM::BETA)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_JS_ROLL_ALPHA_BETA:
                MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::ROLL)][int(MAINT_JS_PARAM::ALPHA)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::ROLL)][int(MAINT_JS_PARAM::BETA)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_JS_PITCH_ALPHA_BETA:
                MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::PITCH)][int(MAINT_JS_PARAM::ALPHA)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::PITCH)][int(MAINT_JS_PARAM::BETA)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_ROLL_PID_PARAMS:
                MAINT_PidParameters[int(EULER_ANGLES::ROLL)][int(MAINT_PID_PARAM::PID_KP)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                MAINT_PidParameters[int(EULER_ANGLES::ROLL)][int(MAINT_PID_PARAM::PID_KI)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                MAINT_PidParameters[int(EULER_ANGLES::ROLL)][int(MAINT_PID_PARAM::PID_KT)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[8]));
                MAINT_PidParameters[int(EULER_ANGLES::ROLL)][int(MAINT_PID_PARAM::PID_SAT)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[12]));       
                MAINT_PidParameters[int(EULER_ANGLES::ROLL)][int(MAINT_PID_PARAM::PID_AD)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[16]));   
                MAINT_PidParameters[int(EULER_ANGLES::ROLL)][int(MAINT_PID_PARAM::PID_BD)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[20]));       
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_PITCH_PID_PARAMS:
                MAINT_PidParameters[int(EULER_ANGLES::PITCH)][int(MAINT_PID_PARAM::PID_KP)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                MAINT_PidParameters[int(EULER_ANGLES::PITCH)][int(MAINT_PID_PARAM::PID_KI)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                MAINT_PidParameters[int(EULER_ANGLES::PITCH)][int(MAINT_PID_PARAM::PID_KT)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[8]));
                MAINT_PidParameters[int(EULER_ANGLES::PITCH)][int(MAINT_PID_PARAM::PID_SAT)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[12]));       
                MAINT_PidParameters[int(EULER_ANGLES::PITCH)][int(MAINT_PID_PARAM::PID_AD)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[16]));   
                MAINT_PidParameters[int(EULER_ANGLES::PITCH)][int(MAINT_PID_PARAM::PID_BD)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[20]));  
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_YAW_PID_PARAMS:
                MAINT_PidParameters[int(EULER_ANGLES::YAW)][int(MAINT_PID_PARAM::PID_KP)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                MAINT_PidParameters[int(EULER_ANGLES::YAW)][int(MAINT_PID_PARAM::PID_KI)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                MAINT_PidParameters[int(EULER_ANGLES::YAW)][int(MAINT_PID_PARAM::PID_KT)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[8]));
                MAINT_PidParameters[int(EULER_ANGLES::YAW)][int(MAINT_PID_PARAM::PID_SAT)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[12]));       
                MAINT_PidParameters[int(EULER_ANGLES::YAW)][int(MAINT_PID_PARAM::PID_AD)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[16]));   
                MAINT_PidParameters[int(EULER_ANGLES::YAW)][int(MAINT_PID_PARAM::PID_BD)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[20]));  
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_PTF1_ACC_PARAMS:
                MAINT_Ptf1Parameters[int(SENSOR_SOURCE::ACCELEROMETER)][int(EUCLIDEAN_AXES::X)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                MAINT_Ptf1Parameters[int(SENSOR_SOURCE::ACCELEROMETER)][int(EUCLIDEAN_AXES::Y)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                MAINT_Ptf1Parameters[int(SENSOR_SOURCE::ACCELEROMETER)][int(EUCLIDEAN_AXES::Z)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[8]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_PTF1_GYRO_PARAMS:
                MAINT_Ptf1Parameters[int(SENSOR_SOURCE::GYROSCOPE)][int(EUCLIDEAN_AXES::X)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                MAINT_Ptf1Parameters[int(SENSOR_SOURCE::GYROSCOPE)][int(EUCLIDEAN_AXES::Y)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                MAINT_Ptf1Parameters[int(SENSOR_SOURCE::GYROSCOPE)][int(EUCLIDEAN_AXES::Z)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[8]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_PTF1_MAGN_PARAMS:
                MAINT_Ptf1Parameters[int(SENSOR_SOURCE::MAGNETOMETER)][int(EUCLIDEAN_AXES::X)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                MAINT_Ptf1Parameters[int(SENSOR_SOURCE::MAGNETOMETER)][int(EUCLIDEAN_AXES::Y)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[4]));
                MAINT_Ptf1Parameters[int(SENSOR_SOURCE::MAGNETOMETER)][int(EUCLIDEAN_AXES::Z)] = (*reinterpret_cast<uint32_t*>(&rx_message.payload[8]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_SET_IMU_TYPE:
                MAINT_ImuType = IMU_TYPE(*reinterpret_cast<uint32_t*>(&rx_message.payload[0]));
                break;
            case MAINT_CMD_ID::MAINT_CMD_I2C_READ:
                MAINT_I2CRead = i2cReadByteFromRegister(*reinterpret_cast<uint32_t*>(&rx_message.payload[0]) == 0 ? i2c0 : i2c1,
                                                        (uint8_t)(*reinterpret_cast<uint32_t*>(&rx_message.payload[4]) & 0xFF),
                                                        (uint8_t)(*reinterpret_cast<uint32_t*>(&rx_message.payload[8]) & 0xFF));
                break;
            case MAINT_CMD_ID::MAINT_CMD_I2C_WRITE:
                i2cWriteByteToRegister(*reinterpret_cast<uint32_t*>(&rx_message.payload[0]) == 0 ? i2c0 : i2c1,
                                                        (uint8_t)(*reinterpret_cast<uint32_t*>(&rx_message.payload[4])  & 0xFF),
                                                        (uint8_t)(*reinterpret_cast<uint32_t*>(&rx_message.payload[8])  & 0xFF),
                                                        (uint8_t)(*reinterpret_cast<uint32_t*>(&rx_message.payload[12]) & 0xFF));
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
        if (tx_message.header.Bits.motor_params)
        {
            memcpy(&tx_message.payload[tx_payload_idx], MAINT_MotorsParameters, FLASH_MOTORS_PARAMS_SIZE);
            tx_payload_idx += FLASH_MOTORS_PARAMS_SIZE;
        }
        if (tx_message.header.Bits.js_params)
        {
            memcpy(&tx_message.payload[tx_payload_idx], MAINT_JoystickParameters, FLASH_JOYSTICK_PARAMS_SIZE);
            tx_payload_idx += FLASH_JOYSTICK_PARAMS_SIZE;
        }
        if (tx_message.header.Bits.pid_params)
        {
            memcpy(&tx_message.payload[tx_payload_idx], MAINT_PidParameters, FLASH_PID_PARAMS_SIZE);
            tx_payload_idx += FLASH_PID_PARAMS_SIZE;
        }
        if (tx_message.header.Bits.ptf1_params)
        {
            memcpy(&tx_message.payload[tx_payload_idx], MAINT_Ptf1Parameters, FLASH_PTF1_PARAMS_SIZE);
            tx_payload_idx += FLASH_PTF1_PARAMS_SIZE;
        }
        if (tx_message.header.Bits.imu_type)
        {
            uint32_t idata = static_cast<uint32_t>(MAINT_ImuType);
            memcpy(&tx_message.payload[tx_payload_idx], reinterpret_cast<uint32_t*>(&idata), sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.i2c_read)
        {
            uint32_t idata = static_cast<uint32_t>(MAINT_I2CRead);
            memcpy(&tx_message.payload[tx_payload_idx], reinterpret_cast<uint32_t*>(&idata), sizeof(uint32_t));
            tx_payload_idx += sizeof(uint32_t);
        }
        if (tx_message.header.Bits.sw_ver)
        {
            SW_VER_TAG sw_ver = {MAJOR_V, MINOR_V, STAGE_V, static_cast<uint8_t>(REL_TYPE)};

            memcpy(&tx_message.payload[tx_payload_idx], reinterpret_cast<uint32_t*>(&sw_ver), sizeof(uint32_t));
            
            tx_payload_idx += sizeof(uint32_t);
        }

        tx_message.payload[tx_payload_idx] = checksum(reinterpret_cast<uint8_t*>(&tx_message), sizeof(MAINT_HEADER_T) + tx_payload_idx);

        put_packet(reinterpret_cast<uint8_t*>(&tx_message), sizeof(MAINT_HEADER_T) + tx_payload_idx + 1, sender);
        
        shall_tx = false;
    }

    if (MAINT_FlashWriteRequested)
    {
        /** Wait for CPU1 to disable interrupts **/
        while (CPU1_WaitFlashWritten == false)
        {
            sleep_ms(1000);
        }
        /** CPU1 has disabled interrupts **/

        flash_write_params();

        MAINT_FlashWriteRequested = false;
        
        /** Notify CPU1 write terminated **/
        CPU1_WaitFlashWritten = false;
    }

    
}


bool MAINT_IsPresent()
{
    return (time_us_64() - last_msg_us) < MAINT_TIMEOUT_S * SECONDS_TO_MICROSECONDS;
}


void MAINT_UsbHandler()
{
    int byteIn = getchar();
    MAINT_OnByteReceived((uint8_t)byteIn & 0xFF, MaintClient::USB);
}


void MAINT_UartHandler()
{
#if 0
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    uint32_t tx_size;

    if (uart_ll_tx_buf > 0 && uart_ll_tx_buf < UART_TX_CHUNK_SIZE)
    {
        tx_size = uart_ll_tx_buf;
    }
    else if (uart_ll_tx_buf >= UART_TX_CHUNK_SIZE)
    {
        tx_size = UART_TX_CHUNK_SIZE;
    }
    else
    {
        tx_size = 0;
    }

    for (uint32_t i = 0; i < tx_size; i++)
    {
        uart_putc_raw(uart0, uart_tx_buf[i]);
    }
    
    for (uint32_t i = tx_size; i < uart_ll_tx_buf; i++)
    {
        uart_tx_buf[i - tx_size] = uart_tx_buf[i];
    }

    uart_ll_tx_buf -= tx_size;
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
#endif
}


bool MAINT_IsControllingMotors()
{
    return controlling_motors;
}