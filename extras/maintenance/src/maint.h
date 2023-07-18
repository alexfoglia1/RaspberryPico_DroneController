#ifndef MAINT_H
#define MAINT_H

#include <qobject.h>
#include <qserialport.h>
#include <qmutex.h>

namespace Maint
{
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
            uint64_t reserved : 13;    //47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59
            uint64_t maint_cmd_id : 4;     //60,61,62,63
        } Bits;

        uint8_t  Bytes[8];
        uint64_t All;
    } MAINT_HEADER_T;

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
        MAINT_CMD_SET_M4
    };

    static inline uint8_t checksum(uint8_t* buf, uint32_t size)
    {
        uint8_t cks = 0;
        for (uint32_t i = 0; i < size; i++)
        {
            cks ^= buf[i];
        }
        return cks;
    }

    const int SYNC_CHAR = 0xFF;

class Maintenance : public QObject
{
	Q_OBJECT
public:
	Maintenance(QString serialPortName);
    void Open();
    void EnableTx();
    void SetTxHeader(MAINT_HEADER_T txHeader);

public slots:
    void Tx();
    void OnRx();

signals:
    void receivedRawAccelX(float data);
    void receivedRawAccelY(float data);
    void receivedRawAccelZ(float data);
    void receivedRawGyroX(float data);
    void receivedRawGyroY(float data);
    void receivedRawGyroZ(float data);
    void receivedRawMagnX(float data);
    void receivedRawMagnY(float data);
    void receivedRawMagnZ(float data);
    void receivedFilteredAccelX(float data);
    void receivedFilteredAccelY(float data);
    void receivedFilteredAccelZ(float data);
    void receivedFilteredGyroX(float data);
    void receivedFilteredGyroY(float data);
    void receivedFilteredGyroZ(float data);
    void receivedFilteredMagnX(float data);
    void receivedFilteredMagnY(float data);
    void receivedFilteredMagnZ(float data);
    void receivedThrottleSgn(uint32_t data);
    void receivedRollSgn(uint32_t data);
    void receivedPitchSgn(uint32_t data);
    void receivedCmdThr(float data);
    void receivedCmdRoll(float data);
    void receivedCmdPitch(float data);
    void receivedBodyRoll(float data);
    void receivedBodyPitch(float data);
    void receivedBodyYaw(float data);
    void receivedRollPidErr(float data);
    void receivedRollPidP(float data);
    void receivedRollPidI(float data);
    void receivedRollPidD(float data);
    void receivedRollPidU(float data);
    void receivedPitchPidErr(float data);
    void receivedPitchPidP(float data);
    void receivedPitchPidI(float data);
    void receivedPitchPidD(float data);
    void receivedPitchPidU(float data);
    void receivedYawPidErr(float data);
    void receivedYawPidP(float data);
    void receivedYawPidI(float data);
    void receivedYawPidD(float data);
    void receivedYawPidU(float data);
    void receivedMotor1(uint32_t data);
    void receivedMotor2(uint32_t data);
    void receivedMotor3(uint32_t data);
    void receivedMotor4(uint32_t data);
    void receivedMotorsArmed(uint32_t data);

private:
	QSerialPort* _serialPort;
    Maint::MAINT_STATUS _status;
    uint32_t _expected_bytes;
    uint32_t _rx_payload_idx;
    uint8_t _rx_buf[1024];
    Maint::MAINT_HEADER_T _txHeader;
    QMutex _txMutex;

    void update_fsm(uint8_t byte_rx);
    void data_ingest(uint8_t rx_cks, uint32_t data_len);
    uint32_t calc_exp_bytes(Maint::MAINT_HEADER_T* header);
};

};

#endif //MAINT_H