#ifndef MAINT_H
#define MAINT_H

#include <qobject.h>
#include <qserialport.h>
#include <qmutex.h>
#include <qtimer.h>

#define FLASH_MOTORS_PARAMS_SIZE   48
#define FLASH_JOYSTICK_PARAMS_SIZE 32
#define FLASH_PID_PARAMS_SIZE      72
#define FLASH_PTF1_PARAMS_SIZE     36
#define FLASH_IMU_TYPE_SIZE         1
#define FLASH_THROTTLE_PARAMS_SIZE  6

namespace Maint
{
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
            uint64_t throttle_params : 1;//56
            uint64_t maint_cmd_id : 7; //57, 58, 59, 60, 61, 62, 63
        } Bits;

        uint8_t  Bytes[8];
        uint64_t All;
    } MAINT_HEADER_T;

    typedef union
    {
        struct
        {
            uint32_t imu_failure : 1;
            uint32_t js_timeout : 1;
            uint32_t maint_timeout : 1;
            uint32_t cpu0_launch_failure : 1;
            uint32_t cpu1_launch_failure : 1;
            uint32_t cpu0_timer_failure : 1;
            uint32_t cpu1_timer_failure : 1;
            uint32_t : 25;
        } Bits;
        uint8_t Bytes[4];
        uint32_t Dword;
    } CBIT_TAG;

    typedef struct
    {
        uint8_t override_radio;
        uint16_t armed_signal;
        uint16_t roll_signal;
        uint16_t pitch_signal;
        uint16_t throttle_signal;
    } REMOTE_CONTROL_TAG;

    typedef struct
    {
        uint8_t major_v;
        uint8_t minor_v;
        uint8_t stage_v;
        uint8_t rel_type;
    } SW_VER_TAG;

    enum class REL_TYPE : uint8_t
    {
        BETA = 0x00,
        RELEASE = 0x01
    };

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
        WAIT_PAYLOAD
    };

    enum class TX_STATUS
    {
        TX_GET = 0,
        TX_SET
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

    static inline uint8_t checksum(uint8_t* buf, uint32_t size, bool firstSync=false)
    {
        uint8_t cks = 0;
        uint32_t first = (firstSync) ? 1 : 0;
        for (uint32_t i = first; i < size; i++)
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
	Maintenance();
    bool Open(QString serialPortName, enum QSerialPort::BaudRate baud = QSerialPort::Baud38400);
    void Close();
    void EnableTx(int delayMillis);
    void UpdateGetMessageHeader(MAINT_HEADER_T txHeader);
    void UpdateRemoteControlTag(bool override_radio, uint16_t armed_signal, uint16_t roll_signal, uint16_t pitch_signal, uint16_t throttle_signal);
    void TxControlMotors(bool control_motors);
    void TxSetMotors(uint32_t motorNo, uint16_t data);
    void TxMotorParams(uint32_t motorNo, uint8_t enabled, uint16_t minParam, uint16_t maxParam);
    void TxJoystickParams(uint32_t jsChannel, float alpha, float beta);
    void TxPidParams(uint32_t eulerAngle, float kp, float ki, float kt, float sat, float ad, float bd);
    void TxPtf1params(uint32_t sensorSource, float x, float y, float z);
    void TxThrottleParams(uint16_t descend, uint16_t hovering, uint16_t climb);
    void TxImuType(IMU_TYPE imuType);
    void I2CRead(uint8_t i2c, uint8_t addr, uint8_t reg);
    void I2CWrite(uint8_t i2c, uint8_t addr, uint8_t reg, uint8_t val);
    void TxWriteToFlash();
    void ReplayLogFile(const char* path);
    void CreateMatlabMatrix(const char* path);
    void TxImuOffset(float roll_offset, float pitch_offset);
    int ClearLogs();

public slots:
    void Tx();
    void OnRx();

private slots:
    void logBytes(quint8* data, int size);
    void onDownlinkTimeout();

signals:
    void rxRawData(bool valid, quint8* data, int size);
    void rxBytes(quint8* bytes, int size);
    void txRawData(quint8* data, int size);
    void downlink();

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
    void receivedThrottleSgn(uint16_t data);
    void receivedRollSgn(uint16_t data);
    void receivedPitchSgn(uint16_t data);
    void receivedCmdThr(uint16_t data);
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
    void receivedMotor1(uint16_t data);
    void receivedMotor2(uint16_t data);
    void receivedMotor3(uint16_t data);
    void receivedMotor4(uint16_t data);
    void receivedMotorsArmed(uint8_t data);
    void receivedCbit(uint32_t data);
    void receivedMotorsParams(uint32_t motor_no, bool enabled, uint16_t min_signal, uint16_t max_signal);
    void receivedJsParams(uint32_t channel_no, float alpha, float beta);
    void receivedPidParams(uint32_t angle_no, float kp, float ki, float kt, float sat, float ad, float bd);
    void receivedPtf1Params(uint32_t source_no, float x, float y, float z);
    void receivedImuType(uint8_t imu_type);
    void receivedI2CRead(uint8_t i2c_read);
    void receivedSwVer(uint8_t major_v, uint8_t minor_v, uint8_t stage_v, uint8_t rel_type);
    void receivedImuOffset(float offset_roll, float offset_pitch);
    void receivedThrottleParams(uint16_t descend, uint16_t hovering, uint16_t climb);

private:
	QSerialPort* _serialPort;
    QTimer* _checkDownlink;
    Maint::MAINT_STATUS _status;
    Maint::TX_STATUS _txStatus;
    uint32_t _expected_bytes;
    uint32_t _rx_payload_idx;
    uint8_t _rx_buf[1024];
    Maint::MAINT_HEADER_T _txMessageGet;
    Maint::MAINT_HEADER_T _txMessageSet;
    Maint::REMOTE_CONTROL_TAG _remCtrl;

    QList<uint8_t> _txSetParams;

    QTimer* _txTimer;
    QString _logFileName;
    FILE* _logFile;

    void pushParams(uint8_t* bytes, int size);
    void update_fsm(uint8_t byte_rx);
    void data_ingest(uint8_t rx_cks, uint32_t data_len);
    uint32_t calc_exp_bytes(Maint::MAINT_HEADER_T* header);
    QByteArray txMsg(Maint::MAINT_HEADER_T* header);
};

};

#endif //MAINT_H