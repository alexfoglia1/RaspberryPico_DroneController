#ifndef MAINT_H
#define MAINT_H

#include <qobject.h>
#include <qserialport.h>
#include <qmutex.h>
#include <qtimer.h>

namespace Maint
{
    enum class IMU_TYPE : uint32_t
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
            uint64_t maint_cmd_id : 8;    //56, 57, 58, 59, 60, 61, 62, 63
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
        TX_SET_CMD,
        TX_SET_PARAMS,
        TX_SET_JS_PARAMS,
        TX_SET_PID_PARAMS,
        TX_SET_PTF1_PARAMS,
        TX_SET_IMU_TYPE,
        TX_I2C_READ,
        TX_I2C_WRITE,
        TX_WRITE_TO_FLASH,
        TX_SET_IMU_OFFSET,
        TX_SET_OVERRIDE_RADIO,
        TX_SET_OVERRIDE_ROLL_PITCH_THROTTLE_SIGNAL,
        TX_SET_OVERRIDE_ARMED_SIGNAL
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
        MAINT_CMD_SET_OVERRIDE_RADIO,
        MAINT_CMD_SET_ROLL_PITCH_THROTTLE_SIGNAL,
        MAINT_CMD_SET_ARMED_SIGNAL
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
    void SetTxHeader(MAINT_HEADER_T txHeader);
    void TxMaintenanceCommand(uint32_t motorNo, uint32_t data);
    void TxMaintenanceParams(uint32_t motorNo, bool enabled, uint32_t minParam, uint32_t maxParam);
    void TxJoystickParams(uint32_t jsChannel, float alpha, float beta);
    void TxPidParams(uint32_t eulerAngle, float kp, float ki, float kt, float sat, float ad, float bd);
    void TxPtf1params(uint32_t sensorSource, float x, float y, float z);
    void TxImuType(IMU_TYPE imuType);
    void I2CRead(uint32_t i2c, uint32_t addr, uint32_t reg);
    void I2CWrite(uint32_t i2c, uint32_t addr, uint32_t reg, uint32_t val);
    void TxWriteToFlash();
    void SetImuOffset(float roll_offset, float pitch_offset);
    void TxOverrideRadio(bool is_override);
    void TxSetArmedSignal(uint16_t signal);
    void TxSetRollPitchThrottleSignal(uint16_t roll_signal, uint16_t pitch_signal, uint16_t throttle_signal);
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
    void receivedCbit(uint32_t data);
    void receivedMotorsParams(uint32_t motor_no, bool enabled, uint32_t min_signal, uint32_t max_signal);
    void receivedJsParams(uint32_t channel_no, float alpha, float beta);
    void receivedPidParams(uint32_t angle_no, float kp, float ki, float kt, float sat, float ad, float bd);
    void receivedPtf1Params(uint32_t source_no, float x, float y, float z);
    void receivedImuType(uint32_t imu_type);
    void receivedI2CRead(uint32_t i2c_read);
    void receivedSwVer(uint8_t major_v, uint8_t minor_v, uint8_t stage_v, uint8_t rel_type);
    void receivedImuOffset(float offset_roll, float offset_pitch);

private:
	QSerialPort* _serialPort;
    QTimer* _checkDownlink;
    Maint::MAINT_STATUS _status;
    Maint::TX_STATUS _txStatus;
    uint32_t _expected_bytes;
    uint32_t _rx_payload_idx;
    uint8_t _rx_buf[1024];
    Maint::MAINT_HEADER_T _txHeader;
    Maint::MAINT_HEADER_T _txCommand;
    uint32_t _tx_data;
    uint32_t _tx_param_enabled;
    uint32_t _tx_param_min_signal;
    uint32_t _tx_param_max_signal;
    uint32_t _tx_param_js_alpha;
    uint32_t _tx_param_js_beta;
    uint32_t _tx_param_kp;
    uint32_t _tx_param_ki;
    uint32_t _tx_param_kt;
    uint32_t _tx_param_sat;
    uint32_t _tx_param_ad;
    uint32_t _tx_param_bd;
    uint32_t _tx_param_x;
    uint32_t _tx_param_y;
    uint32_t _tx_param_z;
    uint32_t _tx_param_imu;
    uint32_t _tx_param_i2c_chan;
    uint32_t _tx_param_i2c_addr;
    uint32_t _tx_param_i2c_reg;
    uint32_t _tx_param_i2c_val;
    float _tx_param_roll_offset;
    float _tx_param_pitch_offset;
    uint32_t _tx_param_override_radio;
    uint16_t _tx_param_override_armed_signal;
    uint16_t _tx_param_override_roll_signal;
    uint16_t _tx_param_override_pitch_signal;
    uint16_t _tx_param_override_throttle_signal;
    QMutex _txMutex;
    QTimer* _txTimer;
    QString _logFileName;
    FILE* _logFile;


    void update_fsm(uint8_t byte_rx);
    void data_ingest(uint8_t rx_cks, uint32_t data_len);
    uint32_t calc_exp_bytes(Maint::MAINT_HEADER_T* header);
};

};

#endif //MAINT_H