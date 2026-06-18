#include "maint.h"

#include <qdatetime.h>
#include <qdir.h>
#include <qdiriterator.h>
#include <qfile.h>
#include <qtextstream.h>
#include <qregularexpression.h>
#include <qthread.h>

#include <cstring>
#include <cstdint>

namespace
{
    static QString hexU8(uint8_t v)
    {
        return QString("0x%1")
            .arg(static_cast<uint32_t>(v), 2, 16, QLatin1Char('0'))
            .toUpper();
    }

    static QString hexU16(uint16_t v)
    {
        return QString("0x%1")
            .arg(static_cast<uint32_t>(v), 4, 16, QLatin1Char('0'))
            .toUpper();
    }

    static QString hexU32(uint32_t v)
    {
        return QString("0x%1")
            .arg(v, 8, 16, QLatin1Char('0'))
            .toUpper();
    }

    static QString hexU64(uint64_t v)
    {
        return QString("0x%1")
            .arg(static_cast<qulonglong>(v), 16, 16, QLatin1Char('0'))
            .toUpper();
    }

    static uint16_t readLeU16(const uint8_t* p)
    {
        return static_cast<uint16_t>(p[0]) |
            static_cast<uint16_t>(p[1] << 8);
    }

    static uint32_t readLeU32(const uint8_t* p)
    {
        return static_cast<uint32_t>(p[0]) |
            static_cast<uint32_t>(p[1]) << 8 |
            static_cast<uint32_t>(p[2]) << 16 |
            static_cast<uint32_t>(p[3]) << 24;
    }

    static uint64_t readLeU64(const uint8_t* p)
    {
        uint64_t v = 0;

        for (int i = 0; i < 8; ++i)
        {
            v |= static_cast<uint64_t>(p[i]) << (8 * i);
        }

        return v;
    }

    static float readLeF32(const uint8_t* p)
    {
        uint32_t raw = readLeU32(p);

        float f;
        std::memcpy(&f, &raw, sizeof(float));

        return f;
    }

    static QString byteDump(const uint8_t* data, uint32_t len)
    {
        QString s;

        for (uint32_t i = 0; i < len; ++i)
        {
            s += hexU8(data[i]);
            s += " ";
        }

        return s.trimmed();
    }

    static const char* bitName(int bit)
    {
        static const char* names[] =
        {
            "accel_x",          // 0
            "accel_y",          // 1
            "accel_z",          // 2
            "gyro_x",           // 3
            "gyro_y",           // 4
            "gyro_z",           // 5
            "magn_x",           // 6
            "magn_y",           // 7
            "magn_z",           // 8
            "accel_x_f",        // 9
            "accel_y_f",        // 10
            "accel_z_f",        // 11
            "gyro_x_f",         // 12
            "gyro_y_f",         // 13
            "gyro_z_f",         // 14
            "magn_x_f",         // 15
            "magn_y_f",         // 16
            "magn_z_f",         // 17
            "throttle_sgn",     // 18
            "roll_sgn",         // 19
            "pitch_sgn",        // 20
            "cmd_thr",          // 21
            "cmd_roll",         // 22
            "cmd_pitch",        // 23
            "body_roll",        // 24
            "body_pitch",       // 25
            "body_yaw",         // 26
            "roll_pid_err",     // 27
            "roll_pid_p",       // 28
            "roll_pid_i",       // 29
            "roll_pid_d",       // 30
            "roll_pid_u",       // 31
            "pitch_pid_err",    // 32
            "pitch_pid_p",      // 33
            "pitch_pid_i",      // 34
            "pitch_pid_d",      // 35
            "pitch_pid_u",      // 36
            "yaw_pid_err",      // 37
            "yaw_pid_p",        // 38
            "yaw_pid_i",        // 39
            "yaw_pid_d",        // 40
            "yaw_pid_u",        // 41
            "motor1",           // 42
            "motor2",           // 43
            "motor3",           // 44
            "motor4",           // 45
            "motors_armed",     // 46
            "cbit",             // 47
            "motor_params",     // 48
            "js_params",        // 49
            "pid_params",       // 50
            "ptf1_params",      // 51
            "imu_type",         // 52
            "i2c_read",         // 53
            "sw_ver",           // 54
            "imu_offset",       // 55
            "throttle_params"   // 56
        };

        if (bit < 0 || bit >= static_cast<int>(sizeof(names) / sizeof(names[0])))
        {
            return "unknown";
        }

        return names[bit];
    }
}

Maint::Maintenance::Maintenance()
{
    _logFile = NULL;

    _txMessageGet.All = 0;
    _txMessageSet.All = 0;

    _serialPort = new QSerialPort();
    _txTimer = new QTimer();

    _status = Maint::MAINT_STATUS::WAIT_SYNC;
    _txStatus = Maint::TX_STATUS::TX_GET;

    _expected_bytes = 0;
    _rx_payload_idx = 0;
    memset(&_rx_buf, 0x00, 1024);

    _remCtrl.override_radio = 1;
    _remCtrl.armed_signal = 2000;
    _remCtrl.roll_signal = 1500;
    _remCtrl.pitch_signal = 1500;
    _remCtrl.throttle_signal = 1000;

    _checkDownlink = new QTimer();
}

bool Maint::Maintenance::Open(QString serialPortName, enum QSerialPort::BaudRate baud)
{
    _serialPort->setPortName(serialPortName);
	_serialPort->setBaudRate(baud);
	_serialPort->setParity(QSerialPort::NoParity);
	_serialPort->setDataBits(QSerialPort::Data8);
	_serialPort->setStopBits(QSerialPort::OneStop);
	_serialPort->setFlowControl(QSerialPort::NoFlowControl);

    _logFileName = QString("log-%1-%2.txt").arg(QDateTime::currentDateTime().toString().replace(" ", "-").replace(":","-")).arg(serialPortName.replace("/", ""));
    _logFile = fopen(_logFileName.toStdString().c_str(), "w");

	connect(_serialPort, SIGNAL(readyRead()), this, SLOT(OnRx()));
    connect(this, SIGNAL(rxBytes(quint8*, int)), this, SLOT(logBytes(quint8*, int)));

	bool ret = _serialPort->open(QSerialPort::OpenModeFlag::ReadWrite);

    _serialPort->setDataTerminalReady(true);
    _serialPort->setRequestToSend(true);

    if (ret)
    {
        _checkDownlink->setInterval(1000);
        _checkDownlink->setSingleShot(false);
        _checkDownlink->setTimerType(Qt::PreciseTimer);

        connect(_checkDownlink, SIGNAL(timeout()), this, SLOT(onDownlinkTimeout()));
        _checkDownlink->start();
    }

    return ret;
}


int Maint::Maintenance::ClearLogs()
{
    QDirIterator it(".", QStringList() << "log-*", QDir::Files, QDirIterator::Subdirectories);
    int n = 0;
    while (it.hasNext())
    {
        QFile f(it.next());
        if (f.exists())
        {
            n++;
            f.remove();
        }
    }

    return n;
}


void Maint::Maintenance::Close()
{
    _txTimer->stop();
    _checkDownlink->stop();

    disconnect(_txTimer, SIGNAL(timeout()), this, SLOT(Tx()));
    disconnect(_checkDownlink, SIGNAL(timeout()), this, SLOT(onDownlinkTimeout()));
    disconnect(this, SIGNAL(rxBytes(quint8*, int)), this, SLOT(logBytes(quint8*, int)));

    _serialPort->disconnect(_serialPort, SIGNAL(readyRead()), this, SLOT(OnRx()));
    _serialPort->close();

    fclose(_logFile);
}


void Maint::Maintenance::EnableTx(int delayMillis)
{
    _txTimer->setInterval(delayMillis);
    _txTimer->setSingleShot(false);
    _txTimer->setTimerType(Qt::PreciseTimer);

    connect(_txTimer, SIGNAL(timeout()), this, SLOT(Tx()));
    _txTimer->start();
}


void Maint::Maintenance::UpdateGetMessageHeader(MAINT_HEADER_T txHeader)
{
    _txMessageGet.All = txHeader.All;
}



void Maint::Maintenance::UpdateRemoteControlTag(bool override_radio, uint16_t armed_signal, uint16_t roll_signal, uint16_t pitch_signal, uint16_t throttle_signal)
{
    _remCtrl.override_radio = override_radio ? 1 : 0;
    _remCtrl.armed_signal = armed_signal;
    _remCtrl.roll_signal = roll_signal;
    _remCtrl.pitch_signal = pitch_signal;
    _remCtrl.throttle_signal = throttle_signal;
}


void Maint::Maintenance::TxControlMotors(bool control_motors)
{
    _txMessageSet.All = 0;
    _txMessageSet.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_CTRL_MOTORS);

    uint8_t data = control_motors ? 1 : 0;
    _txSetParams.clear();
    pushParams(reinterpret_cast<uint8_t*>(&data), sizeof(uint8_t));
    
    _txStatus = Maint::TX_STATUS::TX_SET;
}


void Maint::Maintenance::TxSetMotors(uint32_t motorNo, uint16_t data)
{
    _txMessageSet.All = 0;
    _txMessageSet.Bits.maint_cmd_id = (motorNo == 1) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M1) :
                                      (motorNo == 2) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M2) :
                                      (motorNo == 3) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M3) :
                                      (motorNo == 4) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M4) : uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_MALL);
    
    if (_txMessageSet.Bits.maint_cmd_id == uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE))
    {
        return;
    }

    _txSetParams.clear();
    pushParams(reinterpret_cast<uint8_t*>(&data), sizeof(uint16_t));

    _txStatus = Maint::TX_STATUS::TX_SET;
}


void Maint::Maintenance::TxMotorParams(uint32_t motorNo, uint8_t enabled, uint16_t minSignalParam, uint16_t maxSignalParam)
{
    _txMessageSet.All = 0;
    _txMessageSet.Bits.maint_cmd_id =  (motorNo == 1) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M1_PARAMS) :
                                    (motorNo == 2) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M2_PARAMS) :
                                    (motorNo == 3) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M3_PARAMS) :
                                    (motorNo == 4) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M4_PARAMS) : uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE);

    if (_txMessageSet.Bits.maint_cmd_id != uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE))
    {

        _txSetParams.clear();
        pushParams(reinterpret_cast<uint8_t*>(&enabled), sizeof(uint8_t));
        pushParams(reinterpret_cast<uint8_t*>(&minSignalParam), sizeof(uint16_t));
        pushParams(reinterpret_cast<uint8_t*>(&maxSignalParam), sizeof(uint16_t));
        _txStatus = Maint::TX_STATUS::TX_SET;
    }
}

void Maint::Maintenance::TxJoystickParams(uint32_t jsChannel, float alpha, float beta)
{
    _txMessageSet.All = 0;
    _txMessageSet.Bits.maint_cmd_id = (jsChannel == 1) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_JS_THROTTLE_ALPHA_BETA) :
        (jsChannel == 2) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_JS_ROLL_ALPHA_BETA) :
        (jsChannel == 3) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_JS_PITCH_ALPHA_BETA) : uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE);

    if (_txMessageSet.Bits.maint_cmd_id != uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE))
    {
        _txSetParams.clear();
        pushParams(reinterpret_cast<uint8_t*>(&alpha), sizeof(float));
        pushParams(reinterpret_cast<uint8_t*>(&beta), sizeof(float));

        _txStatus = Maint::TX_STATUS::TX_SET;
    }
}


void Maint::Maintenance::TxPidParams(uint32_t eulerAngle, float kp, float ki, float kd, float sat)
{
    _txMessageSet.All = 0;
    _txMessageSet.Bits.maint_cmd_id = (eulerAngle == 1) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_ROLL_PID_PARAMS) :
        (eulerAngle == 2) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_PITCH_PID_PARAMS) :
        (eulerAngle == 3) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_YAW_PID_PARAMS) : uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE);

    if (_txMessageSet.Bits.maint_cmd_id != uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE))
    {
        _txSetParams.clear();
        pushParams(reinterpret_cast<uint8_t*>(&kp), sizeof(float));
        pushParams(reinterpret_cast<uint8_t*>(&ki), sizeof(float));
        pushParams(reinterpret_cast<uint8_t*>(&kd), sizeof(float));
        pushParams(reinterpret_cast<uint8_t*>(&sat), sizeof(float));
        
        _txStatus = Maint::TX_STATUS::TX_SET;
        
    }
}


void Maint::Maintenance::TxPtf1params(uint32_t sensorSource, float x, float y, float z)
{
    _txMessageSet.All = 0;
    _txMessageSet.Bits.maint_cmd_id = (sensorSource == 1) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_PTF1_ACC_PARAMS) :
        (sensorSource == 2) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_PTF1_GYRO_PARAMS) :
        (sensorSource == 3) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_PTF1_MAGN_PARAMS) : uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE);

    if (_txMessageSet.Bits.maint_cmd_id != uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE))
    {
        _txSetParams.clear();
        pushParams(reinterpret_cast<uint8_t*>(&x), sizeof(float));
        pushParams(reinterpret_cast<uint8_t*>(&y), sizeof(float));
        pushParams(reinterpret_cast<uint8_t*>(&z), sizeof(float));
        
        _txStatus = Maint::TX_STATUS::TX_SET;
        
    }
}


void Maint::Maintenance::TxThrottleParams(uint16_t descend, uint16_t hovering, uint16_t climb)
{
    _txMessageSet.All = 0;
    _txMessageSet.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_THROTTLE_PARAMS);

    _txSetParams.clear();
    pushParams(reinterpret_cast<uint8_t*>(&descend), sizeof(uint16_t));
    pushParams(reinterpret_cast<uint8_t*>(&hovering), sizeof(uint16_t));
    pushParams(reinterpret_cast<uint8_t*>(&climb), sizeof(uint16_t));

    _txStatus = Maint::TX_STATUS::TX_SET;
}


void Maint::Maintenance::TxImuType(IMU_TYPE imuType)
{
    _txMessageSet.All = 0;
    _txMessageSet.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_IMU_TYPE);

    uint32_t iImuType = static_cast<uint32_t>(imuType);

    _txSetParams.clear();
    pushParams(reinterpret_cast<uint8_t*>(&iImuType), sizeof(uint8_t));
    
    _txStatus = Maint::TX_STATUS::TX_SET;
    
}


void Maint::Maintenance::I2CRead(uint8_t i2c, uint8_t addr, uint8_t reg)
{
    _txMessageSet.All = 0;
    _txMessageSet.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_I2C_READ);

    _txSetParams.clear();
    pushParams(reinterpret_cast<uint8_t*>(&i2c), sizeof(uint8_t));
    pushParams(reinterpret_cast<uint8_t*>(&addr), sizeof(uint8_t));
    pushParams(reinterpret_cast<uint8_t*>(&reg), sizeof(uint8_t));

    
    _txStatus = Maint::TX_STATUS::TX_SET;
    
}


void Maint::Maintenance::I2CWrite(uint8_t i2c, uint8_t addr, uint8_t reg, uint8_t val)
{
    _txMessageSet.All = 0;
    _txMessageSet.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_I2C_WRITE);

    _txSetParams.clear();
    pushParams(reinterpret_cast<uint8_t*>(&i2c), sizeof(uint8_t));
    pushParams(reinterpret_cast<uint8_t*>(&addr), sizeof(uint8_t));
    pushParams(reinterpret_cast<uint8_t*>(&reg), sizeof(uint8_t));
    pushParams(reinterpret_cast<uint8_t*>(&val), sizeof(uint8_t));
    
    _txStatus = Maint::TX_STATUS::TX_SET;
    
}


void Maint::Maintenance::TxWriteToFlash()
{
    _txMessageSet.All = 0;
    _txMessageSet.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_FLASH_WRITE);

    _txSetParams.clear();

    _txStatus = Maint::TX_STATUS::TX_SET;
}


void Maint::Maintenance::CreateMatlabMatrix(const char* path)
{
    QFile file(path);
    QByteArray byteArray;

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qWarning("Cannot open file: %s", file.errorString().toUtf8().constData());
        return;
    }

    QTextStream in(&file);
    QRegularExpression regex("\\[.*?\\]\\s*(.*)");

    FILE* outputFile = fopen("output.m", "w");
    fprintf(outputFile, "M = [\n");
    QVector<QByteArray> binLines;
    QVector<int> syncIndexes;
    int byteIdx = 0;
    Maint::MAINT_STATUS parseStatus = Maint::MAINT_STATUS::WAIT_SYNC;
    int expectedPayloadSize = 0;
    int currentPayloadIndex = 0;
    int currentHeaderIndex = 0;
    while (!in.atEnd())
    {
        QString line = in.readLine();
        QRegularExpressionMatch match = regex.match(line);

        if (match.hasMatch())
        {
            QString byteString = match.captured(1); // Captures the part after the timestamp
            QStringList byteTokens = byteString.split(" ", Qt::SkipEmptyParts);

            for (const QString& token : byteTokens)
            {
                bool ok;
                int byteValue = token.toInt(&ok, 16); // Convert hex string to int
                if (ok)
                {
                    byteArray.append(static_cast<char>(byteValue));

                    switch (parseStatus)
                    {
                    case MAINT_STATUS::WAIT_SYNC:
                    {
                        if (byteValue == 0xFF && parseStatus == Maint::MAINT_STATUS::WAIT_SYNC)
                        {
                            parseStatus = MAINT_STATUS::WAIT_HEADER_BYTE_0;
                            syncIndexes.push_back(byteIdx);
                        }
                        break;
                    }
                    case MAINT_STATUS::WAIT_HEADER_BYTE_0:
                        currentHeaderIndex = byteIdx;
                        parseStatus = MAINT_STATUS::WAIT_HEADER_BYTE_1;
                        break;
                    case MAINT_STATUS::WAIT_HEADER_BYTE_1:
                        parseStatus = MAINT_STATUS::WAIT_HEADER_BYTE_2;
                        break;
                    case MAINT_STATUS::WAIT_HEADER_BYTE_2:
                        parseStatus = MAINT_STATUS::WAIT_HEADER_BYTE_3;
                        break;
                    case MAINT_STATUS::WAIT_HEADER_BYTE_3:
                        parseStatus = MAINT_STATUS::WAIT_HEADER_BYTE_4;
                        break;
                    case MAINT_STATUS::WAIT_HEADER_BYTE_4:
                        parseStatus = MAINT_STATUS::WAIT_HEADER_BYTE_5;
                        break;
                    case MAINT_STATUS::WAIT_HEADER_BYTE_5:
                        parseStatus = MAINT_STATUS::WAIT_HEADER_BYTE_6;
                        break;
                    case MAINT_STATUS::WAIT_HEADER_BYTE_6:
                        parseStatus = MAINT_STATUS::WAIT_HEADER_BYTE_7;
                        break;
                    case MAINT_STATUS::WAIT_HEADER_BYTE_7:
                        expectedPayloadSize = calc_exp_bytes(reinterpret_cast<MAINT_HEADER_T*>(byteArray.data() + currentHeaderIndex));
                        parseStatus = MAINT_STATUS::WAIT_PAYLOAD;
                        break;
                    case MAINT_STATUS::WAIT_PAYLOAD:
                        currentPayloadIndex += 1;

                        if (currentPayloadIndex == expectedPayloadSize)
                        {
                            expectedPayloadSize = 0;
                            currentPayloadIndex = 0;
                            currentHeaderIndex = 0;
                            parseStatus = MAINT_STATUS::WAIT_SYNC;
                        }
                        break;
                    }

                    byteIdx += 1;
                }
                else
                {
                    qWarning("Invalid byte value: %s", token.toUtf8().constData());
                }
            }
        }
        else
        {
            qWarning("Line does not match expected format: %s", line.toUtf8().constData());
        }
    }

    int nSyncs = syncIndexes.size();
    for (int i = 0; i < nSyncs; i++)
    {
        binLines.push_back(QByteArray());
    }
    for (int i = 0; i < nSyncs; i++)
    {
        int startCopy = syncIndexes.at(i) + 1;
        int endCopy = (i == nSyncs - 1) ? byteArray.size() - 1 : syncIndexes.at(i + 1) - 1;
        for (int j = startCopy; j < endCopy; j++)
        {
            binLines[i].push_back(byteArray.at(j));
        }
    }

    int binLineIdx = 0;
    for (auto& binLine : binLines)
    {
        const Maint::MAINT_HEADER_T* pHdr = reinterpret_cast<const Maint::MAINT_HEADER_T*>(binLine.constData());

        uint16_t motor1 = 0;
        uint16_t motor2 = 0;
        uint16_t motor3 = 0;
        uint16_t motor4 = 0;

        float roll = 0.0f;
        float pitch = 0.0f;
        if (pHdr->Bits.motor1 &&
            pHdr->Bits.motor2 &&
            pHdr->Bits.motor3 &&
            pHdr->Bits.motor4 &&
            pHdr->Bits.body_pitch &&
            pHdr->Bits.body_roll)
        {
            const uint8_t* pByte = reinterpret_cast<const uint8_t*>(binLine.constData());
            roll = *reinterpret_cast<const float*>(pByte + sizeof(Maint::MAINT_HEADER_T) + 0);
            pitch = *reinterpret_cast<const float*>(pByte + sizeof(Maint::MAINT_HEADER_T) + 1 * sizeof(float));
            motor1 = *reinterpret_cast<const uint16_t*>(pByte + sizeof(Maint::MAINT_HEADER_T) + 2 * sizeof(float));
            motor2 = *reinterpret_cast<const uint16_t*>(pByte + sizeof(Maint::MAINT_HEADER_T) + 2 * sizeof(float) + 1 * sizeof(uint16_t));
            motor3 = *reinterpret_cast<const uint16_t*>(pByte + sizeof(Maint::MAINT_HEADER_T) + 2 * sizeof(float) + 2 * sizeof(uint16_t));
            motor4 = *reinterpret_cast<const uint16_t*>(pByte + sizeof(Maint::MAINT_HEADER_T) + 2 * sizeof(float) + 3 * sizeof(uint16_t));

            fprintf(outputFile, "%d, %d, %d, %d, %.10f, %.10f;\n", motor1, motor2, motor3, motor4, roll, pitch);
        }

        binLineIdx += 1;
    }
    fprintf(outputFile, "];\n");
    fclose(outputFile);

    file.close();
}



void Maint::Maintenance::ReplayLogFile(const char* path)
{
    QFile file(path);
    QByteArray byteArray;

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qWarning("Cannot open file: %s", file.errorString().toUtf8().constData());
        return;
    }

    QTextStream in(&file);
    QRegularExpression regex("\\[.*?\\]\\s*(.*)");

    while (!in.atEnd())
    {
        QString line = in.readLine();
        QRegularExpressionMatch match = regex.match(line);

        if (match.hasMatch())
        {
            QString byteString = match.captured(1); // Captures the part after the timestamp
            QStringList byteTokens = byteString.split(" ", Qt::SkipEmptyParts);

            for (const QString& token : byteTokens)
            {
                bool ok;
                int byteValue = token.toInt(&ok, 16); // Convert hex string to int
                if (ok)
                {
                    byteArray.append(static_cast<char>(byteValue));
                }
                else
                {
                    qWarning("Invalid byte value: %s", token.toUtf8().constData());
                }
            }

            for (auto& byte : byteArray)
            {
                //printf("%c", byte);
                update_fsm(byte);
            }
            byteArray.clear(); // Clear the QByteArray for the next line
            QThread::msleep(10);
        }
        else
        {
            qWarning("Line does not match expected format: %s", line.toUtf8().constData());
        }
    }

    file.close();
}



void Maint::Maintenance::TxImuOffset(float roll_offset, float pitch_offset)
{
    _txMessageSet.All = 0;
    _txMessageSet.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_IMU_OFFSET);

    _txSetParams.clear();
    pushParams(reinterpret_cast<uint8_t*>(&roll_offset), sizeof(float));
    pushParams(reinterpret_cast<uint8_t*>(&pitch_offset), sizeof(float));
    
    _txStatus = Maint::TX_STATUS::TX_SET;

}


QByteArray Maint::Maintenance::txMsg(Maint::MAINT_HEADER_T* header)
{
    QByteArray qba;
    
    qba.push_back(header->Bytes[0]);
    qba.push_back(header->Bytes[1]);
    qba.push_back(header->Bytes[2]);
    qba.push_back(header->Bytes[3]);
    qba.push_back(header->Bytes[4]);
    qba.push_back(header->Bytes[5]);
    qba.push_back(header->Bytes[6]);
    qba.push_back(header->Bytes[7]);

    pushParams(reinterpret_cast<uint8_t*>(&_remCtrl.override_radio), sizeof(uint8_t));
    pushParams(reinterpret_cast<uint8_t*>(&_remCtrl.armed_signal), sizeof(uint16_t));
    pushParams(reinterpret_cast<uint8_t*>(&_remCtrl.roll_signal), sizeof(uint16_t));
    pushParams(reinterpret_cast<uint8_t*>(&_remCtrl.pitch_signal), sizeof(uint16_t));
    pushParams(reinterpret_cast<uint8_t*>(&_remCtrl.throttle_signal), sizeof(uint16_t));
    while (!_txSetParams.isEmpty())
    {
        qba.push_back(_txSetParams.takeFirst());
    }

    uint8_t cks = checksum(reinterpret_cast<uint8_t*>(qba.data()), qba.size());

    qba.push_back(cks);
    qba.push_front(SYNC_CHAR);

    _serialPort->write(qba);
    _serialPort->flush();

    return qba;
}



void Maint::Maintenance::Tx()
{
    QByteArray qba;

    if (_txStatus == Maint::TX_STATUS::TX_GET)
    {
        qba = txMsg(&_txMessageGet);
    }
    else
    {
        //motor_speed.html:109 Inviato messaggio al motore 1 con PWM 1080 e checksum : 213
        qba = txMsg(&_txMessageSet);
        _txStatus = Maint::TX_STATUS::TX_GET;
    }

    emit txRawData(reinterpret_cast<quint8*>(qba.data()), qba.size());
}


void Maint::Maintenance::OnRx()
{
    _checkDownlink->stop();
    _checkDownlink->start();

	QByteArray qba = _serialPort->readAll();
	
    emit rxBytes(reinterpret_cast<quint8*>(qba.data()), qba.size());

	for (auto& byte : qba)
	{
        //printf("%c", byte);
		update_fsm(byte);
	}
}


void Maint::Maintenance::pushParams(uint8_t* bytes, int size)
{
    for (int i = 0; i < size; i++)
    {
        _txSetParams.push_back(bytes[i]);
    }
}


void Maint::Maintenance::update_fsm(uint8_t byte_rx)
{
    switch (_status)
    {

    case Maint::MAINT_STATUS::WAIT_SYNC:
    {
        if (byte_rx == Maint::SYNC_CHAR)
        {
            _status = Maint::MAINT_STATUS::WAIT_HEADER_BYTE_0;
        }
        break;
    }
    case Maint::MAINT_STATUS::WAIT_HEADER_BYTE_0:
    {
        _rx_buf[0] = byte_rx;
        _status = Maint::MAINT_STATUS::WAIT_HEADER_BYTE_1;
        break;
    }
    case Maint::MAINT_STATUS::WAIT_HEADER_BYTE_1:
    {
        _rx_buf[1] = byte_rx;
        _status = Maint::MAINT_STATUS::WAIT_HEADER_BYTE_2;
        break;
    }
    case Maint::MAINT_STATUS::WAIT_HEADER_BYTE_2:
    {
        _rx_buf[2] = byte_rx;
        _status = Maint::MAINT_STATUS::WAIT_HEADER_BYTE_3;
        break;
    }
    case Maint::MAINT_STATUS::WAIT_HEADER_BYTE_3:
    {
        _rx_buf[3] = byte_rx;
        _status = Maint::MAINT_STATUS::WAIT_HEADER_BYTE_4;
        break;
    }
    case Maint::MAINT_STATUS::WAIT_HEADER_BYTE_4:
    {
        _rx_buf[4] = byte_rx;
        _status = Maint::MAINT_STATUS::WAIT_HEADER_BYTE_5;
        break;
    }
    case Maint::MAINT_STATUS::WAIT_HEADER_BYTE_5:
    {
        _rx_buf[5] = byte_rx;
        _status = Maint::MAINT_STATUS::WAIT_HEADER_BYTE_6;
        break;
    }
    case Maint::MAINT_STATUS::WAIT_HEADER_BYTE_6:
    {
        _rx_buf[6] = byte_rx;
        _status = Maint::MAINT_STATUS::WAIT_HEADER_BYTE_7;
        break;
    }
    case Maint::MAINT_STATUS::WAIT_HEADER_BYTE_7:
    {
        _rx_buf[7] = byte_rx;
        _status = Maint::MAINT_STATUS::WAIT_PAYLOAD;
        _expected_bytes = calc_exp_bytes(reinterpret_cast<MAINT_HEADER_T*>(&_rx_buf[0]));
        _rx_payload_idx = 0;
        break;
    }
    case Maint::MAINT_STATUS::WAIT_PAYLOAD:
    {
        _rx_buf[8 + _rx_payload_idx] = byte_rx;
        _rx_payload_idx += 1;

        if (_rx_payload_idx == _expected_bytes)
        {
            data_ingest(byte_rx, 8 + _rx_payload_idx);
            _status = Maint::MAINT_STATUS::WAIT_SYNC;
        }
        break;
    }
    }
}

void Maint::Maintenance::data_ingest(uint8_t rx_cks, uint32_t data_len)
{
    uint8_t local_cks = checksum(&_rx_buf[0], data_len - 1);
    bool cks_ok = local_cks == rx_cks;

    Maint::MAINT_HEADER_T* rx_header =
        reinterpret_cast<Maint::MAINT_HEADER_T*>(&_rx_buf[0]);

    uint8_t* pPayload =
        reinterpret_cast<uint8_t*>(&_rx_buf[sizeof(Maint::MAINT_HEADER_T)]);

    emit rxRawData(cks_ok, reinterpret_cast<quint8*>(&_rx_buf[0]), data_len);

    logDecodedRxPacket(
        cks_ok,
        rx_cks,
        local_cks,
        reinterpret_cast<const uint8_t*>(&_rx_buf[0]),
        data_len
    );

    if (!cks_ok)
    {
        return;
    }

    float gxf = 0.0f;
    float gyf = 0.0f;

    if (local_cks == rx_cks)
    {
        if (rx_header->Bits.accel_x)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));
            
            emit receivedRawAccelX(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.accel_y)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedRawAccelY(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.accel_z)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedRawAccelZ(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.gyro_x)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedRawGyroX(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.gyro_y)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedRawGyroY(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.gyro_z)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedRawGyroZ(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.magn_x)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedRawMagnX(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.magn_y)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedRawMagnY(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.magn_z)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedRawMagnZ(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.accel_x_f)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedFilteredAccelX(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.accel_y_f)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedFilteredAccelY(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.accel_z_f)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedFilteredAccelZ(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.gyro_x_f)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));
            gxf = fdata;

            emit receivedFilteredGyroX(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.gyro_y_f)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));
            gyf = fdata;

            emit receivedFilteredGyroY(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.gyro_z_f)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedFilteredGyroZ(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.magn_x_f)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedFilteredMagnX(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.magn_y_f)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedFilteredMagnY(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.magn_z_f)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedFilteredMagnZ(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.throttle_sgn)
        {
            uint16_t idata = *(reinterpret_cast<uint16_t*>(pPayload));

            emit receivedThrottleSgn(idata);

            pPayload += sizeof(uint16_t);
        }
        if (rx_header->Bits.roll_sgn)
        {
            uint16_t idata = *(reinterpret_cast<uint16_t*>(pPayload));

            emit receivedRollSgn(idata);

            pPayload += sizeof(uint16_t);
        }
        if (rx_header->Bits.pitch_sgn)
        {
            uint16_t idata = *(reinterpret_cast<uint16_t*>(pPayload));

            emit receivedPitchSgn(idata);

            pPayload += sizeof(uint16_t);
        }
        if (rx_header->Bits.cmd_thr)
        {
            uint16_t idata = *(reinterpret_cast<uint16_t*>(pPayload));
            emit receivedCmdThr(idata);

            pPayload += sizeof(uint16_t);
        }
        if (rx_header->Bits.cmd_roll)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedCmdRoll(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.cmd_pitch)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedCmdPitch(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.body_roll)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedBodyRoll(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.body_pitch)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedBodyPitch(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.body_yaw)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedBodyYaw(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.roll_pid_err)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedRollPidErr(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.roll_pid_p)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedRollPidP(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.roll_pid_i)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedRollPidI(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.roll_pid_d)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedRollPidD(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.roll_pid_u)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedRollPidU(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.pitch_pid_err)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedPitchPidErr(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.pitch_pid_p)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedPitchPidP(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.pitch_pid_i)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedPitchPidI(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.pitch_pid_d)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedPitchPidD(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.pitch_pid_u)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedPitchPidU(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.yaw_pid_err)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedYawPidErr(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.yaw_pid_p)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedYawPidP(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.yaw_pid_i)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedYawPidI(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.yaw_pid_d)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedYawPidD(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.yaw_pid_u)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedYawPidU(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.motor1)
        {
            uint16_t idata = *(reinterpret_cast<uint16_t*>(pPayload));

            emit receivedMotor1(idata);

            pPayload += sizeof(uint16_t);
        }
        if (rx_header->Bits.motor2)
        {
            uint16_t idata = *(reinterpret_cast<uint16_t*>(pPayload));

            emit receivedMotor2(idata);

            pPayload += sizeof(uint16_t);
        }
        if (rx_header->Bits.motor3)
        {
            uint16_t idata = *(reinterpret_cast<uint16_t*>(pPayload));

            emit receivedMotor3(idata);

            pPayload += sizeof(uint16_t);
        }
        if (rx_header->Bits.motor4)
        {
            uint16_t idata = *(reinterpret_cast<uint16_t*>(pPayload));

            emit receivedMotor4(idata);

            pPayload += sizeof(uint16_t);
        }
        if (rx_header->Bits.motors_armed)
        {
            uint8_t idata = *(reinterpret_cast<uint8_t*>(pPayload));

            emit receivedMotorsArmed(idata);

            pPayload += sizeof(uint8_t);
        }
        if (rx_header->Bits.cbit)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));

            emit receivedCbit(idata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.motor_params)
        {
            uint8_t  m1_enabled = *(reinterpret_cast<uint32_t*>(pPayload + 0));
            uint16_t m1_min = *(reinterpret_cast<uint32_t*>(pPayload + 4));
            uint16_t m1_max = *(reinterpret_cast<uint32_t*>(pPayload + 8));
            uint8_t  m2_enabled = *(reinterpret_cast<uint32_t*>(pPayload + 12));
            uint16_t m2_min = *(reinterpret_cast<uint32_t*>(pPayload + 16));
            uint16_t m2_max = *(reinterpret_cast<uint32_t*>(pPayload + 20));
            uint8_t  m3_enabled = *(reinterpret_cast<uint32_t*>(pPayload + 24));
            uint16_t m3_min = *(reinterpret_cast<uint32_t*>(pPayload + 28));
            uint16_t m3_max = *(reinterpret_cast<uint32_t*>(pPayload + 32));
            uint8_t  m4_enabled = *(reinterpret_cast<uint32_t*>(pPayload + 36));
            uint16_t m4_min = *(reinterpret_cast<uint32_t*>(pPayload + 40));
            uint16_t m4_max = *(reinterpret_cast<uint32_t*>(pPayload + 44));

            emit receivedMotorsParams(1, m1_enabled > 0, m1_min, m1_max);
            emit receivedMotorsParams(2, m2_enabled > 0, m2_min, m2_max);
            emit receivedMotorsParams(3, m3_enabled > 0, m3_min, m3_max);
            emit receivedMotorsParams(4, m4_enabled > 0, m4_min, m4_max);

            pPayload += (4 * sizeof(uint8_t) + 8 * sizeof(uint16_t));
        }
        if (rx_header->Bits.js_params)
        {
            float alpha_throttle = *(reinterpret_cast<float*>(pPayload + 0));
            float beta_throttle  = *(reinterpret_cast<float*>(pPayload + 4));
            float alpha_roll = *(reinterpret_cast<float*>(pPayload + 8));
            float beta_roll = *(reinterpret_cast<float*>(pPayload + 12));
            float alpha_pitch = *(reinterpret_cast<float*>(pPayload + 16));
            float beta_pitch = *(reinterpret_cast<float*>(pPayload + 20));


            emit receivedJsParams(1, alpha_throttle, beta_throttle);
            emit receivedJsParams(2, alpha_roll, beta_roll);
            emit receivedJsParams(3, alpha_pitch, beta_pitch);

            pPayload += 6 * sizeof(uint32_t) + 2 * sizeof(uint32_t); // last 2 ignored
        }
        if (rx_header->Bits.pid_params)
        {
            float roll_kp = *(reinterpret_cast<float*>(pPayload + 0));
            float roll_ki = *(reinterpret_cast<float*>(pPayload + 4));
            float roll_kd = *(reinterpret_cast<float*>(pPayload + 8));
            float roll_sat = *(reinterpret_cast<float*>(pPayload + 12));

            float pitch_kp = *(reinterpret_cast<float*>(pPayload + 16));
            float pitch_ki = *(reinterpret_cast<float*>(pPayload + 20));
            float pitch_kd = *(reinterpret_cast<float*>(pPayload + 24));
            float pitch_sat = *(reinterpret_cast<float*>(pPayload + 28));

            float yaw_kp = *(reinterpret_cast<float*>(pPayload + 32));
            float yaw_ki = *(reinterpret_cast<float*>(pPayload + 36));
            float yaw_kd = *(reinterpret_cast<float*>(pPayload + 40));
            float yaw_sat = *(reinterpret_cast<float*>(pPayload + 44));


            emit receivedPidParams(1, roll_kp, roll_ki, roll_kd, roll_sat);
            emit receivedPidParams(2, pitch_kp, pitch_ki, pitch_kd, pitch_sat);
            emit receivedPidParams(3, yaw_kp, yaw_ki, yaw_kd, yaw_sat);

            pPayload += 12 * sizeof(uint32_t);
        }
        if (rx_header->Bits.ptf1_params)
        {
            float acc_x = *(reinterpret_cast<float*>(pPayload + 0));
            float acc_y = *(reinterpret_cast<float*>(pPayload + 4));
            float acc_z = *(reinterpret_cast<float*>(pPayload + 8));

            float gyro_x = *(reinterpret_cast<float*>(pPayload + 12));
            float gyro_y = *(reinterpret_cast<float*>(pPayload + 16));
            float gyro_z = *(reinterpret_cast<float*>(pPayload + 20));

            float magn_x = *(reinterpret_cast<float*>(pPayload + 24));
            float magn_y = *(reinterpret_cast<float*>(pPayload + 28));
            float magn_z = *(reinterpret_cast<float*>(pPayload + 32));

            emit receivedPtf1Params(1, acc_x, acc_y, acc_z);
            emit receivedPtf1Params(2, gyro_x, gyro_y, gyro_z);
            emit receivedPtf1Params(3, magn_x, magn_y, magn_z);

            pPayload += 9 * sizeof(uint32_t);
        }
        if (rx_header->Bits.imu_type)
        {
            uint8_t imu_type = *(reinterpret_cast<uint8_t*>(pPayload));

            emit receivedImuType(imu_type);

            pPayload += sizeof(uint8_t);
        }
        if (rx_header->Bits.i2c_read)
        {
            uint8_t i2c_read = *(reinterpret_cast<uint8_t*>(pPayload));

            emit receivedI2CRead(i2c_read);

            pPayload += sizeof(uint8_t);
        }
        if (rx_header->Bits.sw_ver)
        {
            SW_VER_TAG* sw_ver = (reinterpret_cast<SW_VER_TAG*>(pPayload));

            emit receivedSwVer(sw_ver->major_v, sw_ver->minor_v, sw_ver->stage_v, sw_ver->rel_type);

            pPayload += sizeof(SW_VER_TAG);
        }
        if (rx_header->Bits.imu_offset)
        {
            float offset_roll = *(reinterpret_cast<float*>(pPayload));
            float offset_pitch = *(reinterpret_cast<float*>(pPayload + 4));

            emit receivedImuOffset(offset_roll, offset_pitch);
            pPayload += (2 * sizeof(uint32_t));
        }
        if (rx_header->Bits.throttle_params)
        {
            uint16_t descend = *(reinterpret_cast<uint16_t*>(pPayload));
            uint16_t hovering = *(reinterpret_cast<uint16_t*>(pPayload + 2));
            uint16_t climb = *(reinterpret_cast<uint16_t*>(pPayload + 4));

            emit receivedThrottleParams(descend, hovering, climb);
            pPayload += (3 * sizeof(uint16_t));
        }
        if (rx_header->Bits.gyro_x_f && rx_header->Bits.gyro_y_f)
        {
            emit receivedGyroXYfiltered(gxf, gyf);
        }

    }
}


uint32_t Maint::Maintenance::calc_exp_bytes(Maint::MAINT_HEADER_T* header)
{
    uint32_t rx_payload_idx = 0;

    if (header->Bits.accel_x)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.accel_y)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.accel_z)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.gyro_x)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.gyro_y)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.gyro_z)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.magn_x)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.magn_y)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.magn_z)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.accel_x_f)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.accel_y_f)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.accel_z_f)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.gyro_x_f)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.gyro_y_f)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.gyro_z_f)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.magn_x_f)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.magn_y_f)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.magn_z_f)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.throttle_sgn)
    {
        rx_payload_idx += sizeof(uint16_t);
    }
    if (header->Bits.roll_sgn)
    {
        rx_payload_idx += sizeof(uint16_t);
    }
    if (header->Bits.pitch_sgn)
    {
        rx_payload_idx += sizeof(uint16_t);
    }
    if (header->Bits.cmd_thr)
    {
        rx_payload_idx += sizeof(uint16_t);
    }
    if (header->Bits.cmd_roll)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.cmd_pitch)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.body_roll)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.body_pitch)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.body_yaw)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.roll_pid_err)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.roll_pid_p)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.roll_pid_i)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.roll_pid_d)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.roll_pid_u)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.pitch_pid_err)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.pitch_pid_p)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.pitch_pid_i)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.pitch_pid_d)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.pitch_pid_u)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.yaw_pid_err)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.yaw_pid_p)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.yaw_pid_i)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.yaw_pid_d)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.yaw_pid_u)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.motor1)
    {
        rx_payload_idx += sizeof(uint16_t);
    }
    if (header->Bits.motor2)
    {
        rx_payload_idx += sizeof(uint16_t);
    }
    if (header->Bits.motor3)
    {
        rx_payload_idx += sizeof(uint16_t);
    }
    if (header->Bits.motor4)
    {
        rx_payload_idx += sizeof(uint16_t);
    }
    if (header->Bits.motors_armed)
    {
        rx_payload_idx += sizeof(uint8_t);
    }
    if (header->Bits.cbit)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.motor_params)
    {
        rx_payload_idx += FLASH_MOTORS_PARAMS_SIZE;
    }
    if (header->Bits.js_params)
    {
        rx_payload_idx += FLASH_JOYSTICK_PARAMS_SIZE;
    }
    if (header->Bits.pid_params)
    {
        rx_payload_idx += FLASH_PID_PARAMS_SIZE;
    }
    if (header->Bits.ptf1_params)
    {
        rx_payload_idx += FLASH_PTF1_PARAMS_SIZE;
    }
    if (header->Bits.imu_type)
    {
        rx_payload_idx += sizeof(uint8_t);
    }
    if (header->Bits.i2c_read)
    {
        rx_payload_idx += sizeof(uint8_t);
    }
    if (header->Bits.sw_ver)
    {
        rx_payload_idx += sizeof(uint32_t);
    }
    if (header->Bits.imu_offset)
    {
        rx_payload_idx += (2 * sizeof(uint32_t));
    }
    if (header->Bits.throttle_params)
    {
        rx_payload_idx += FLASH_THROTTLE_PARAMS_SIZE;
    }

    return rx_payload_idx + 1;
}


void Maint::Maintenance::logBytes(quint8* data, int size)
{
    QString timestamp = QDateTime::currentDateTime().toString();
    QString logLine = QString("[%1] ").arg(timestamp);

    for (int i = 0; i < size; i++)
    {
        logLine.append(QString("0x%1 ").arg(QString::number(data[i], 16).toUpper()));
    }
    logLine.append("\n");


    fprintf(_logFile, logLine.toStdString().c_str());
}


void Maint::Maintenance::onDownlinkTimeout()
{
    _checkDownlink->stop();

    emit downlink();
}


void Maint::Maintenance::logDecodedRxPacket(bool cksOk,
    uint8_t rxCks,
    uint8_t localCks,
    const uint8_t* packet,
    uint32_t packetLen)
{
    if (_logFile == nullptr || packet == nullptr)
    {
        return;
    }

    QString log;

    log += "\n";
    log += "============================================================\n";
    log += QString("[%1] RX MAINT PACKET\n")
        .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz"));

    log += QString("LEN          : %1 bytes\n").arg(packetLen);
    log += QString("CHECKSUM     : %1  rx=%2 local=%3\n")
        .arg(cksOk ? "OK" : "FAIL")
        .arg(hexU8(rxCks))
        .arg(hexU8(localCks));

    log += QString("RAW          : %1\n").arg(byteDump(packet, packetLen));

    if (packetLen < sizeof(MAINT_HEADER_T) + 1)
    {
        log += "ERROR        : packet too short\n";
        log += "============================================================\n";

        const QByteArray out = log.toUtf8();
        fprintf(_logFile, "%s", out.constData());
        fflush(_logFile);
        return;
    }

    const uint8_t* headerPtr = packet;
    const uint8_t* payloadPtr = packet + sizeof(MAINT_HEADER_T);
    const uint8_t* payloadEnd = packet + packetLen - 1; // checksum escluso

    const uint64_t header = readLeU64(headerPtr);
    const uint64_t payloadMask = header & ((1ULL << 57) - 1ULL);
    const uint8_t maintCmdId = static_cast<uint8_t>((header >> 57) & 0x7FULL);

    log += QString("HEADER       : %1\n").arg(hexU64(header));
    log += QString("PAYLOAD MASK : %1\n").arg(hexU64(payloadMask));
    log += QString("MAINT CMD ID : %1 %2\n")
        .arg(static_cast<uint32_t>(maintCmdId))
        .arg(hexU8(maintCmdId));

    log += "SET FIELDS   : ";

    bool first = true;

    for (int bit = 0; bit <= 56; ++bit)
    {
        if ((payloadMask & (1ULL << bit)) != 0)
        {
            if (!first)
            {
                log += ", ";
            }

            log += bitName(bit);
            first = false;
        }
    }

    if (first)
    {
        log += "none";
    }

    log += "\n";

    log += QString("PAYLOAD LEN  : %1 bytes\n")
        .arg(static_cast<uint32_t>(payloadEnd - payloadPtr));

    log += "PAYLOAD\n";

    const uint8_t* p = payloadPtr;

    auto offset = [&]() -> uint32_t
        {
            return static_cast<uint32_t>(p - payloadPtr);
        };

    auto has = [&](int bit) -> bool
        {
            return (payloadMask & (1ULL << bit)) != 0;
        };

    auto ensure = [&](uint32_t n, const char* fieldName) -> bool
        {
            if (p + n <= payloadEnd)
            {
                return true;
            }

            log += QString("  %1 @ +%2 : TRUNCATED, need %3 byte, remaining %4\n")
                .arg(QString::fromLatin1(fieldName), -28)
                .arg(offset())
                .arg(n)
                .arg(static_cast<uint32_t>(payloadEnd - p));

            return false;
        };

    auto appendF32 = [&](const char* name) -> float
        {
            if (!ensure(sizeof(uint32_t), name))
            {
                return 0.0f;
            }

            const uint32_t raw = readLeU32(p);
            const float value = readLeF32(p);

            log += QString("  %1 @ +%2 : %3  raw=%4\n")
                .arg(QString::fromLatin1(name), -28)
                .arg(offset(), 3)
                .arg(QString::number(value, 'g', 9))
                .arg(hexU32(raw));

            p += sizeof(uint32_t);
            return value;
        };

    auto appendU32 = [&](const char* name) -> uint32_t
        {
            if (!ensure(sizeof(uint32_t), name))
            {
                return 0;
            }

            const uint32_t value = readLeU32(p);

            log += QString("  %1 @ +%2 : %3  %4\n")
                .arg(QString::fromLatin1(name), -28)
                .arg(offset(), 3)
                .arg(value)
                .arg(hexU32(value));

            p += sizeof(uint32_t);
            return value;
        };

    auto appendU16 = [&](const char* name) -> uint16_t
        {
            if (!ensure(sizeof(uint16_t), name))
            {
                return 0;
            }

            const uint16_t value = readLeU16(p);

            log += QString("  %1 @ +%2 : %3  %4\n")
                .arg(QString::fromLatin1(name), -28)
                .arg(offset(), 3)
                .arg(static_cast<uint32_t>(value))
                .arg(hexU16(value));

            p += sizeof(uint16_t);
            return value;
        };

    auto appendU8 = [&](const char* name) -> uint8_t
        {
            if (!ensure(sizeof(uint8_t), name))
            {
                return 0;
            }

            const uint8_t value = *p;

            log += QString("  %1 @ +%2 : %3  %4\n")
                .arg(QString::fromLatin1(name), -28)
                .arg(offset(), 3)
                .arg(static_cast<uint32_t>(value))
                .arg(hexU8(value));

            p += sizeof(uint8_t);
            return value;
        };

    if (has(0))  appendF32("accel_x");
    if (has(1))  appendF32("accel_y");
    if (has(2))  appendF32("accel_z");

    if (has(3))  appendF32("gyro_x");
    if (has(4))  appendF32("gyro_y");
    if (has(5))  appendF32("gyro_z");

    if (has(6))  appendF32("magn_x");
    if (has(7))  appendF32("magn_y");
    if (has(8))  appendF32("magn_z");

    if (has(9))  appendF32("accel_x_f");
    if (has(10)) appendF32("accel_y_f");
    if (has(11)) appendF32("accel_z_f");

    if (has(12)) appendF32("gyro_x_f");
    if (has(13)) appendF32("gyro_y_f");
    if (has(14)) appendF32("gyro_z_f");

    if (has(15)) appendF32("magn_x_f");
    if (has(16)) appendF32("magn_y_f");
    if (has(17)) appendF32("magn_z_f");

    if (has(18)) appendU16("throttle_sgn");
    if (has(19)) appendU16("roll_sgn");
    if (has(20)) appendU16("pitch_sgn");

    if (has(21)) appendU16("cmd_thr");
    if (has(22)) appendF32("cmd_roll");
    if (has(23)) appendF32("cmd_pitch");

    if (has(24)) appendF32("body_roll");
    if (has(25)) appendF32("body_pitch");
    if (has(26)) appendF32("body_yaw");

    if (has(27)) appendF32("roll_pid_err");
    if (has(28)) appendF32("roll_pid_p");
    if (has(29)) appendF32("roll_pid_i");
    if (has(30)) appendF32("roll_pid_d");
    if (has(31)) appendF32("roll_pid_u");

    if (has(32)) appendF32("pitch_pid_err");
    if (has(33)) appendF32("pitch_pid_p");
    if (has(34)) appendF32("pitch_pid_i");
    if (has(35)) appendF32("pitch_pid_d");
    if (has(36)) appendF32("pitch_pid_u");

    if (has(37)) appendF32("yaw_pid_err");
    if (has(38)) appendF32("yaw_pid_p");
    if (has(39)) appendF32("yaw_pid_i");
    if (has(40)) appendF32("yaw_pid_d");
    if (has(41)) appendF32("yaw_pid_u");

    if (has(42)) appendU16("motor1");
    if (has(43)) appendU16("motor2");
    if (has(44)) appendU16("motor3");
    if (has(45)) appendU16("motor4");

    if (has(46)) appendU8("motors_armed");

    if (has(47)) appendU32("cbit");

    if (has(48))
    {
        log += "  motor_params\n";

        appendU32("motor_params.m1_enabled");
        appendU32("motor_params.m1_min");
        appendU32("motor_params.m1_max");

        appendU32("motor_params.m2_enabled");
        appendU32("motor_params.m2_min");
        appendU32("motor_params.m2_max");

        appendU32("motor_params.m3_enabled");
        appendU32("motor_params.m3_min");
        appendU32("motor_params.m3_max");

        appendU32("motor_params.m4_enabled");
        appendU32("motor_params.m4_min");
        appendU32("motor_params.m4_max");
    }

    if (has(49))
    {
        log += "  js_params\n";

        appendF32("js_params.alpha_throttle");
        appendF32("js_params.beta_throttle");
        appendF32("js_params.alpha_roll");
        appendF32("js_params.beta_roll");
        appendF32("js_params.alpha_pitch");
        appendF32("js_params.beta_pitch");

        appendU32("js_params.ignored_0");
        appendU32("js_params.ignored_1");
    }

    if (has(50))
    {
        log += "  pid_params\n";

        appendF32("pid_params.roll_kp");
        appendF32("pid_params.roll_ki");
        appendF32("pid_params.roll_kd");
        appendF32("pid_params.roll_sat");

        appendF32("pid_params.pitch_kp");
        appendF32("pid_params.pitch_ki");
        appendF32("pid_params.pitch_kd");
        appendF32("pid_params.pitch_sat");

        appendF32("pid_params.yaw_kp");
        appendF32("pid_params.yaw_ki");
        appendF32("pid_params.yaw_kd");
        appendF32("pid_params.yaw_sat");
    }

    if (has(51))
    {
        log += "  ptf1_params\n";

        appendF32("ptf1_params.acc_x");
        appendF32("ptf1_params.acc_y");
        appendF32("ptf1_params.acc_z");

        appendF32("ptf1_params.gyro_x");
        appendF32("ptf1_params.gyro_y");
        appendF32("ptf1_params.gyro_z");

        appendF32("ptf1_params.magn_x");
        appendF32("ptf1_params.magn_y");
        appendF32("ptf1_params.magn_z");
    }

    if (has(52)) appendU8("imu_type");
    if (has(53)) appendU8("i2c_read");

    if (has(54))
    {
        if (ensure(sizeof(SW_VER_TAG), "sw_ver"))
        {
            SW_VER_TAG swVer;
            std::memcpy(&swVer, p, sizeof(SW_VER_TAG));

            log += QString("  %1 @ +%2 : major=%3 minor=%4 stage=%5 rel_type=%6\n")
                .arg("sw_ver", -28)
                .arg(offset(), 3)
                .arg(static_cast<uint32_t>(swVer.major_v))
                .arg(static_cast<uint32_t>(swVer.minor_v))
                .arg(static_cast<uint32_t>(swVer.stage_v))
                .arg(static_cast<uint32_t>(swVer.rel_type));

            p += sizeof(SW_VER_TAG);
        }
    }

    if (has(55))
    {
        appendF32("imu_offset.roll");
        appendF32("imu_offset.pitch");
    }

    if (has(56))
    {
        appendU16("throttle_params.descend");
        appendU16("throttle_params.hovering");
        appendU16("throttle_params.climb");
    }

    const int32_t consumed = static_cast<int32_t>(p - payloadPtr);
    const int32_t expected = static_cast<int32_t>(payloadEnd - payloadPtr);

    log += QString("PAYLOAD USED : %1 / %2 bytes\n")
        .arg(consumed)
        .arg(expected);

    if (consumed != expected)
    {
        log += QString("WARNING      : payload decode size mismatch, remaining=%1 bytes\n")
            .arg(expected - consumed);

        if (p < payloadEnd)
        {
            log += QString("REMAINING    : %1\n")
                .arg(byteDump(p, static_cast<uint32_t>(payloadEnd - p)));
        }
    }

    log += "============================================================\n";

    const QByteArray out = log.toUtf8();
    fprintf(_logFile, "%s", out.constData());
    fflush(_logFile);
}