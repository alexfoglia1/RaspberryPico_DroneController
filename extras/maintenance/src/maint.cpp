#include "maint.h"

#include <qdatetime.h>
#include <qdir.h>
#include <qdiriterator.h>
#include <qfile.h>

Maint::Maintenance::Maintenance()
{
    _txHeader.All = 0;
    _txCommand.All = 0;

    _serialPort = new QSerialPort();
    _txTimer = new QTimer();

    _status = Maint::MAINT_STATUS::WAIT_SYNC;
    _txStatus = Maint::TX_STATUS::TX_GET;

    _expected_bytes = 0;
    _rx_payload_idx = 0;
    memset(&_rx_buf, 0x00, 1024);

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


void Maint::Maintenance::SetTxHeader(MAINT_HEADER_T txHeader)
{
    _txHeader.All = txHeader.All;
}


void Maint::Maintenance::TxSetMotors(uint32_t motorNo, uint16_t data)
{
    _txCommand.All = 0;
    _txCommand.Bits.maint_cmd_id =  (motorNo == 1) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M1) :
                                    (motorNo == 2) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M2) :
                                    (motorNo == 3) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M3) :
                                    (motorNo == 4) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M4) :
                                    (motorNo == uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_MALL)) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_MALL) :
                                    (motorNo == uint64_t(MAINT_CMD_ID::MAINT_CMD_CTRL_MOTORS)) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_CTRL_MOTORS) : uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE);
    
    if (_txCommand.Bits.maint_cmd_id != uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE))
    {
        _txSetParams.clear();
        pushParams(reinterpret_cast<uint8_t*>(&data), sizeof(uint16_t));

        _txStatus = Maint::TX_STATUS::TX_SET;
    }
}


void Maint::Maintenance::TxMotorParams(uint32_t motorNo, uint8_t enabled, uint16_t minSignalParam, uint16_t maxSignalParam)
{
    _txCommand.All = 0;
    _txCommand.Bits.maint_cmd_id =  (motorNo == 1) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M1_PARAMS) :
                                    (motorNo == 2) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M2_PARAMS) :
                                    (motorNo == 3) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M3_PARAMS) :
                                    (motorNo == 4) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M4_PARAMS) : uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE);

    if (_txCommand.Bits.maint_cmd_id != uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE))
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
    _txCommand.All = 0;
    _txCommand.Bits.maint_cmd_id = (jsChannel == 1) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_JS_THROTTLE_ALPHA_BETA) :
        (jsChannel == 2) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_JS_ROLL_ALPHA_BETA) :
        (jsChannel == 3) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_JS_PITCH_ALPHA_BETA) : uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE);

    if (_txCommand.Bits.maint_cmd_id != uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE))
    {
        _txSetParams.clear();
        pushParams(reinterpret_cast<uint8_t*>(&alpha), sizeof(float));
        pushParams(reinterpret_cast<uint8_t*>(&beta), sizeof(float));

        _txStatus = Maint::TX_STATUS::TX_SET;
    }
}


void Maint::Maintenance::TxPidParams(uint32_t eulerAngle, float kp, float ki, float kt, float sat, float ad, float bd)
{
    _txCommand.All = 0;
    _txCommand.Bits.maint_cmd_id = (eulerAngle == 1) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_ROLL_PID_PARAMS) :
        (eulerAngle == 2) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_PITCH_PID_PARAMS) :
        (eulerAngle == 3) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_YAW_PID_PARAMS) : uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE);

    if (_txCommand.Bits.maint_cmd_id != uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE))
    {
        _txSetParams.clear();
        pushParams(reinterpret_cast<uint8_t*>(&kp), sizeof(float));
        pushParams(reinterpret_cast<uint8_t*>(&ki), sizeof(float));
        pushParams(reinterpret_cast<uint8_t*>(&kt), sizeof(float));
        pushParams(reinterpret_cast<uint8_t*>(&sat), sizeof(float));
        pushParams(reinterpret_cast<uint8_t*>(&ad), sizeof(float));
        pushParams(reinterpret_cast<uint8_t*>(&bd), sizeof(float));
        
        _txStatus = Maint::TX_STATUS::TX_SET;
        
    }
}


void Maint::Maintenance::TxPtf1params(uint32_t sensorSource, float x, float y, float z)
{
    _txCommand.All = 0;
    _txCommand.Bits.maint_cmd_id = (sensorSource == 1) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_PTF1_ACC_PARAMS) :
        (sensorSource == 2) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_PTF1_GYRO_PARAMS) :
        (sensorSource == 3) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_PTF1_MAGN_PARAMS) : uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE);

    if (_txCommand.Bits.maint_cmd_id != uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE))
    {
        _txSetParams.clear();
        pushParams(reinterpret_cast<uint8_t*>(&x), sizeof(float));
        pushParams(reinterpret_cast<uint8_t*>(&y), sizeof(float));
        pushParams(reinterpret_cast<uint8_t*>(&z), sizeof(float));
        
        _txStatus = Maint::TX_STATUS::TX_SET;
        
    }
}


void Maint::Maintenance::TxImuType(IMU_TYPE imuType)
{
    _txCommand.All = 0;
    _txCommand.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_IMU_TYPE);

    uint32_t iImuType = static_cast<uint32_t>(imuType);

    _txSetParams.clear();
    pushParams(reinterpret_cast<uint8_t*>(&iImuType), sizeof(uint32_t));
    
    _txStatus = Maint::TX_STATUS::TX_SET;
    
}


void Maint::Maintenance::I2CRead(uint8_t i2c, uint8_t addr, uint8_t reg)
{
    _txCommand.All = 0;
    _txCommand.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_I2C_READ);

    _txSetParams.clear();
    pushParams(reinterpret_cast<uint8_t*>(&i2c), sizeof(uint8_t));
    pushParams(reinterpret_cast<uint8_t*>(&addr), sizeof(uint8_t));
    pushParams(reinterpret_cast<uint8_t*>(&reg), sizeof(uint8_t));

    
    _txStatus = Maint::TX_STATUS::TX_SET;
    
}


void Maint::Maintenance::I2CWrite(uint8_t i2c, uint8_t addr, uint8_t reg, uint8_t val)
{
    _txCommand.All = 0;
    _txCommand.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_I2C_WRITE);

    _txSetParams.clear();
    pushParams(reinterpret_cast<uint8_t*>(&i2c), sizeof(uint8_t));
    pushParams(reinterpret_cast<uint8_t*>(&addr), sizeof(uint8_t));
    pushParams(reinterpret_cast<uint8_t*>(&reg), sizeof(uint8_t));
    pushParams(reinterpret_cast<uint8_t*>(&val), sizeof(uint8_t));
    
    _txStatus = Maint::TX_STATUS::TX_SET;
    
}


void Maint::Maintenance::TxWriteToFlash()
{
    _txCommand.All = 0;
    _txCommand.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_FLASH_WRITE);

    _txSetParams.clear();

    _txStatus = Maint::TX_STATUS::TX_SET;
}


void Maint::Maintenance::TxImuOffset(float roll_offset, float pitch_offset)
{
    _txCommand.All = 0;
    _txCommand.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_IMU_OFFSET);

    _txSetParams.clear();
    pushParams(reinterpret_cast<uint8_t*>(&roll_offset), sizeof(float));
    pushParams(reinterpret_cast<uint8_t*>(&pitch_offset), sizeof(float));
    
    _txStatus = Maint::TX_STATUS::TX_SET;

}


QByteArray Maint::Maintenance::txSet(Maint::MAINT_HEADER_T* header)
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


QByteArray Maint::Maintenance::txGet(Maint::MAINT_HEADER_T* header)
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
        qba = txGet(&_txHeader);
    }
    else
    {
        qba = txSet(&_txCommand);
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
    Maint::MAINT_HEADER_T* rx_header = reinterpret_cast<Maint::MAINT_HEADER_T*>(&_rx_buf[0]);
    uint8_t* pPayload = reinterpret_cast<uint8_t*>(&_rx_buf[sizeof(Maint::MAINT_HEADER_T)]);

    emit rxRawData(local_cks == rx_cks, reinterpret_cast<quint8*>(&_rx_buf[0]), data_len);

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

            emit receivedFilteredGyroX(fdata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.gyro_y_f)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

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
            float roll_kt = *(reinterpret_cast<float*>(pPayload + 8));
            float roll_sat = *(reinterpret_cast<float*>(pPayload + 12));
            float roll_ad = *(reinterpret_cast<float*>(pPayload + 16));
            float roll_bd = *(reinterpret_cast<float*>(pPayload + 20));

            float pitch_kp = *(reinterpret_cast<float*>(pPayload + 24));
            float pitch_ki = *(reinterpret_cast<float*>(pPayload + 28));
            float pitch_kt = *(reinterpret_cast<float*>(pPayload + 32));
            float pitch_sat = *(reinterpret_cast<float*>(pPayload + 36));
            float pitch_ad = *(reinterpret_cast<float*>(pPayload + 40));
            float pitch_bd = *(reinterpret_cast<float*>(pPayload + 44));

            float yaw_kp = *(reinterpret_cast<float*>(pPayload + 48));
            float yaw_ki = *(reinterpret_cast<float*>(pPayload + 52));
            float yaw_kt = *(reinterpret_cast<float*>(pPayload + 56));
            float yaw_sat = *(reinterpret_cast<float*>(pPayload + 60));
            float yaw_ad = *(reinterpret_cast<float*>(pPayload + 64));
            float yaw_bd = *(reinterpret_cast<float*>(pPayload + 68));

            emit receivedPidParams(1, roll_kp, roll_ki, roll_kt, roll_sat, roll_ad, roll_bd);
            emit receivedPidParams(2, pitch_kp, pitch_ki, pitch_kt, pitch_sat, pitch_ad, pitch_bd);
            emit receivedPidParams(3, yaw_kp, yaw_ki, yaw_kt, yaw_sat, yaw_ad, yaw_bd);

            pPayload += 18 * sizeof(uint32_t);
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
