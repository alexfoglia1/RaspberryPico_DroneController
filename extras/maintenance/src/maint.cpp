#include "maint.h"

#include <qdatetime.h>


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

    _tx_data = 0;
    _tx_param_enabled = 0;
    _tx_param_min_signal = 0;
    _tx_param_max_signal = 0;
    _tx_param_js_alpha = 0;
    _tx_param_js_beta = 0;
    _tx_param_kp = 0;
    _tx_param_ki = 0;
    _tx_param_kt = 0;
    _tx_param_sat = 0;
    _tx_param_ad = 0;
    _tx_param_bd = 0;
    _tx_param_x = 0;
    _tx_param_y = 0;
    _tx_param_z = 0;

}

bool Maint::Maintenance::Open(QString serialPortName, enum QSerialPort::BaudRate baud)
{
    _serialPort->setPortName(serialPortName);
	_serialPort->setBaudRate(baud);
	_serialPort->setParity(QSerialPort::NoParity);
	_serialPort->setDataBits(QSerialPort::Data8);
	_serialPort->setStopBits(QSerialPort::OneStop);
	_serialPort->setFlowControl(QSerialPort::NoFlowControl);

    _logFileName = QString("%1-%2.txt").arg(QDateTime::currentDateTime().toString().replace(" ", "-").replace(":","-")).arg(serialPortName);
    _logFile = fopen(_logFileName.toStdString().c_str(), "w");

    fclose(_logFile);

	connect(_serialPort, SIGNAL(readyRead()), this, SLOT(OnRx()));
    connect(this, SIGNAL(rxBytes(quint8*, int)), this, SLOT(logBytes(quint8*, int)));

	return _serialPort->open(QSerialPort::OpenModeFlag::ReadWrite);
}


void Maint::Maintenance::Close()
{
    _txTimer->stop();
    _txTimer->disconnect(_txTimer, SIGNAL(timeout()), this, SLOT(Tx()));

    disconnect(this, SIGNAL(rxBytes(quint8*, int)), this, SLOT(logBytes(quint8*, int)));

    _serialPort->disconnect(_serialPort, SIGNAL(readyRead()), this, SLOT(OnRx()));
    _serialPort->close();
}


void Maint::Maintenance::EnableTx()
{
    _txTimer->setInterval(10);
    _txTimer->setSingleShot(false);
    _txTimer->setTimerType(Qt::PreciseTimer);

    connect(_txTimer, SIGNAL(timeout()), this, SLOT(Tx()));
    _txTimer->start();
}


void Maint::Maintenance::SetTxHeader(MAINT_HEADER_T txHeader)
{
    _txMutex.lock();

    _txHeader.All = txHeader.All;

    _txMutex.unlock();
}


void Maint::Maintenance::TxMaintenanceCommand(uint32_t motorNo, uint32_t data)
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
        _tx_data = data;

        _txMutex.lock();
        _txStatus = Maint::TX_STATUS::TX_SET_CMD;
        _txMutex.unlock();
    }
}


void Maint::Maintenance::TxMaintenanceParams(uint32_t motorNo, bool enabled, uint32_t minSignalParam, uint32_t maxSignalParam)
{
    _txCommand.All = 0;
    _txCommand.Bits.maint_cmd_id =  (motorNo == 1) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M1_PARAMS) :
                                    (motorNo == 2) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M2_PARAMS) :
                                    (motorNo == 3) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M3_PARAMS) :
                                    (motorNo == 4) ? uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_M4_PARAMS) : uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE);

    if (_txCommand.Bits.maint_cmd_id != uint64_t(MAINT_CMD_ID::MAINT_CMD_NONE))
    {
        _tx_param_enabled = (enabled) ? 1 : 0;
        _tx_param_min_signal = minSignalParam;
        _tx_param_max_signal = maxSignalParam;

        _txMutex.lock();
        _txStatus = Maint::TX_STATUS::TX_SET_PARAMS;
        _txMutex.unlock();
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
        _tx_param_js_alpha = *reinterpret_cast<uint32_t*>(&alpha);
        _tx_param_js_beta = *reinterpret_cast<uint32_t*>(&beta);

        _txMutex.lock();
        _txStatus = Maint::TX_STATUS::TX_SET_JS_PARAMS;
        _txMutex.unlock();
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
        _tx_param_kp = *reinterpret_cast<uint32_t*>(&kp);
        _tx_param_ki = *reinterpret_cast<uint32_t*>(&ki);
        _tx_param_kt = *reinterpret_cast<uint32_t*>(&kt);
        _tx_param_sat = *reinterpret_cast<uint32_t*>(&sat);
        _tx_param_ad = *reinterpret_cast<uint32_t*>(&ad);
        _tx_param_bd = *reinterpret_cast<uint32_t*>(&bd);

        _txMutex.lock();
        _txStatus = Maint::TX_STATUS::TX_SET_PID_PARAMS;
        _txMutex.unlock();
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
        _tx_param_x = *reinterpret_cast<uint32_t*>(&x);
        _tx_param_y = *reinterpret_cast<uint32_t*>(&y);
        _tx_param_z = *reinterpret_cast<uint32_t*>(&z);

        _txMutex.lock();
        _txStatus = Maint::TX_STATUS::TX_SET_PTF1_PARAMS;
        _txMutex.unlock();
    }
}


void Maint::Maintenance::TxImuType(IMU_TYPE imuType)
{
    _txCommand.All = 0;
    _txCommand.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_SET_IMU_TYPE);

    _tx_param_imu = uint32_t(imuType);

    _txMutex.lock();
    _txStatus = Maint::TX_STATUS::TX_SET_IMU_TYPE;
    _txMutex.unlock();
}


void Maint::Maintenance::I2CRead(uint32_t i2c, uint32_t addr, uint32_t reg)
{
    _txCommand.All = 0;
    _txCommand.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_I2C_READ);

    _tx_param_i2c_chan = uint32_t(i2c);
    _tx_param_i2c_addr = uint32_t(addr);
    _tx_param_i2c_reg  = uint32_t(reg);

    _txMutex.lock();
    _txStatus = Maint::TX_STATUS::TX_I2C_READ;
    _txMutex.unlock();
}


void Maint::Maintenance::I2CWrite(uint32_t i2c, uint32_t addr, uint32_t reg, uint32_t val)
{
    _txCommand.All = 0;
    _txCommand.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_I2C_WRITE);

    _tx_param_i2c_chan = uint32_t(i2c);
    _tx_param_i2c_addr = uint32_t(addr);
    _tx_param_i2c_reg  = uint32_t(reg);
    _tx_param_i2c_val  = uint32_t(val);

    _txMutex.lock();
    _txStatus = Maint::TX_STATUS::TX_I2C_WRITE;
    _txMutex.unlock();
}


void Maint::Maintenance::TxWriteToFlash()
{
    _txCommand.All = 0;
    _txCommand.Bits.maint_cmd_id = uint64_t(MAINT_CMD_ID::MAINT_CMD_FLASH_WRITE);

    _txMutex.lock();
    _txStatus = Maint::TX_STATUS::TX_WRITE_TO_FLASH;
    _txMutex.unlock();

}


void Maint::Maintenance::Tx()
{
    _txMutex.lock();

    QByteArray qba;
    int bytesWritten = 0;

    if (_txStatus == Maint::TX_STATUS::TX_GET)
    {
        uint8_t cks = Maint::checksum(reinterpret_cast<uint8_t*>(&_txHeader), sizeof(Maint::MAINT_HEADER_T), false);

        qba.push_back(Maint::SYNC_CHAR);
        qba.push_back(_txHeader.Bytes[0]);
        qba.push_back(_txHeader.Bytes[1]);
        qba.push_back(_txHeader.Bytes[2]);
        qba.push_back(_txHeader.Bytes[3]);
        qba.push_back(_txHeader.Bytes[4]);
        qba.push_back(_txHeader.Bytes[5]);
        qba.push_back(_txHeader.Bytes[6]);
        qba.push_back(_txHeader.Bytes[7]);

        _txMutex.unlock();

        qba.push_back(cks);
        bytesWritten = _serialPort->write(qba);
        _serialPort->flush();
    }
    else if (_txStatus == Maint::TX_STATUS::TX_SET_CMD)
    {
        qba.push_back(Maint::SYNC_CHAR);
        qba.push_back(_txCommand.Bytes[0]);
        qba.push_back(_txCommand.Bytes[1]);
        qba.push_back(_txCommand.Bytes[2]);
        qba.push_back(_txCommand.Bytes[3]);
        qba.push_back(_txCommand.Bytes[4]);
        qba.push_back(_txCommand.Bytes[5]);
        qba.push_back(_txCommand.Bytes[6]);
        qba.push_back(_txCommand.Bytes[7]);

        uint8_t* dataBytes = reinterpret_cast<uint8_t*>(&_tx_data);
        qba.push_back(dataBytes[0]);
        qba.push_back(dataBytes[1]);
        qba.push_back(dataBytes[2]);
        qba.push_back(dataBytes[3]);

        uint8_t cks = Maint::checksum(reinterpret_cast<uint8_t*>(qba.data()), qba.size(), true);

        _txMutex.unlock();

        qba.push_back(cks);
        bytesWritten = _serialPort->write(qba);
        _serialPort->flush();

        _tx_data = 0;
        _txCommand.All = 0;
        _txStatus = Maint::TX_STATUS::TX_GET;
    }
    else if (_txStatus == Maint::TX_STATUS::TX_SET_PARAMS)
    {
        qba.push_back(Maint::SYNC_CHAR);
        qba.push_back(_txCommand.Bytes[0]);
        qba.push_back(_txCommand.Bytes[1]);
        qba.push_back(_txCommand.Bytes[2]);
        qba.push_back(_txCommand.Bytes[3]);
        qba.push_back(_txCommand.Bytes[4]);
        qba.push_back(_txCommand.Bytes[5]);
        qba.push_back(_txCommand.Bytes[6]);
        qba.push_back(_txCommand.Bytes[7]);

        uint8_t* enabledParamBytes = reinterpret_cast<uint8_t*>(&_tx_param_enabled);
        qba.push_back(enabledParamBytes[0]);
        qba.push_back(enabledParamBytes[1]);
        qba.push_back(enabledParamBytes[2]);
        qba.push_back(enabledParamBytes[3]);

        uint8_t* minSignalParamBytes = reinterpret_cast<uint8_t*>(&_tx_param_min_signal);
        qba.push_back(minSignalParamBytes[0]);
        qba.push_back(minSignalParamBytes[1]);
        qba.push_back(minSignalParamBytes[2]);
        qba.push_back(minSignalParamBytes[3]);

        uint8_t* maxSignalParamBytes = reinterpret_cast<uint8_t*>(&_tx_param_max_signal);
        qba.push_back(maxSignalParamBytes[0]);
        qba.push_back(maxSignalParamBytes[1]);
        qba.push_back(maxSignalParamBytes[2]);
        qba.push_back(maxSignalParamBytes[3]);

        uint8_t cks = Maint::checksum(reinterpret_cast<uint8_t*>(qba.data()), qba.size(), true);

        _txMutex.unlock();

        qba.push_back(cks);
        bytesWritten = _serialPort->write(qba);
        _serialPort->flush();

        _tx_data = 0;
        _txCommand.All = 0;
        _txStatus = Maint::TX_STATUS::TX_GET;
    }
    else if (_txStatus == Maint::TX_STATUS::TX_SET_JS_PARAMS)
    {
        qba.push_back(Maint::SYNC_CHAR);
        qba.push_back(_txCommand.Bytes[0]);
        qba.push_back(_txCommand.Bytes[1]);
        qba.push_back(_txCommand.Bytes[2]);
        qba.push_back(_txCommand.Bytes[3]);
        qba.push_back(_txCommand.Bytes[4]);
        qba.push_back(_txCommand.Bytes[5]);
        qba.push_back(_txCommand.Bytes[6]);
        qba.push_back(_txCommand.Bytes[7]);

        uint8_t* alphaBytes = reinterpret_cast<uint8_t*>(&_tx_param_js_alpha);
        qba.push_back(alphaBytes[0]);
        qba.push_back(alphaBytes[1]);
        qba.push_back(alphaBytes[2]);
        qba.push_back(alphaBytes[3]);

        uint8_t* betaBytes = reinterpret_cast<uint8_t*>(&_tx_param_js_beta);
        qba.push_back(betaBytes[0]);
        qba.push_back(betaBytes[1]);
        qba.push_back(betaBytes[2]);
        qba.push_back(betaBytes[3]);

        uint8_t cks = Maint::checksum(reinterpret_cast<uint8_t*>(qba.data()), qba.size(), true);

        _txMutex.unlock();

        qba.push_back(cks);
        bytesWritten = _serialPort->write(qba);
        _serialPort->flush();

        _tx_data = 0;
        _txCommand.All = 0;
        _txStatus = Maint::TX_STATUS::TX_GET;
    }
    else if (_txStatus == Maint::TX_STATUS::TX_SET_PID_PARAMS)
    {
        qba.push_back(Maint::SYNC_CHAR);
        qba.push_back(_txCommand.Bytes[0]);
        qba.push_back(_txCommand.Bytes[1]);
        qba.push_back(_txCommand.Bytes[2]);
        qba.push_back(_txCommand.Bytes[3]);
        qba.push_back(_txCommand.Bytes[4]);
        qba.push_back(_txCommand.Bytes[5]);
        qba.push_back(_txCommand.Bytes[6]);
        qba.push_back(_txCommand.Bytes[7]);

        uint8_t* kpBytes = reinterpret_cast<uint8_t*>(&_tx_param_kp);
        qba.push_back(kpBytes[0]);
        qba.push_back(kpBytes[1]);
        qba.push_back(kpBytes[2]);
        qba.push_back(kpBytes[3]);

        uint8_t* kiBytes = reinterpret_cast<uint8_t*>(&_tx_param_ki);
        qba.push_back(kiBytes[0]);
        qba.push_back(kiBytes[1]);
        qba.push_back(kiBytes[2]);
        qba.push_back(kiBytes[3]);

        uint8_t* ktBytes = reinterpret_cast<uint8_t*>(&_tx_param_kt);
        qba.push_back(ktBytes[0]);
        qba.push_back(ktBytes[1]);
        qba.push_back(ktBytes[2]);
        qba.push_back(ktBytes[3]);

        uint8_t* satBytes = reinterpret_cast<uint8_t*>(&_tx_param_sat);
        qba.push_back(satBytes[0]);
        qba.push_back(satBytes[1]);
        qba.push_back(satBytes[2]);
        qba.push_back(satBytes[3]);

        uint8_t* adBytes = reinterpret_cast<uint8_t*>(&_tx_param_ad);
        qba.push_back(adBytes[0]);
        qba.push_back(adBytes[1]);
        qba.push_back(adBytes[2]);
        qba.push_back(adBytes[3]);

        uint8_t* bdBytes = reinterpret_cast<uint8_t*>(&_tx_param_bd);
        qba.push_back(bdBytes[0]);
        qba.push_back(bdBytes[1]);
        qba.push_back(bdBytes[2]);
        qba.push_back(bdBytes[3]);

        uint8_t cks = Maint::checksum(reinterpret_cast<uint8_t*>(qba.data()), qba.size(), true);

        _txMutex.unlock();

        qba.push_back(cks);
        bytesWritten = _serialPort->write(qba);
        _serialPort->flush();

        _tx_data = 0;
        _txCommand.All = 0;
        _txStatus = Maint::TX_STATUS::TX_GET;
    }
    else if (_txStatus == Maint::TX_STATUS::TX_SET_PTF1_PARAMS)
    {
        qba.push_back(Maint::SYNC_CHAR);
        qba.push_back(_txCommand.Bytes[0]);
        qba.push_back(_txCommand.Bytes[1]);
        qba.push_back(_txCommand.Bytes[2]);
        qba.push_back(_txCommand.Bytes[3]);
        qba.push_back(_txCommand.Bytes[4]);
        qba.push_back(_txCommand.Bytes[5]);
        qba.push_back(_txCommand.Bytes[6]);
        qba.push_back(_txCommand.Bytes[7]);

        uint8_t* xBytes = reinterpret_cast<uint8_t*>(&_tx_param_x);
        qba.push_back(xBytes[0]);
        qba.push_back(xBytes[1]);
        qba.push_back(xBytes[2]);
        qba.push_back(xBytes[3]);

        uint8_t* yBytes = reinterpret_cast<uint8_t*>(&_tx_param_y);
        qba.push_back(yBytes[0]);
        qba.push_back(yBytes[1]);
        qba.push_back(yBytes[2]);
        qba.push_back(yBytes[3]);

        uint8_t* zBytes = reinterpret_cast<uint8_t*>(&_tx_param_z);
        qba.push_back(zBytes[0]);
        qba.push_back(zBytes[1]);
        qba.push_back(zBytes[2]);
        qba.push_back(zBytes[3]);

        uint8_t cks = Maint::checksum(reinterpret_cast<uint8_t*>(qba.data()), qba.size(), true);

        _txMutex.unlock();

        qba.push_back(cks);
        bytesWritten = _serialPort->write(qba);
        _serialPort->flush();

        _tx_data = 0;
        _txCommand.All = 0;
        _txStatus = Maint::TX_STATUS::TX_GET;
    }
    else if (_txStatus == Maint::TX_STATUS::TX_SET_IMU_TYPE)
    {
        qba.push_back(Maint::SYNC_CHAR);
        qba.push_back(_txCommand.Bytes[0]);
        qba.push_back(_txCommand.Bytes[1]);
        qba.push_back(_txCommand.Bytes[2]);
        qba.push_back(_txCommand.Bytes[3]);
        qba.push_back(_txCommand.Bytes[4]);
        qba.push_back(_txCommand.Bytes[5]);
        qba.push_back(_txCommand.Bytes[6]);
        qba.push_back(_txCommand.Bytes[7]);

        uint8_t* imuTypeBytes = reinterpret_cast<uint8_t*>(&_tx_param_imu);

        qba.push_back(imuTypeBytes[0]);
        qba.push_back(imuTypeBytes[1]);
        qba.push_back(imuTypeBytes[2]);
        qba.push_back(imuTypeBytes[3]);

        uint8_t cks = Maint::checksum(reinterpret_cast<uint8_t*>(qba.data()), qba.size(), true);

        _txMutex.unlock();

        qba.push_back(cks);
        bytesWritten = _serialPort->write(qba);
        _serialPort->flush();

        _tx_data = 0;
        _txCommand.All = 0;
        _txStatus = Maint::TX_STATUS::TX_GET;

    }
    else if (_txStatus == Maint::TX_STATUS::TX_I2C_READ)
    {
        qba.push_back(Maint::SYNC_CHAR);
        qba.push_back(_txCommand.Bytes[0]);
        qba.push_back(_txCommand.Bytes[1]);
        qba.push_back(_txCommand.Bytes[2]);
        qba.push_back(_txCommand.Bytes[3]);
        qba.push_back(_txCommand.Bytes[4]);
        qba.push_back(_txCommand.Bytes[5]);
        qba.push_back(_txCommand.Bytes[6]);
        qba.push_back(_txCommand.Bytes[7]);

        uint8_t* i2cChannelBytes = reinterpret_cast<uint8_t*>(&_tx_param_i2c_chan);
        uint8_t* i2cAddressBytes = reinterpret_cast<uint8_t*>(&_tx_param_i2c_addr);
        uint8_t* i2cRegisterBytes = reinterpret_cast<uint8_t*>(&_tx_param_i2c_reg);

        qba.push_back(i2cChannelBytes[0]);
        qba.push_back(i2cChannelBytes[1]);
        qba.push_back(i2cChannelBytes[2]);
        qba.push_back(i2cChannelBytes[3]);
        qba.push_back(i2cAddressBytes[0]);
        qba.push_back(i2cAddressBytes[1]);
        qba.push_back(i2cAddressBytes[2]);
        qba.push_back(i2cAddressBytes[3]);
        qba.push_back(i2cRegisterBytes[0]);
        qba.push_back(i2cRegisterBytes[1]);
        qba.push_back(i2cRegisterBytes[2]);
        qba.push_back(i2cRegisterBytes[3]);

        uint8_t cks = Maint::checksum(reinterpret_cast<uint8_t*>(qba.data()), qba.size(), true);

        _txMutex.unlock();

        qba.push_back(cks);
        bytesWritten = _serialPort->write(qba);
        _serialPort->flush();

        _tx_data = 0;
        _txCommand.All = 0;
        _txStatus = Maint::TX_STATUS::TX_GET;
    }
    else if (_txStatus == Maint::TX_STATUS::TX_I2C_WRITE)
    {
        qba.push_back(Maint::SYNC_CHAR);
        qba.push_back(_txCommand.Bytes[0]);
        qba.push_back(_txCommand.Bytes[1]);
        qba.push_back(_txCommand.Bytes[2]);
        qba.push_back(_txCommand.Bytes[3]);
        qba.push_back(_txCommand.Bytes[4]);
        qba.push_back(_txCommand.Bytes[5]);
        qba.push_back(_txCommand.Bytes[6]);
        qba.push_back(_txCommand.Bytes[7]);

        uint8_t* i2cChannelBytes = reinterpret_cast<uint8_t*>(&_tx_param_i2c_chan);
        uint8_t* i2cAddressBytes = reinterpret_cast<uint8_t*>(&_tx_param_i2c_addr);
        uint8_t* i2cRegisterBytes = reinterpret_cast<uint8_t*>(&_tx_param_i2c_reg);
        uint8_t* i2cValBytes = reinterpret_cast<uint8_t*>(&_tx_param_i2c_val);

        qba.push_back(i2cChannelBytes[0]);
        qba.push_back(i2cChannelBytes[1]);
        qba.push_back(i2cChannelBytes[2]);
        qba.push_back(i2cChannelBytes[3]);
        qba.push_back(i2cAddressBytes[0]);
        qba.push_back(i2cAddressBytes[1]);
        qba.push_back(i2cAddressBytes[2]);
        qba.push_back(i2cAddressBytes[3]);
        qba.push_back(i2cRegisterBytes[0]);
        qba.push_back(i2cRegisterBytes[1]);
        qba.push_back(i2cRegisterBytes[2]);
        qba.push_back(i2cRegisterBytes[3]);
        qba.push_back(i2cValBytes[0]);
        qba.push_back(i2cValBytes[1]);
        qba.push_back(i2cValBytes[2]);
        qba.push_back(i2cValBytes[3]);        

        uint8_t cks = Maint::checksum(reinterpret_cast<uint8_t*>(qba.data()), qba.size(), true);

        _txMutex.unlock();

        qba.push_back(cks);
        bytesWritten = _serialPort->write(qba);
        _serialPort->flush();

        _tx_data = 0;
        _txCommand.All = 0;
        _txStatus = Maint::TX_STATUS::TX_GET;
    }
    else // TX_WRITE_TO_FLASH
    {
        qba.push_back(Maint::SYNC_CHAR);
        qba.push_back(_txCommand.Bytes[0]);
        qba.push_back(_txCommand.Bytes[1]);
        qba.push_back(_txCommand.Bytes[2]);
        qba.push_back(_txCommand.Bytes[3]);
        qba.push_back(_txCommand.Bytes[4]);
        qba.push_back(_txCommand.Bytes[5]);
        qba.push_back(_txCommand.Bytes[6]);
        qba.push_back(_txCommand.Bytes[7]);

        uint8_t cks = Maint::checksum(reinterpret_cast<uint8_t*>(qba.data()), qba.size(), true);

        _txMutex.unlock();

        qba.push_back(cks);
        bytesWritten = _serialPort->write(qba);
        _serialPort->flush();

        _tx_data = 0;
        _txCommand.All = 0;
        _txStatus = Maint::TX_STATUS::TX_GET;
    }

    emit txRawData(reinterpret_cast<quint8*>(qba.data()), bytesWritten);
}


void Maint::Maintenance::OnRx()
{
	QByteArray qba = _serialPort->readAll();
	
    emit rxBytes(reinterpret_cast<quint8*>(qba.data()), qba.size());

	for (auto& byte : qba)
	{
        //printf("%c", byte);
		update_fsm(byte);
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
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));

            emit receivedThrottleSgn(idata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.roll_sgn)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));

            emit receivedRollSgn(idata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.pitch_sgn)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));

            emit receivedPitchSgn(idata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.cmd_thr)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));
            float fdata = *(reinterpret_cast<float*>(&idata));

            emit receivedCmdThr(fdata);

            pPayload += sizeof(uint32_t);
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
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));

            emit receivedMotor1(idata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.motor2)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));

            emit receivedMotor2(idata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.motor3)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));

            emit receivedMotor3(idata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.motor4)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));

            emit receivedMotor4(idata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.motors_armed)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));

            emit receivedMotorsArmed(idata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.cbit)
        {
            uint32_t idata = *(reinterpret_cast<uint32_t*>(pPayload));

            emit receivedCbit(idata);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.motor_params)
        {
            uint32_t m1_enabled = *(reinterpret_cast<uint32_t*>(pPayload + 0));
            uint32_t m1_min = *(reinterpret_cast<uint32_t*>(pPayload + 4));
            uint32_t m1_max = *(reinterpret_cast<uint32_t*>(pPayload + 8));
            uint32_t m2_enabled = *(reinterpret_cast<uint32_t*>(pPayload + 12));
            uint32_t m2_min = *(reinterpret_cast<uint32_t*>(pPayload + 16));
            uint32_t m2_max = *(reinterpret_cast<uint32_t*>(pPayload + 20));
            uint32_t m3_enabled = *(reinterpret_cast<uint32_t*>(pPayload + 24));
            uint32_t m3_min = *(reinterpret_cast<uint32_t*>(pPayload + 28));
            uint32_t m3_max = *(reinterpret_cast<uint32_t*>(pPayload + 32));
            uint32_t m4_enabled = *(reinterpret_cast<uint32_t*>(pPayload + 36));
            uint32_t m4_min = *(reinterpret_cast<uint32_t*>(pPayload + 40));
            uint32_t m4_max = *(reinterpret_cast<uint32_t*>(pPayload + 44));

            emit receivedMotorsParams(1, m1_enabled > 0, m1_min, m1_max);
            emit receivedMotorsParams(2, m2_enabled > 0, m2_min, m2_max);
            emit receivedMotorsParams(3, m3_enabled > 0, m3_min, m3_max);
            emit receivedMotorsParams(4, m4_enabled > 0, m4_min, m4_max);

            pPayload += 12 * sizeof(uint32_t);
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
            uint32_t imu_type = *(reinterpret_cast<uint32_t*>(pPayload));

            emit receivedImuType(imu_type);

            pPayload += sizeof(uint32_t);
        }
        if (rx_header->Bits.i2c_read)
        {
            uint32_t i2c_read = *(reinterpret_cast<uint32_t*>(pPayload));

            emit receivedI2CRead(i2c_read);

            pPayload += sizeof(uint32_t);
        }
    }
}


uint32_t Maint::Maintenance::calc_exp_bytes(Maint::MAINT_HEADER_T* header)
{
    int bit_sets = 0;
    uint64_t ulong = header->All;
    while (ulong)
    {
        bit_sets += (ulong & 0x01) ? 1 : 0;
        ulong >>= 1;
    }

    if (header->Bits.motor_params)
    {
        bit_sets += 11; // Expecting 4 blocks of 3 integer = 12, 1 still counted
    }

    if (header->Bits.js_params)
    {
        bit_sets += 7; // Expecting 4 blocks of 2 float = 8, 1 still counted
    }

    if (header->Bits.pid_params)
    {
        bit_sets += 17; // Expecting 3 blocks of 6 float = 18, 1 still counted
    }

    if (header->Bits.ptf1_params)
    {
        bit_sets += 8; // Expecting 3 blocks of 3 float = 9, 1 still counted
    }

    return 1 + sizeof(uint32_t) * bit_sets;
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

    _logFile = fopen(_logFileName.toStdString().c_str(), "a");
    fprintf(_logFile, logLine.toStdString().c_str());
    fclose(_logFile);
}