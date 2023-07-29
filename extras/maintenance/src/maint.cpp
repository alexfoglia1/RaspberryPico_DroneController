#include "maint.h"


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
    _tx_param_enabled = 1;
    _tx_param_min_signal = 1000;
    _tx_param_max_signal = 2000;

}

bool Maint::Maintenance::Open(QString serialPortName, enum QSerialPort::BaudRate baud)
{
    _serialPort->setPortName(serialPortName);
	_serialPort->setBaudRate(baud);
	_serialPort->setParity(QSerialPort::NoParity);
	_serialPort->setDataBits(QSerialPort::Data8);
	_serialPort->setStopBits(QSerialPort::OneStop);
	_serialPort->setFlowControl(QSerialPort::NoFlowControl);

	connect(_serialPort, SIGNAL(readyRead()), this, SLOT(OnRx()));

	return _serialPort->open(QSerialPort::OpenModeFlag::ReadWrite);
}


void Maint::Maintenance::Close()
{
    _txTimer->stop();
    _txTimer->disconnect(_txTimer, SIGNAL(timeout()), this, SLOT(Tx()));

    _serialPort->disconnect(_serialPort, SIGNAL(readyRead()), this, SLOT(OnRx()));
    _serialPort->close();
}


void Maint::Maintenance::EnableTx()
{
    _txTimer->setInterval(20);
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

        _txCommand.All = 0;
        _txStatus = Maint::TX_STATUS::TX_GET;
    }

    emit txRawData(reinterpret_cast<quint8*>(qba.data()), bytesWritten);
}


void Maint::Maintenance::OnRx()
{
	QByteArray qba = _serialPort->readAll();
	
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

    return 1 + sizeof(uint32_t) * bit_sets;
}

