#include "comthread.h"
#include <qdatetime.h>

ComThread::ComThread(QObject* parent) : QThread(parent)
{
	get_status_msg.All = 0;
	get_status_msg.Bits.body_roll = 1;
	get_status_msg.Bits.body_pitch = 1;
	get_status_msg.Bits.body_yaw = 1;
	get_status_msg.Bits.motor1 = 1;
	get_status_msg.Bits.motor2 = 1;
	get_status_msg.Bits.motor3 = 1;
	get_status_msg.Bits.motor4 = 1;

	get_pid_params_msg.All = 0;
	get_pid_params_msg.Bits.pid_params = 1;

	set_pid_roll_gain_msg.All = 0;
	set_pid_roll_gain_msg.Bits.maint_cmd_id = quint64(MaintenanceCommand::MAINT_CMD_SET_ROLL_PID_PARAMS);

	set_pid_pitch_gain_msg.All = 0;
	set_pid_pitch_gain_msg.Bits.maint_cmd_id = quint64(MaintenanceCommand::MAINT_CMD_SET_PITCH_PID_PARAMS);

	set_pid_yaw_gain_msg.All = 0;
	set_pid_yaw_gain_msg.Bits.maint_cmd_id = quint64(MaintenanceCommand::MAINT_CMD_SET_YAW_PID_PARAMS);

	override_radio_msg.All = 0;
	override_radio_msg.Bits.maint_cmd_id = quint64(MaintenanceCommand::MAINT_CMD_SET_OVERRIDE_RADIO);

	override_roll_pitch_throttle_msg.All = 0;
	override_radio_msg.Bits.maint_cmd_id = quint64(MaintenanceCommand::MAINT_CMD_SET_ROLL_PITCH_THROTTLE_SIGNAL);

	override_armed_msg.All = 0;
	override_armed_msg.Bits.maint_cmd_id = quint64(MaintenanceCommand::MAINT_CMD_SET_ARMED_SIGNAL);

	_delay = 250;
	_status = ComThreadStatus::INIT_SERIAL_PORT;
	_serialPortName = "";
	_serialPort = nullptr;

	_rxStatus = RxStatus::WAIT_SYNC;
	_bytesReceived = 0;
	_expectedPayloadSize = 0;
	_lastMessageFromDroneMSecs = -1;
}


void ComThread::setDelay(quint32 delay)
{
	_delay = delay;
}


void ComThread::setSerialPort(QString portName, QSerialPort::BaudRate baudRate)
{
	_serialPortName = portName;
	_baudRate = baudRate;
}

void ComThread::stopCom()
{
	_status = ComThreadStatus::EXIT;
}


void ComThread::refreshPidParams()
{
	_status = ComThreadStatus::TX_GET_PID;
}


void ComThread::pushDataToPayload(quint8* data, int len)
{
	for (int i = 0; i < len; i++)
	{
		_txSetParams.push_back(data[i]);
	}
}


void ComThread::writeRollPidParams(float kp, float ki, float kt, float sat, float ad, float bd)
{
	_txSetParams.clear();

	pushDataToPayload(reinterpret_cast<quint8*>(&kp), 4);
	pushDataToPayload(reinterpret_cast<quint8*>(&ki), 4);
	pushDataToPayload(reinterpret_cast<quint8*>(&kt), 4);
	pushDataToPayload(reinterpret_cast<quint8*>(&sat), 4);
	pushDataToPayload(reinterpret_cast<quint8*>(&ad), 4);
	pushDataToPayload(reinterpret_cast<quint8*>(&bd), 4);

	_status = ComThreadStatus::TX_PID_ROLL;
}


void ComThread::writePitchPidParams(float kp, float ki, float kt, float sat, float ad, float bd)
{
	_txSetParams.clear();

	pushDataToPayload(reinterpret_cast<quint8*>(&kp), 4);
	pushDataToPayload(reinterpret_cast<quint8*>(&ki), 4);
	pushDataToPayload(reinterpret_cast<quint8*>(&kt), 4);
	pushDataToPayload(reinterpret_cast<quint8*>(&sat), 4);
	pushDataToPayload(reinterpret_cast<quint8*>(&ad), 4);
	pushDataToPayload(reinterpret_cast<quint8*>(&bd), 4);

	_status = ComThreadStatus::TX_PID_PITCH;
}


void ComThread::writeYawPidParams(float kp, float ki, float kt, float sat, float ad, float bd)
{
	_txSetParams.clear();

	pushDataToPayload(reinterpret_cast<quint8*>(&kp), 4);
	pushDataToPayload(reinterpret_cast<quint8*>(&ki), 4);
	pushDataToPayload(reinterpret_cast<quint8*>(&kt), 4);
	pushDataToPayload(reinterpret_cast<quint8*>(&sat), 4);
	pushDataToPayload(reinterpret_cast<quint8*>(&ad), 4);
	pushDataToPayload(reinterpret_cast<quint8*>(&bd), 4);

	_status = ComThreadStatus::TX_PID_YAW;
}


void ComThread::overrideArmedSignal(quint16 armed_signal)
{
	_txSetParams.clear();

	pushDataToPayload(reinterpret_cast<quint8*>(&armed_signal), 2);

	_status = ComThreadStatus::TX_SET_ARMED;
}


void ComThread::controlFlight(quint16 roll_signal, quint16 pitch_signal, quint16 throttle_signal)
{
	_txSetParams.clear();

	pushDataToPayload(reinterpret_cast<quint8*>(&roll_signal), 2);
	pushDataToPayload(reinterpret_cast<quint8*>(&pitch_signal), 2);
	pushDataToPayload(reinterpret_cast<quint8*>(&throttle_signal), 2);

	_status = ComThreadStatus::TX_SET_ROLL_PITCH_THROTTLE_SIGNAL;
}


void ComThread::overrideRadio(bool override)
{
	_txSetParams.clear();

	uint32_t iOverride = override ? 1 : 0;

	pushDataToPayload(reinterpret_cast<quint8*>(&iOverride), 4);

	_status = ComThreadStatus::TX_OVERRIDE_RADIO;
}


void ComThread::run()
{
	_serialPort = new QSerialPort();

	while (_status != ComThreadStatus::EXIT)
	{
		switch (_status)
		{
		case ComThreadStatus::INIT_SERIAL_PORT:
		{
			if (initSerialPort())
			{
				_status = ComThreadStatus::TX_GET_STATUS;
			}

			emit serialPortOpened();
			break;
		}
		case ComThreadStatus::TX_GET_STATUS:
		{
			txGet(&get_status_msg);
			break;
		}
		case ComThreadStatus::TX_GET_PID:
		{
			txGet(&get_pid_params_msg);
			_status = ComThreadStatus::TX_GET_STATUS;
			break;
		}
		case ComThreadStatus::TX_OVERRIDE_RADIO:
		{
			txSet(&override_radio_msg);
			_status = ComThreadStatus::TX_GET_STATUS;
			break;
		}
		case ComThreadStatus::TX_SET_ROLL_PITCH_THROTTLE_SIGNAL:
		{
			txSet(&override_roll_pitch_throttle_msg);
			_status = ComThreadStatus::TX_GET_STATUS;
			break;
		}
		case ComThreadStatus::TX_SET_ARMED:
		{
			txSet(&override_armed_msg);
			_status = ComThreadStatus::TX_GET_STATUS;
			break;
		}
		case ComThreadStatus::TX_PID_ROLL:
		{
			txSet(&set_pid_roll_gain_msg);
			_status = ComThreadStatus::TX_GET_STATUS;
			break;
		}
		case ComThreadStatus::TX_PID_PITCH:
		{
			txSet(&set_pid_pitch_gain_msg);
			_status = ComThreadStatus::TX_GET_STATUS;
			break;
		}
		case ComThreadStatus::TX_PID_YAW:
		{
			txSet(&set_pid_yaw_gain_msg);
			_status = ComThreadStatus::TX_GET_STATUS;
			break;
		}

		}

		checkDownlink();

		QThread::msleep(_delay);
	}

	closeSerialPort();
}


void ComThread::checkDownlink()
{
	qint64 currentMSecs = QDateTime::currentMSecsSinceEpoch();
	if (_lastMessageFromDroneMSecs > 0)
	{
		if (currentMSecs - _lastMessageFromDroneMSecs > 1000)
		{
			emit droneDownlink();
		}
	}
}


bool ComThread::initSerialPort()
{
	_serialPort->setPortName(_serialPortName);
	_serialPort->setBaudRate(_baudRate);
	_serialPort->setParity(QSerialPort::NoParity);
	_serialPort->setDataBits(QSerialPort::Data8);
	_serialPort->setStopBits(QSerialPort::OneStop);
	_serialPort->setFlowControl(QSerialPort::NoFlowControl);

	connect(_serialPort, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
	return _serialPort->open(QSerialPort::OpenModeFlag::ReadWrite);
}


void ComThread::txGet(MaintenanceProtocolHdr* msg)
{
	quint8 cks = checksum(reinterpret_cast<quint8*>(msg), sizeof(MaintenanceProtocolHdr));

	QByteArray qba;
	qba.push_back(SYNC_CHAR);
	qba.push_back(msg->Bytes[0]);
	qba.push_back(msg->Bytes[1]);
	qba.push_back(msg->Bytes[2]);
	qba.push_back(msg->Bytes[3]);
	qba.push_back(msg->Bytes[4]);
	qba.push_back(msg->Bytes[5]);
	qba.push_back(msg->Bytes[6]);
	qba.push_back(msg->Bytes[7]);
	qba.push_back(cks);

	_serialPort->write(qba);
	_serialPort->flush();
	_serialPort->waitForBytesWritten();
}


void ComThread::txSet(MaintenanceProtocolHdr* msg)
{
	QByteArray qba;
	qba.push_back(SYNC_CHAR);
	qba.push_back(msg->Bytes[0]);
	qba.push_back(msg->Bytes[1]);
	qba.push_back(msg->Bytes[2]);
	qba.push_back(msg->Bytes[3]);
	qba.push_back(msg->Bytes[4]);
	qba.push_back(msg->Bytes[5]);
	qba.push_back(msg->Bytes[6]);
	qba.push_back(msg->Bytes[7]);

	int payloadSize = 0;
	while (!_txSetParams.isEmpty())
	{
		quint8 payloadByte = _txSetParams.takeFirst();
		qba.push_back(payloadByte);

		payloadSize += 1;
	}

	quint8 cks = checksum(reinterpret_cast<quint8*>(qba.data()), sizeof(MaintenanceProtocolHdr) + payloadSize, true);
	qba.push_back(cks);

	_serialPort->write(qba);
	_serialPort->flush();
	_serialPort->waitForBytesWritten();
}


void ComThread::onReadyRead()
{
	QByteArray qba = _serialPort->readAll();
	for (int i = 0; i < qba.size(); i++)
	{
		updateFsm(qba.at(i));
	}
}


void ComThread::closeSerialPort()
{
	disconnect(_serialPort, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
	_serialPort->close();
	_serialPort->deleteLater();

	emit droneDownlink();
	emit serialPortClosed();
}


quint32 ComThread::calculatePayloadSize(MaintenanceProtocolHdr* rxHeader)
{
	if (rxHeader->All == get_status_msg.All)
	{
		return 36;
	}
	else if (rxHeader->All == get_pid_params_msg.All)
	{
		return 80;
	}
	else
	{
		return 0;
	}
}


void ComThread::updateFsm(quint8 byteIn)
{
	_lastMessageFromDroneMSecs = QDateTime::currentMSecsSinceEpoch();
	emit droneAlive();

	switch (_rxStatus)
	{
		case RxStatus::WAIT_SYNC:
		{
			if (byteIn == SYNC_CHAR)
			{
				_rxStatus = RxStatus::WAIT_HEADER;
				_bytesReceived = 0;
			}
			break;
		}
		case RxStatus::WAIT_HEADER:
		{
			_rxBuf[_bytesReceived] = byteIn;
			_bytesReceived += 1;

			if (_bytesReceived == sizeof(MaintenanceProtocolHdr))
			{
				_expectedPayloadSize = calculatePayloadSize(reinterpret_cast<MaintenanceProtocolHdr*>(_rxBuf));
				_rxStatus = _expectedPayloadSize ? RxStatus::WAIT_PAYLOAD : RxStatus::WAIT_CHECKSUM;
			}
			break;
		}
		case RxStatus::WAIT_PAYLOAD:
		{
			_rxBuf[_bytesReceived] = byteIn;
			_bytesReceived += 1;

			if (_bytesReceived == _expectedPayloadSize)
			{
				_rxStatus = RxStatus::WAIT_CHECKSUM;
			}
			break;
		}
		case RxStatus::WAIT_CHECKSUM:
		{
			quint8 local_cks = checksum(_rxBuf, _bytesReceived);
			quint8 rx_cks = byteIn;
			
			if (rx_cks == local_cks)
			{
				dataIngest();
			}

			_rxStatus = RxStatus::WAIT_SYNC;
		}
		break;
		default:
			_rxStatus = RxStatus::WAIT_SYNC;
			break;
	}
}


void ComThread::dataIngest()
{
	MaintenanceProtocolHdr* hdr = reinterpret_cast<MaintenanceProtocolHdr*>(_rxBuf);

	if (hdr->All == get_status_msg.All)
	{
		emit droneAttitudeUpdate(*reinterpret_cast<float*>(&_rxBuf[8]),
								 *reinterpret_cast<float*>(&_rxBuf[12]),
								 *reinterpret_cast<float*>(&_rxBuf[16]));

		emit droneMotorsUpdate(*reinterpret_cast<float*>(&_rxBuf[20]),
			*reinterpret_cast<float*>(&_rxBuf[24]),
			*reinterpret_cast<float*>(&_rxBuf[28]),
			*reinterpret_cast<float*>(&_rxBuf[32]));
	}
	else if (hdr->All == get_pid_params_msg.All)
	{
		emit rollPidParamsUpdate(*reinterpret_cast<float*>(&_rxBuf[8]),
			*reinterpret_cast<float*>(&_rxBuf[12]),
			*reinterpret_cast<float*>(&_rxBuf[16]),
			*reinterpret_cast<float*>(&_rxBuf[20]),
			*reinterpret_cast<float*>(&_rxBuf[24]),
			*reinterpret_cast<float*>(&_rxBuf[28]));

		emit pitchPidParamsUpdate(*reinterpret_cast<float*>(&_rxBuf[32]),
			*reinterpret_cast<float*>(&_rxBuf[36]),
			*reinterpret_cast<float*>(&_rxBuf[40]),
			*reinterpret_cast<float*>(&_rxBuf[44]),
			*reinterpret_cast<float*>(&_rxBuf[48]),
			*reinterpret_cast<float*>(&_rxBuf[52]));

		emit yawPidParamsUpdate(*reinterpret_cast<float*>(&_rxBuf[56]),
			*reinterpret_cast<float*>(&_rxBuf[60]),
			*reinterpret_cast<float*>(&_rxBuf[64]),
			*reinterpret_cast<float*>(&_rxBuf[68]),
			*reinterpret_cast<float*>(&_rxBuf[72]),
			*reinterpret_cast<float*>(&_rxBuf[76]));
	}
	
}