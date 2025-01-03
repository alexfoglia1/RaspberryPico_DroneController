#include "MaintenanceWindow.h"
#include <qserialport.h>
#include <qdatetime.h>
#include <qmessagebox.h>
#include <qfiledialog.h>
#include <qtconcurrentrun.h>
#include <corecrt_math_defines.h>


MaintenanceWindow::MaintenanceWindow()
{

	_ui.setupUi(this);
	_progressUi.setupUi(&_autoscanProgressWindow);
	_autoscanProgressWindow.setVisible(false);

	_maintHandler = nullptr;
	_rxCounter = 0;
	_rxT0millis = -1;
	_jsIsControlling = false;
	_jsIsArmed = false;

	_imuTypeToString =
	{
		{Maint::IMU_TYPE::LSM9DS1, "LSM9DS1"},
		{Maint::IMU_TYPE::MPU6050, "MPU6050"},
		{Maint::IMU_TYPE::BNO055,  "BNO055"}
	};

	_imuTypeToI2CAddr =
	{
		{Maint::IMU_TYPE::LSM9DS1, 0x6A},
		{Maint::IMU_TYPE::MPU6050, 0x68},
		{Maint::IMU_TYPE::BNO055,  0x28}
	};


	_defaultPlotSpan =
	{
		{"NONE", 16000},
		{"RX_COUNT", 4000},
		{"FREQUENCY", 300},
		{"RAW_ACC_X", 4},
		{"RAW_ACC_Y", 4},
		{"RAW_ACC_Z", 4},
		{"RAW_GYRO_X", 360},
		{"RAW_GYRO_Y", 360},
		{"RAW_GYRO_Z", 360},
		{"RAW_MAGN_X", 100},
		{"RAW_MAGN_Y", 100},
		{"RAW_MAGN_Z", 100},
		{"FILTERED_ACC_X", 4},
		{"FILTERED_ACC_Y", 4},
		{"FILTERED_ACC_Z", 4},
		{"FILTERED_GYRO_X", 360},
		{"FILTERED_GYRO_Y", 360},
		{"FILTERED_GYRO_Z", 360},
		{"FILTERED_MAGN_X", 100},
		{"FILTERED_MAGN_Y", 100},
		{"FILTERED_MAGN_Z", 100},
		{"THROTTLE_SIGNAL", 4000},
		{"ROLL_SIGNAL", 4000},
		{"PITCH_SIGNAL", 4000},
		{"CMD_THROTTLE", 4000},
		{"CMD_ROLL", 10},
		{"CMD_PITCH", 10},
		{"BODY_ROLL", 360},
		{"BODY_PITCH", 360},
		{"BODY_YAW", 360},
		{"ROLL_PID_ERR", 100},
		{"ROLL_PID_P", 100},
		{"ROLL_PID_I", 100},
		{"ROLL_PID_D", 100},
		{"ROLL_PID_U", 100},
		{"PITCH_PID_ERR", 100},
		{"PITCH_PID_P", 100},
		{"PITCH_PID_I", 100},
		{"PITCH_PID_D", 100},
		{"PITCH_PID_U", 100},
		{"YAW_PID_ERR", 100},
		{"YAW_PID_P", 100},
		{"YAW_PID_I", 100},
		{"YAW_PID_D", 100},
		{"YAW_PID_U", 100},
		{"MOTOR_1", 4000},
		{"MOTOR_2", 4000},
		{"MOTOR_3", 4000},
		{"MOTOR_4", 4000},
		{"MOTORS_ARMED", 2}
	};

	_rxMotorParams =
	{
		{1, {true, 1000, 2000}}, //M1
		{2, {true, 1000, 2000}}, //M2
		{3, {true, 1000, 2000}}, //M3
		{4, {true, 1000, 2000}}, //M4
	};

	_rxJsParams =
	{
		{1, {0.0f, 1.0f}}, //THROTTLE
		{2, {0.0f, 1.0f}}, //ROLL
		{3, {0.0f, 1.0f}}, //PITCH
	};

	_rxPidParams =
	{
		{1, {1.0f, 0.0f, 0.0f, 50.0f, 0.0f, 0.0f}}, //ROLL
		{2, {1.0f, 0.0f, 0.0f, 50.0f, 0.0f, 0.0f}}, //PITCH
		{3, {1.0f, 0.0f, 0.0f, 50.0f, 0.0f, 0.0f}}, //YAW
	};

	_rxPtf1Params =
	{
		{1, {0.1f, 0.1f, 0.1f}}, //ACCELEROMETER
		{2, {0.1f, 0.1f, 0.1f}}, //GYROSCOPE
		{3, {0.1f, 0.1f, 0.1f}}, //MAGNETOMETER
	};

	_rxRollOffset = 0.0f;
	_rxPitchOffset = 0.0f;
	_rxRoll = 0.0f;
	_rxPitch = 0.0f;
	_rxThrottleParams[0] = 1000;
	_rxThrottleParams[1] = 1000;
	_rxThrottleParams[2] = 1000;

	_rxImuType = Maint::IMU_TYPE::BNO055;
	for (uint32_t i = uint32_t(Maint::IMU_TYPE::FIRST); i < uint32_t(Maint::IMU_TYPE::SIZE); i++)
	{
		QString text = _imuTypeToString[Maint::IMU_TYPE(i)];
		QVariant userData(i);

		_ui.comboSetImuType->addItem(text, userData);
	}

	_ui.comboSelBaud->addItem("1200", QSerialPort::Baud1200);
	_ui.comboSelBaud->addItem("2400", QSerialPort::Baud2400);
	_ui.comboSelBaud->addItem("4800", QSerialPort::Baud4800);
	_ui.comboSelBaud->addItem("9600", QSerialPort::Baud9600);
	_ui.comboSelBaud->addItem("19200", QSerialPort::Baud19200);
	_ui.comboSelBaud->addItem("38400", QSerialPort::Baud38400);
	_ui.comboSelBaud->addItem("57600", QSerialPort::Baud57600);
	_ui.comboSelBaud->addItem("115200", QSerialPort::Baud115200);
	_ui.comboSelBaud->setCurrentIndex(5); // Default 38400

	_maintHandler = new Maint::Maintenance();

	connect(&_js, SIGNAL(overrideRadio(bool)), this, SLOT(OnOverrideRadio(bool)));
	connect(&_js, SIGNAL(overrideSignals(quint16, quint16, quint16, quint16)), this, SLOT(OnOverrideSignals(quint16, quint16, quint16, quint16)));

	connect(_ui.btnOpenSerialPort, SIGNAL(clicked()), this, SLOT(OnBtnOpenSerialPort()));
	connect(_ui.btnOpenBoot, SIGNAL(clicked()), this, SLOT(OnBtnOpenBoot()));
	connect(_ui.btnRescanPorts, SIGNAL(clicked()), this, SLOT(OnBtnRescanPorts()));
	connect(_ui.btnSendSetMotors, SIGNAL(clicked()), this, SLOT(OnBtnSendSetMotors()));
	connect(_ui.btnSendMaintenanceParams, SIGNAL(clicked()), this, SLOT(OnBtnSendMaintenanceParams()));
	connect(_ui.btnSendMaintenanceJsParams, SIGNAL(clicked()), this, SLOT(OnBtnSendJsParams()));
	connect(_ui.btnSendPidParams, SIGNAL(clicked()), this, SLOT(OnBtnSendPidParams()));
	connect(_ui.btnSendPtf1Params, SIGNAL(clicked()), this, SLOT(OnBtnSendPtf1Params()));
	connect(_ui.btnSendImuType, SIGNAL(clicked()), this, SLOT(OnBtnSendImuType()));
	connect(_ui.btnFlashWrite, SIGNAL(clicked()), this, SLOT(OnBtnFlashWrite()));
	connect(_ui.btnRefreshParams, SIGNAL(clicked()), this, SLOT(OnBtnRefreshParams()));
	connect(_ui.btnI2CRead, SIGNAL(clicked()), this, SLOT(OnBtnI2CRead()));
	connect(_ui.btnI2CWrite, SIGNAL(clicked()), this, SLOT(OnBtnI2CWrite()));
	connect(_ui.btnTxImuOffset, SIGNAL(clicked()), this, SLOT(OnBtnTxImuOffset()));
	connect(_ui.btnImuAutoOffset, SIGNAL(clicked()), this, SLOT(OnBtnImuAutoOffset()));
	connect(_ui.btnSendThrottleParams, SIGNAL(clicked()), this, SLOT(OnBtnSendThrottleParams()));

	connect(_ui.spinSetMotorsValue, SIGNAL(valueChanged(int)), this, SLOT(OnSpinSetMotorsValue(int)));

	connect(_ui.plotTimeSlider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotSliderValueChanged(int)));
	connect(_ui.plotTrack1Slider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotTrack1ValueChanged(int)));
	connect(_ui.plotTrack2Slider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotTrack2ValueChanged(int)));
	connect(_ui.plotTrack3Slider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotTrack3ValueChanged(int)));
	connect(_ui.plotTrack4Slider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotTrack4ValueChanged(int)));

	connect(_ui.comboSelTrack1, SIGNAL(currentTextChanged(const QString&)), this, SLOT(OnComboTrack1TextChanged(const QString&)));
	connect(_ui.comboSelTrack2, SIGNAL(currentTextChanged(const QString&)), this, SLOT(OnComboTrack2TextChanged(const QString&)));
	connect(_ui.comboSelTrack3, SIGNAL(currentTextChanged(const QString&)), this, SLOT(OnComboTrack3TextChanged(const QString&)));
	connect(_ui.comboSelTrack4, SIGNAL(currentTextChanged(const QString&)), this, SLOT(OnComboTrack4TextChanged(const QString&)));

	connect(_ui.actionClear_logs, SIGNAL(triggered()), this, SLOT(OnClearLogs()));
	connect(_ui.actionOpen, SIGNAL(triggered()), this, SLOT(OnActionOpen()));


	connect(_maintHandler, SIGNAL(receivedRawAccelX(float)), this, SLOT(OnReceivedRawAccelX(float)));
	connect(_maintHandler, SIGNAL(receivedRawAccelY(float)), this, SLOT(OnReceivedRawAccelY(float)));
	connect(_maintHandler, SIGNAL(receivedRawAccelZ(float)), this, SLOT(OnReceivedRawAccelZ(float)));
	connect(_maintHandler, SIGNAL(receivedRawGyroX(float)), this, SLOT(OnReceivedRawGyroX(float)));
	connect(_maintHandler, SIGNAL(receivedRawGyroY(float)), this, SLOT(OnReceivedRawGyroY(float)));
	connect(_maintHandler, SIGNAL(receivedRawGyroZ(float)), this, SLOT(OnReceivedRawGyroZ(float)));
	connect(_maintHandler, SIGNAL(receivedRawMagnX(float)), this, SLOT(OnReceivedRawMagnX(float)));
	connect(_maintHandler, SIGNAL(receivedRawMagnY(float)), this, SLOT(OnReceivedRawMagnY(float)));
	connect(_maintHandler, SIGNAL(receivedRawMagnZ(float)), this, SLOT(OnReceivedRawMagnZ(float)));

	connect(_maintHandler, SIGNAL(receivedFilteredAccelX(float)), this, SLOT(OnReceivedFilteredAccelX(float)));
	connect(_maintHandler, SIGNAL(receivedFilteredAccelY(float)), this, SLOT(OnReceivedFilteredAccelY(float)));
	connect(_maintHandler, SIGNAL(receivedFilteredAccelZ(float)), this, SLOT(OnReceivedFilteredAccelZ(float)));
	connect(_maintHandler, SIGNAL(receivedFilteredGyroX(float)), this, SLOT(OnReceivedFilteredGyroX(float)));
	connect(_maintHandler, SIGNAL(receivedFilteredGyroY(float)), this, SLOT(OnReceivedFilteredGyroY(float)));
	connect(_maintHandler, SIGNAL(receivedFilteredGyroZ(float)), this, SLOT(OnReceivedFilteredGyroZ(float)));
	connect(_maintHandler, SIGNAL(receivedFilteredMagnX(float)), this, SLOT(OnReceivedFilteredMagnX(float)));
	connect(_maintHandler, SIGNAL(receivedFilteredMagnY(float)), this, SLOT(OnReceivedFilteredMagnY(float)));
	connect(_maintHandler, SIGNAL(receivedFilteredMagnZ(float)), this, SLOT(OnReceivedFilteredMagnZ(float)));

	connect(_maintHandler, SIGNAL(receivedThrottleSgn(uint16_t)), this, SLOT(OnReceivedThrottleSgn(uint16_t)));
	connect(_maintHandler, SIGNAL(receivedPitchSgn(uint16_t)), this, SLOT(OnReceivedPitchSgn(uint16_t)));
	connect(_maintHandler, SIGNAL(receivedRollSgn(uint16_t)), this, SLOT(OnReceivedRollSgn(uint16_t)));
	connect(_maintHandler, SIGNAL(receivedCmdThr(uint16_t)), this, SLOT(OnReceivedCmdThr(uint16_t)));
	connect(_maintHandler, SIGNAL(receivedCmdPitch(float)), this, SLOT(OnReceivedCmdPitch(float)));
	connect(_maintHandler, SIGNAL(receivedCmdRoll(float)), this, SLOT(OnReceivedCmdRoll(float)));

	connect(_maintHandler, SIGNAL(receivedBodyRoll(float)), this, SLOT(OnReceivedBodyRoll(float)));
	connect(_maintHandler, SIGNAL(receivedBodyPitch(float)), this, SLOT(OnReceivedBodyPitch(float)));
	connect(_maintHandler, SIGNAL(receivedBodyYaw(float)), this, SLOT(OnReceivedBodyYaw(float)));

	connect(_maintHandler, SIGNAL(receivedMotor1(uint16_t)), this, SLOT(OnReceivedMotor1(uint16_t)));
	connect(_maintHandler, SIGNAL(receivedMotor2(uint16_t)), this, SLOT(OnReceivedMotor2(uint16_t)));
	connect(_maintHandler, SIGNAL(receivedMotor3(uint16_t)), this, SLOT(OnReceivedMotor3(uint16_t)));
	connect(_maintHandler, SIGNAL(receivedMotor4(uint16_t)), this, SLOT(OnReceivedMotor4(uint16_t)));
	connect(_maintHandler, SIGNAL(receivedMotorsArmed(uint8_t)), this, SLOT(OnReceivedMotorsArmed(uint8_t)));

	connect(_maintHandler, SIGNAL(receivedRollPidErr(float)), this, SLOT(OnReceivedRollPidErr(float)));
	connect(_maintHandler, SIGNAL(receivedRollPidP(float)), this, SLOT(OnReceivedRollPidP(float)));
	connect(_maintHandler, SIGNAL(receivedRollPidI(float)), this, SLOT(OnReceivedRollPidI(float)));
	connect(_maintHandler, SIGNAL(receivedRollPidD(float)), this, SLOT(OnReceivedRollPidD(float)));
	connect(_maintHandler, SIGNAL(receivedRollPidU(float)), this, SLOT(OnReceivedRollPidU(float)));
	connect(_maintHandler, SIGNAL(receivedPitchPidErr(float)), this, SLOT(OnReceivedPitchPidErr(float)));
	connect(_maintHandler, SIGNAL(receivedPitchPidP(float)), this, SLOT(OnReceivedPitchPidP(float)));
	connect(_maintHandler, SIGNAL(receivedPitchPidI(float)), this, SLOT(OnReceivedPitchPidI(float)));
	connect(_maintHandler, SIGNAL(receivedPitchPidD(float)), this, SLOT(OnReceivedPitchPidD(float)));
	connect(_maintHandler, SIGNAL(receivedPitchPidU(float)), this, SLOT(OnReceivedPitchPidU(float)));
	connect(_maintHandler, SIGNAL(receivedYawPidErr(float)), this, SLOT(OnReceivedYawPidErr(float)));
	connect(_maintHandler, SIGNAL(receivedYawPidP(float)), this, SLOT(OnReceivedYawPidP(float)));
	connect(_maintHandler, SIGNAL(receivedYawPidI(float)), this, SLOT(OnReceivedYawPidI(float)));
	connect(_maintHandler, SIGNAL(receivedYawPidD(float)), this, SLOT(OnReceivedYawPidD(float)));
	connect(_maintHandler, SIGNAL(receivedYawPidU(float)), this, SLOT(OnReceivedYawPidU(float)));

	connect(_maintHandler, SIGNAL(receivedCbit(uint32_t)), this, SLOT(OnReceivedCbit(uint32_t)));

	connect(_maintHandler, SIGNAL(receivedMotorsParams(uint32_t, bool, uint16_t, uint16_t)), this, SLOT(OnReceivedMotorsParams(uint32_t, bool, uint16_t, uint16_t)));
	connect(_maintHandler, SIGNAL(receivedJsParams(uint32_t, float, float)), this, SLOT(OnReceivedJsParams(uint32_t, float, float)));
	connect(_maintHandler, SIGNAL(receivedPidParams(uint32_t, float, float, float, float, float, float)), this, SLOT(OnReceivedPidParams(uint32_t, float, float, float, float, float, float)));
	connect(_maintHandler, SIGNAL(receivedPtf1Params(uint32_t, float, float, float)), this, SLOT(OnReceivedPtf1Params(uint32_t, float, float, float)));
	connect(_maintHandler, SIGNAL(receivedImuType(uint8_t)), this, SLOT(OnReceivedImuType(uint8_t)));
	connect(_maintHandler, SIGNAL(receivedI2CRead(uint8_t)), this, SLOT(OnReceivedI2CRead(uint8_t)));
	connect(_maintHandler, SIGNAL(receivedSwVer(uint8_t, uint8_t, uint8_t, uint8_t)), this, SLOT(OnReceivedSwVer(uint8_t, uint8_t, uint8_t, uint8_t)));
	connect(_maintHandler, SIGNAL(receivedImuOffset(float, float)), this, SLOT(OnReceivedImuOffset(float, float)));
	connect(_maintHandler, SIGNAL(receivedThrottleParams(uint16_t, uint16_t, uint16_t)), this, SLOT(OnReceivedThrottleParams(uint16_t, uint16_t, uint16_t)));

	connect(_maintHandler, SIGNAL(txRawData(quint8*, int)), this, SLOT(OnTxRawData(quint8*, int)));
	connect(_maintHandler, SIGNAL(rxRawData(bool, quint8*, int)), this, SLOT(OnRxRawData(bool, quint8*, int)));
	
	connect(_maintHandler, SIGNAL(downlink()), this, SLOT(OnPicoDownlink()));

	QTimer* autoscanComPortsTimer = new QTimer();
	autoscanComPortsTimer->setSingleShot(true);
	autoscanComPortsTimer->setTimerType(Qt::PreciseTimer);

	connect(autoscanComPortsTimer, &QTimer::timeout, this, [this, autoscanComPortsTimer] { this->autoScanComPorts(); autoscanComPortsTimer->deleteLater(); });
	//autoscanComPortsTimer->start(500);

	QTimer* checkHeaderChanged = new QTimer();
	checkHeaderChanged->setSingleShot(false);
	checkHeaderChanged->setTimerType(Qt::PreciseTimer);
	checkHeaderChanged->setInterval(_txDelayMillis);

	connect(checkHeaderChanged, &QTimer::timeout, this, &MaintenanceWindow::OnHeaderChanged);
	checkHeaderChanged->start();

	const double SAMPLE_PERIOD_S = _txDelayMillis * 1e-3;
	const double SAMPLE_FREQ = 1 / SAMPLE_PERIOD_S;

	_ui.plot->UpdateSamplesPerSecond(SAMPLE_FREQ);
	int samplesInNewValue = SAMPLE_FREQ * _ui.plotTimeSlider->value();

	_ui.plot->SetXSpan(samplesInNewValue);

    _ui.plot->ForceRepaint();
    _ui.pfdHeading->ForceRepaint();
    _ui.pfdRollPitch->ForceRepaint();
}


void MaintenanceWindow::OnPicoDownlink()
{
	_ui.lblRxData->setStyleSheet("background-color:#FF0000");
}


void MaintenanceWindow::autoScanComPorts()
{
	_progressUi.autoscanStatusPrompt->setText("");

	_autoscanProgressWindow.setVisible(true);
	for (int i = 0; i < 100; i++)
	{
#ifdef __linux__
        QString portName = QString("/dev/ttyACM%1").arg(i);
#else
		QString portName = QString("COM%1").arg(i);
#endif
		_progressUi.autoscanStatusPrompt->append(QString("Testing %1").arg(portName));
		qApp->processEvents();

		QSerialPort* serialPort = new QSerialPort(portName);
		if (serialPort->open(QSerialPort::ReadWrite))
		{
			serialPort->close();
			serialPort->deleteLater();

			_ui.comboSelPort->addItem(portName);
			_progressUi.autoscanStatusPrompt->append(QString("OK"));
		}
		else
		{
			_progressUi.autoscanStatusPrompt->append(QString("FAIL"));
		}

		_progressUi.autoscanStatusProgress->setValue(i + 1);

		qApp->processEvents();
	}
	_autoscanProgressWindow.setVisible(false);
}


void MaintenanceWindow::checkPlot(QString expected, double value)
{
	if (_ui.comboSelTrack1->currentText().toUpper() == expected.toUpper())
	{
		_ui.plot->AddValue(0, value);
	}

	if (_ui.comboSelTrack2->currentText().toUpper() == expected.toUpper())
	{
		_ui.plot->AddValue(1, value);
	}

	if (_ui.comboSelTrack3->currentText().toUpper() == expected.toUpper())
	{
		_ui.plot->AddValue(2, value);
	}

	if (_ui.comboSelTrack4->currentText().toUpper() == expected.toUpper())
	{
		_ui.plot->AddValue(3, value);
	}
}


void MaintenanceWindow::OnPlotSliderValueChanged(int newValue)
{
	const double SAMPLE_PERIOD_S = _txDelayMillis * 1e-3;
	const double SAMPLE_FREQ = 1 / SAMPLE_PERIOD_S;

	_ui.plot->UpdateSamplesPerSecond(SAMPLE_FREQ);

	int samplesInNewValue = SAMPLE_FREQ * newValue;

	_ui.plot->SetXSpan(samplesInNewValue);

	_ui.lblTimeSpan->setText(QString("Time: %1s").arg(newValue));
}


void MaintenanceWindow::OnPlotTrack1ValueChanged(int newValue)
{
	_ui.plot->SetYSpan(0, newValue);
}


void MaintenanceWindow::OnPlotTrack2ValueChanged(int newValue)
{
	_ui.plot->SetYSpan(1, newValue);
}


void MaintenanceWindow::OnPlotTrack3ValueChanged(int newValue)
{
	_ui.plot->SetYSpan(2, newValue);
}


void MaintenanceWindow::OnPlotTrack4ValueChanged(int newValue)
{
	_ui.plot->SetYSpan(3, newValue);
}



void MaintenanceWindow::OnComboTrack1TextChanged(const QString& newText)
{
	_ui.plot->SetYSpan(0, _defaultPlotSpan[newText]);

	if (newText.toUpper().contains("NONE"))
	{
		_ui.plot->ClearData(0);
	}
}


void MaintenanceWindow::OnComboTrack2TextChanged(const QString& newText)
{
	_ui.plot->SetYSpan(1, _defaultPlotSpan[newText]);

	if (newText.toUpper().contains("NONE"))
	{
		_ui.plot->ClearData(1);
	}
}

void MaintenanceWindow::OnComboTrack3TextChanged(const QString& newText)
{
	_ui.plot->SetYSpan(2, _defaultPlotSpan[newText]);

	if (newText.toUpper().contains("NONE"))
	{
		_ui.plot->ClearData(2);
	}
}

void MaintenanceWindow::OnComboTrack4TextChanged(const QString& newText)
{
	_ui.plot->SetYSpan(3, _defaultPlotSpan[newText]);

	if (newText.toUpper().contains("NONE"))
	{
		_ui.plot->ClearData(3);
	}
}



void MaintenanceWindow::OnOverrideRadio(bool radioOverride)
{
	_ui.checkRadioOverride->setChecked(radioOverride);
	_maintHandler->UpdateRemoteControlTag(_ui.checkRadioOverride->isChecked(), 1000, 1500, 1500, 1000);

	if (!radioOverride)
	{
		_ui.lineTxArmedSignal->setText("");
		_ui.lineTxRollSignal->setText("");
		_ui.lineTxPitchSignal->setText("");
		_ui.lineTxThrottleSignal->setText("");
	}
	else
	{
		_ui.lineTxArmedSignal->setText("1000");
		_ui.lineTxRollSignal->setText("1500");
		_ui.lineTxPitchSignal->setText("1500");
		_ui.lineTxThrottleSignal->setText("1000");
	}
}


void MaintenanceWindow::OnOverrideSignals(quint16 armedSignal, quint16 rollSignal, quint16 pitchSignal, quint16 throttleSignal)
{
	_ui.lineTxArmedSignal->setText(QString::number(armedSignal));
	_ui.lineTxRollSignal->setText(QString::number(rollSignal));
	_ui.lineTxPitchSignal->setText(QString::number(pitchSignal));
	_ui.lineTxThrottleSignal->setText(QString::number(throttleSignal));

	if (_maintHandler)
	{
		_maintHandler->UpdateRemoteControlTag(_ui.checkRadioOverride->isChecked(), armedSignal, rollSignal, pitchSignal, throttleSignal);
	}
}


void MaintenanceWindow::OnBtnOpenSerialPort()
{
	if (_ui.btnOpenSerialPort->text().toUpper() == "OPEN")
	{
		int idx = _ui.comboSelBaud->currentIndex();
		enum QSerialPort::BaudRate baud = QSerialPort::Baud38400;
		if (idx != -1)
		{
#ifdef __linux__
            baud = QSerialPort::BaudRate(_ui.comboSelBaud->itemData(idx).toInt());
#else
            baud = enum QSerialPort::BaudRate(_ui.comboSelBaud->itemData(idx).toInt());
#endif
		}

		if (_maintHandler->Open(_ui.comboSelPort->currentText(), baud))
		{
			_maintHandler->EnableTx(_txDelayMillis);

			_ui.comboSelPort->setEnabled(false);
			_ui.groupBoxTx->setEnabled(true);
			_ui.TxMaintenanceGroup->setEnabled(true);
			_ui.TxMaintenanceGroup_2->setEnabled(true);
			_ui.TxMaintenanceGroup_3->setEnabled(true);
			_ui.TxMotorParamsGroup->setEnabled(true);

			_ui.btnOpenSerialPort->setText("Close");
			_ui.btnRescanPorts->setEnabled(false);

			_rxCounter = 0;
			_rxT0millis = -1;

			_ui.lblRxCount->setText("Count: 0");
			_ui.lblRxFreq->setText("Frequency: NaN Hz");
		}
		else
		{
			QMessageBox::warning(this, "Error", QString("Cannot open serial port %1").arg(_ui.comboSelPort->currentText()));
		}
	}
	else
	{
		_maintHandler->Close();
		_ui.comboSelPort->setEnabled(true);
		_ui.btnRescanPorts->setEnabled(true);
		_ui.groupBoxTx->setEnabled(false);
		_ui.btnOpenSerialPort->setText("Open");
		_ui.lblRxData->setStyleSheet("background-color:#FF0000");
		_ui.lblTxData->setStyleSheet("background-color:#FF0000");

		_rxCounter = 0;
		_rxT0millis = -1;

		_ui.lblRxCount->setText("Count: 0");
		_ui.lblRxFreq->setText("Frequency: NaN Hz");
	}
}


void MaintenanceWindow::OnBtnOpenBoot()
{
	_maintHandler->Open(_ui.comboSelPort->currentText(), QSerialPort::Baud1200);
	_maintHandler->Close();
	_ui.comboSelPort->setEnabled(true);
	_ui.btnOpenSerialPort->setText("Open");
	_ui.lblRxData->setStyleSheet("background-color:#FF0000");
	_ui.lblTxData->setStyleSheet("background-color:#FF0000");
}


void MaintenanceWindow::OnBtnRescanPorts()
{
	_ui.comboSelPort->clear();
	autoScanComPorts();
}


void MaintenanceWindow::OnClearLogs()
{
	int n = 0;
	if (_maintHandler)
	{
		n = _maintHandler->ClearLogs();
	}

	QMessageBox::information(this, "Info", QString("Deleted %1 files").arg(n));
}


void MaintenanceWindow::OnHeaderChanged()
{
	Maint::MAINT_HEADER_T header;
	header.All = 0x00;

	if (_ui.checkTxRawAccX->isChecked())
	{
		header.Bits.accel_x = 1;
	}
	else
	{
		header.Bits.accel_x = 0;
		_ui.checkRxRawAccX->setChecked(false);
		_ui.lineRxRawAccX->setText("");
	}

	if (_ui.checkTxRawAccY->isChecked())
	{
		header.Bits.accel_y = 1;
	}
	else
	{
		header.Bits.accel_y = 0;
		_ui.checkRxRawAccY->setChecked(false);
		_ui.lineRxRawAccY->setText("");
	}

	if (_ui.checkTxRawAccZ->isChecked())
	{
		header.Bits.accel_z = 1;
	}
	else
	{
		header.Bits.accel_z = 0;
		_ui.checkRxRawAccZ->setChecked(false);
		_ui.lineRxRawAccZ->setText("");
	}

	if (_ui.checkTxRawGyroX->isChecked())
	{
		header.Bits.gyro_x = 1;
	}
	else
	{
		header.Bits.gyro_x = 0;
		_ui.checkRxRawGyroX->setChecked(false);
		_ui.lineRxRawGyroX->setText("");
	}

	if (_ui.checkTxRawGyroY->isChecked())
	{
		header.Bits.gyro_y = 1;
	}
	else
	{
		header.Bits.gyro_y = 0;
		_ui.checkRxRawGyroY->setChecked(false);
		_ui.lineRxRawGyroY->setText("");
	}

	if (_ui.checkTxRawGyroZ->isChecked())
	{
		header.Bits.gyro_z = 1;
	}
	else
	{
		header.Bits.gyro_z = 0;
		_ui.checkRxRawGyroZ->setChecked(false);
		_ui.lineRxRawGyroZ->setText("");
	}

	if (_ui.checkTxRawMagnX->isChecked())
	{
		header.Bits.magn_x = 1;
	}
	else
	{
		header.Bits.magn_x = 0;
		_ui.checkRxRawMagnX->setChecked(false);
		_ui.lineRxRawMagnX->setText("");
	}

	if (_ui.checkTxRawMagnY->isChecked())
	{
		header.Bits.magn_y = 1;
	}
	else
	{
		header.Bits.magn_y = 0;
		_ui.checkRxRawMagnY->setChecked(false);
		_ui.lineRxRawMagnY->setText("");
	}

	if (_ui.checkTxRawMagnZ->isChecked())
	{
		header.Bits.magn_z = 1;
	}
	else
	{
		header.Bits.magn_z = 0;
		_ui.checkRxRawMagnZ->setChecked(false);
		_ui.lineRxRawMagnZ->setText("");
	}

	if (_ui.checkTxFiltAccX->isChecked())
	{
		header.Bits.accel_x_f = 1;
	}
	else
	{
		header.Bits.accel_x_f = 0;
		_ui.checkRxFiltAccX->setChecked(false);
		_ui.lineRxFiltAccX->setText("");
	}

	if (_ui.checkTxFiltAccY->isChecked())
	{
		header.Bits.accel_y_f = 1;
	}
	else
	{
		header.Bits.accel_y_f = 0;
		_ui.checkRxFiltAccY->setChecked(false);
		_ui.lineRxFiltAccY->setText("");
	}

	if (_ui.checkTxFiltAccZ->isChecked())
	{
		header.Bits.accel_z_f = 1;
	}
	else
	{
		header.Bits.accel_z_f = 0;
		_ui.checkRxFiltAccZ->setChecked(false);
		_ui.lineRxFiltAccZ->setText("");
	}

	if (_ui.checkTxFiltGyroX->isChecked())
	{
		header.Bits.gyro_x_f = 1;
	}
	else
	{
		header.Bits.gyro_x_f = 0;
		_ui.checkRxFiltGyroX->setChecked(false);
		_ui.lineRxFiltGyroX->setText("");
	}

	if (_ui.checkTxFiltGyroY->isChecked())
	{
		header.Bits.gyro_y_f = 1;
	}
	else
	{
		header.Bits.gyro_y_f = 0;
		_ui.checkRxFiltGyroY->setChecked(false);
		_ui.lineRxFiltGyroY->setText("");
	}

	if (_ui.checkTxFiltGyroZ->isChecked())
	{
		header.Bits.gyro_z_f = 1;
	}
	else
	{
		header.Bits.gyro_z_f = 0;
		_ui.checkRxFiltGyroZ->setChecked(false);
		_ui.lineRxFiltGyroZ->setText("");
	}

	if (_ui.checkTxFiltMagnX->isChecked())
	{
		header.Bits.magn_x_f = 1;
	}
	else
	{
		header.Bits.magn_x_f = 0;
		_ui.checkRxFiltMagnX->setChecked(false);
		_ui.lineRxFiltMagnX->setText("");
	}

	if (_ui.checkTxFiltMagnY->isChecked())
	{
		header.Bits.magn_y_f = 1;
	}
	else
	{
		header.Bits.magn_y_f = 0;
		_ui.checkRxFiltMagnY->setChecked(false);
		_ui.lineRxFiltMagnY->setText("");
	}

	if (_ui.checkTxFiltMagnZ->isChecked())
	{
		header.Bits.magn_z_f = 1;
	}
	else
	{
		header.Bits.magn_z_f = 0;
		_ui.checkRxFiltMagnZ->setChecked(false);
		_ui.lineRxFiltMagnZ->setText("");
	}

	if (_ui.checkTxThrottleSignal->isChecked())
	{
		header.Bits.throttle_sgn = 1;
	}
	else
	{
		header.Bits.throttle_sgn = 0;
		_ui.checkRxThrottleSignal->setChecked(false);
		_ui.lineRxThrottleSignal->setText("");
	}

	if (_ui.checkTxRollSignal->isChecked())
	{
		header.Bits.roll_sgn = 1;
	}
	else
	{
		header.Bits.roll_sgn = 0;
		_ui.checkRxRollSignal->setChecked(false);
		_ui.lineRxRollSignal->setText("");
	}

	if (_ui.checkTxPitchSignal->isChecked())
	{
		header.Bits.pitch_sgn = 1;
	}
	else
	{
		header.Bits.pitch_sgn = 0;
		_ui.checkRxPitchSignal->setChecked(false);
		_ui.lineRxPitchSignal->setText("");
	}

	if (_ui.checkTxCmdThrottle->isChecked())
	{
		header.Bits.cmd_thr = 1;
	}
	else
	{
		header.Bits.cmd_thr = 0;
		_ui.checkRxCmdThrottle->setChecked(false);
		_ui.lineRxCmdThrottle->setText("");
	}

	if (_ui.checkTxCmdPitch->isChecked())
	{
		header.Bits.cmd_pitch = 1;
	}
	else
	{
		header.Bits.cmd_pitch = 0;
		_ui.checkRxCmdPitch->setChecked(false);
		_ui.lineRxCmdPitch->setText("");
	}

	if (_ui.checkTxCmdRoll->isChecked())
	{
		header.Bits.cmd_roll = 1;
	}
	else
	{
		header.Bits.cmd_roll = 0;
		_ui.checkRxCmdRoll->setChecked(false);
		_ui.lineRxCmdRoll->setText("");
	}

	if (_ui.checkTxBodyPitch->isChecked())
	{
		header.Bits.body_pitch = 1;
	}
	else
	{
		header.Bits.body_pitch = 0;
		_ui.checkRxBodyPitch->setChecked(false);
		_ui.lineRxBodyPitch->setText("");
	}

	if (_ui.checkTxBodyRoll->isChecked())
	{
		header.Bits.body_roll = 1;
	}
	else
	{
		header.Bits.body_roll = 0;
		_ui.checkRxBodyRoll->setChecked(false);
		_ui.lineRxBodyRoll->setText("");
	}

	if (_ui.checkTxBodyYaw->isChecked())
	{
		header.Bits.body_yaw = 1;
	}
	else
	{
		header.Bits.body_yaw = 0;
		_ui.checkRxBodyYaw->setChecked(false);
		_ui.lineRxBodyYaw->setText("");
	}

	if (_ui.checkTxRollPidErr->isChecked())
	{
		header.Bits.roll_pid_err = 1;
	}
	else
	{
		header.Bits.roll_pid_err = 0;
		_ui.checkRxRollPidErr->setChecked(false);
		_ui.lineRxRollPidErr->setText("");
	}

	if (_ui.checkTxRollPidP->isChecked())
	{
		header.Bits.roll_pid_p = 1;
	}
	else
	{
		header.Bits.roll_pid_p = 0;
		_ui.checkRxRollPidP->setChecked(false);
		_ui.lineRxRollPidP->setText("");
	}

	if (_ui.checkTxRollPidI->isChecked())
	{
		header.Bits.roll_pid_i = 1;
	}
	else
	{
		header.Bits.roll_pid_i = 0;
		_ui.checkRxRollPidI->setChecked(false);
		_ui.lineRxRollPidI->setText("");
	}

	if (_ui.checkTxRollPidD->isChecked())
	{
		header.Bits.roll_pid_d = 1;
	}
	else
	{
		header.Bits.roll_pid_d = 0;
		_ui.checkRxRollPidD->setChecked(false);
		_ui.lineRxRollPidD->setText("");
	}

	if (_ui.checkTxRollPidU->isChecked())
	{
		header.Bits.roll_pid_u = 1;
	}
	else
	{
		header.Bits.roll_pid_u = 0;
		_ui.checkRxRollPidU->setChecked(false);
		_ui.lineRxRollPidU->setText("");
	}

	if (_ui.checkTxPitchPidErr->isChecked())
	{
		header.Bits.pitch_pid_err = 1;
	}
	else
	{
		header.Bits.pitch_pid_err = 0;
		_ui.checkRxPitchPidErr->setChecked(false);
		_ui.lineRxPitchPidErr->setText("");
	}

	if (_ui.checkTxPitchPidP->isChecked())
	{
		header.Bits.pitch_pid_p = 1;
	}
	else
	{
		header.Bits.pitch_pid_p = 0;
		_ui.checkRxPitchPidP->setChecked(false);
		_ui.lineRxPitchPidP->setText("");
	}

	if (_ui.checkTxPitchPidI->isChecked())
	{
		header.Bits.pitch_pid_i = 1;
	}
	else
	{
		header.Bits.pitch_pid_i = 0;
		_ui.checkRxPitchPidI->setChecked(false);
		_ui.lineRxPitchPidI->setText("");
	}

	if (_ui.checkTxPitchPidD->isChecked())
	{
		header.Bits.pitch_pid_d = 1;
	}
	else
	{
		header.Bits.pitch_pid_d = 0;
		_ui.checkRxPitchPidD->setChecked(false);
		_ui.lineRxPitchPidD->setText("");
	}

	if (_ui.checkTxPitchPidU->isChecked())
	{
		header.Bits.pitch_pid_u = 1;
	}
	else
	{
		header.Bits.pitch_pid_u = 0;
		_ui.checkRxPitchPidU->setChecked(false);
		_ui.lineRxPitchPidU->setText("");
	}

	if (_ui.checkTxYawPidErr->isChecked())
	{
		header.Bits.yaw_pid_err = 1;
	}
	else
	{
		header.Bits.yaw_pid_err = 0;
		_ui.checkRxYawPidErr->setChecked(false);
		_ui.lineRxYawPidErr->setText("");
	}

	if (_ui.checkTxYawPidP->isChecked())
	{
		header.Bits.yaw_pid_p = 1;
	}
	else
	{
		header.Bits.yaw_pid_p = 0;
		_ui.checkRxYawPidP->setChecked(false);
		_ui.lineRxYawPidP->setText("");
	}

	if (_ui.checkTxYawPidI->isChecked())
	{
		header.Bits.yaw_pid_i = 1;
	}
	else
	{
		header.Bits.yaw_pid_i = 0;
		_ui.checkRxYawPidI->setChecked(false);
		_ui.lineRxYawPidI->setText("");
	}

	if (_ui.checkTxYawPidD->isChecked())
	{
		header.Bits.yaw_pid_d = 1;
	}
	else
	{
		header.Bits.yaw_pid_d = 0;
		_ui.checkRxYawPidD->setChecked(false);
		_ui.lineRxYawPidD->setText("");
	}

	if (_ui.checkTxYawPidU->isChecked())
	{
		header.Bits.yaw_pid_u = 1;
	}
	else
	{
		header.Bits.yaw_pid_u = 0;
		_ui.checkRxYawPidU->setChecked(false);
		_ui.lineRxYawPidU->setText("");
	}

	if (_ui.checkTxMotor1Signal->isChecked())
	{
		header.Bits.motor1 = 1;
	}
	else
	{
		header.Bits.motor1 = 0;
		_ui.checkRxMotor1Signal->setChecked(false);
		_ui.lineRxMotor1Signal->setText("");
	}

	if (_ui.checkTxMotor2Signal->isChecked())
	{
		header.Bits.motor2 = 1;
	}
	else
	{
		header.Bits.motor2 = 0;
		_ui.checkRxMotor2Signal->setChecked(false);
		_ui.lineRxMotor2Signal->setText("");
	}

	if (_ui.checkTxMotor3Signal->isChecked())
	{
		header.Bits.motor3 = 1;
	}
	else
	{
		header.Bits.motor3 = 0;
		_ui.checkRxMotor3Signal->setChecked(false);
		_ui.lineRxMotor3Signal->setText("");
	}

	if (_ui.checkTxMotor4Signal->isChecked())
	{
		header.Bits.motor4 = 1;
	}
	else
	{
		header.Bits.motor4 = 0;
		_ui.checkRxMotor4Signal->setChecked(false);
		_ui.lineRxMotor4Signal->setText("");
	}

	if (_ui.checkTxMotorsArmed->isChecked())
	{
		header.Bits.motors_armed = 1;
	}
	else
	{
		header.Bits.motors_armed = 0;
		_ui.checkRxMotorsArmed->setChecked(false);
		_ui.lineRxMotorsArmed->setText("");
	}

	if (_ui.checkTxCbit->isChecked())
	{
		header.Bits.cbit = 1;
	}
	else
	{
		header.Bits.cbit = 0;
		_ui.checkRxCbit->setChecked(false);
		_ui.lineRxCbit->setText("");
	}

	if (_ui.checkTxI2CRead->isChecked())
	{
		header.Bits.i2c_read = 1;
	}
	else
	{
		header.Bits.i2c_read = 0;
		_ui.checkRxI2CRead->setChecked(false);
		_ui.lineRxI2CRead->setText("");
	}

	if (_ui.checkTxSwVer->isChecked())
	{
		header.Bits.sw_ver = 1;
	}
	else
	{
		header.Bits.sw_ver = 0;
		_ui.checkRxSwVer->setChecked(false);
		_ui.lineRxSwVer->setText("");
	}

	if (_ui.checkTxImuOffset->isChecked())
	{
		header.Bits.imu_offset = 1;
	}
	else
	{
		header.Bits.imu_offset = 0;

		_ui.checkRxRollOffset->setChecked(false);
		_ui.checkRxPitchOffset->setChecked(false);

		_ui.lineRxRollOffset->setText("");
		_ui.lineRxPitchOffset->setText("");
	}

	if (_ui.checkTxMotorParams->isChecked())
	{
		header.Bits.motor_params = 1;
	}
	else
	{
		header.Bits.motor_params = 0;
	}

	if (_ui.checkTxJsParams->isChecked())
	{
		header.Bits.js_params = 1;
	}
	else
	{
		header.Bits.js_params = 0;
	}

	if (_ui.checkTxPidParams->isChecked())
	{
		header.Bits.pid_params = 1;
	}
	else
	{
		header.Bits.pid_params = 0;
	}

	if (_ui.checkTxPtf1Params->isChecked())
	{
		header.Bits.ptf1_params = 1;
	}
	else
	{
		header.Bits.ptf1_params = 0;
	}

	if (_ui.checkTxImuType->isChecked())
	{
		header.Bits.imu_type = 1;
	}
	else
	{
		header.Bits.imu_type = 0;
	}

	if (_ui.checkTxThrottleParams->isChecked())
	{
		header.Bits.throttle_params = 1;
	}
	else
	{
		header.Bits.throttle_params = 0;
	}


	if (_maintHandler)
	{
		_maintHandler->UpdateGetMessageHeader(header);
	}
}


void MaintenanceWindow::OnBtnSendSetMotors()
{
	if (_maintHandler)
	{
		uint16_t data = static_cast<uint16_t>(_ui.spinSetMotorsValue->value() & 0xFFFF);

		int motorNo = _ui.comboSetMotors->currentIndex() + 1;

		if (motorNo < 6)
		{
			_maintHandler->TxSetMotors(motorNo, data);
		}
		else
		{
			_maintHandler->TxControlMotors(data > 0);
		}
	}
}


void MaintenanceWindow::OnBtnI2CRead()
{
	if (_maintHandler)
	{
		uint32_t i2c = _ui.comboSetI2cChannel->currentIndex();
		uint32_t addr = _ui.lineSetI2cAddress->text().toUInt(nullptr, 16) & 0xFF;
		uint32_t reg = _ui.lineSetI2cRegister->text().toUInt(nullptr, 16) & 0xFF;

		_ui.lineSetI2cAddress->setText(QString::number(addr, 16));
		_ui.lineSetI2cRegister->setText(QString::number(reg, 16));

		_maintHandler->I2CRead(i2c, addr, reg);
	}
}


void MaintenanceWindow::OnBtnI2CWrite()
{
	if (_maintHandler)
	{
		uint32_t i2c = _ui.comboSetI2cChannelW->currentIndex();
		uint32_t addr = _ui.lineSetI2cAddressW->text().toUInt(nullptr, 16) & 0xFF;
		uint32_t reg = _ui.lineSetI2cRegisterW->text().toUInt(nullptr, 16) & 0xFF;
		uint32_t val = _ui.lineSetI2cValW->text().toUInt(nullptr, 16) & 0xFF;

		_ui.lineSetI2cAddressW->setText(QString::number(addr, 16));
		_ui.lineSetI2cRegisterW->setText(QString::number(reg, 16));
		_ui.lineSetI2cValW->setText(QString::number(val, 16));

		_maintHandler->I2CWrite(i2c, addr, reg, val);
	}
}


void MaintenanceWindow::OnBtnTxImuOffset()
{
	if (_maintHandler)
	{
		float roll_offset = _ui.spinRollOffset->value();
		float pitch_offset = _ui.spinPitchOffset->value();

		_maintHandler->TxImuOffset(roll_offset, pitch_offset);
	}
}


void MaintenanceWindow::OnBtnImuAutoOffset()
{
	float auto_offset_roll = _rxRollOffset + _rxRoll;
	float auto_offset_pitch = _rxPitchOffset + _rxPitch;

	_ui.spinRollOffset->setValue(auto_offset_roll);
	_ui.spinPitchOffset->setValue(auto_offset_pitch);

	OnBtnTxImuOffset();
}


void MaintenanceWindow::OnBtnSendThrottleParams()
{
	uint16_t descend = _ui.spinDescend->value();
	uint16_t hovering = _ui.spinHovering->value();
	uint16_t climb = _ui.spinClimb->value();

	if (_maintHandler)
	{
		_maintHandler->TxThrottleParams(descend, hovering, climb);
	}
}


void MaintenanceWindow::OnBtnSendMaintenanceParams()
{
	if (_maintHandler)
	{
		bool enabledParam = _ui.checkSetEnabledParam->isChecked();
		uint32_t minSignalParam = _ui.spinSetMinParam->value();
		uint32_t maxSignalParam = _ui.spinSetMaxParam->value();
		
		_maintHandler->TxMotorParams(_ui.comboSetParamId->currentIndex() + 1, enabledParam, minSignalParam, maxSignalParam);
	}
}

void MaintenanceWindow::OnBtnSendJsParams()
{
	if (_maintHandler)
	{
		float alpha = _ui.spinSetJsAlpha->value();
		float beta = _ui.spinSetJsBeta->value();

		_maintHandler->TxJoystickParams(_ui.comboSetJsParam->currentIndex() + 1, alpha, beta);
	}
}


void MaintenanceWindow::OnBtnSendPidParams()
{
	if (_maintHandler)
	{
		float kp = _ui.spinSetKp->value();
		float ki = _ui.spinSetKi->value();
		float kt = _ui.spinSetKt->value();
		float sat = _ui.spinSetSat->value();
		float ad = _ui.spinSetAd->value();
		float bd = _ui.spinSetBd->value();

		_maintHandler->TxPidParams(_ui.comboSetPidParam->currentIndex() + 1, kp, ki, kt, sat, ad, bd);
	}
}

void MaintenanceWindow::OnBtnSendPtf1Params()
{
	if (_maintHandler)
	{
		float x = _ui.spinSetPtf1X->value();
		float y = _ui.spinSetPtf1Y->value();
		float z = _ui.spinSetPtf1Z->value();

		_maintHandler->TxPtf1params(_ui.comboSetPtf1Param->currentIndex() + 1, x, y, z);
	}
}


void MaintenanceWindow::OnBtnSendImuType()
{
	if (_maintHandler)
	{
		Maint::IMU_TYPE imuType = Maint::IMU_TYPE(_ui.comboSetImuType->currentData().toInt());

		_maintHandler->TxImuType(imuType);
	}
}


void MaintenanceWindow::OnBtnFlashWrite()
{
	if (_maintHandler)
	{
		_maintHandler->TxWriteToFlash();
	}
}


void MaintenanceWindow::OnBtnRefreshParams()
{
	int currentMotor = _ui.comboSetParamId->currentIndex() + 1;
	int currentJsChan = _ui.comboSetJsParam->currentIndex() + 1;
	int currentPidAngle = _ui.comboSetPidParam->currentIndex() + 1;
	int currentPtf1Source = _ui.comboSetPtf1Param->currentIndex() + 1;

	_ui.checkSetEnabledParam->setChecked(_rxMotorParams[currentMotor].enabled);
	_ui.spinSetMinParam->setValue(_rxMotorParams[currentMotor].min_signal);
	_ui.spinSetMaxParam->setValue(_rxMotorParams[currentMotor].max_signal);

	_ui.spinSetJsAlpha->setValue(_rxJsParams[currentJsChan].alpha);
	_ui.spinSetJsBeta->setValue(_rxJsParams[currentJsChan].beta);

	_ui.spinSetKp->setValue(_rxPidParams[currentPidAngle].kp);
	_ui.spinSetKi->setValue(_rxPidParams[currentPidAngle].ki);
	_ui.spinSetKt->setValue(_rxPidParams[currentPidAngle].kt);
	_ui.spinSetSat->setValue(_rxPidParams[currentPidAngle].sat);
	_ui.spinSetAd->setValue(_rxPidParams[currentPidAngle].ad);
	_ui.spinSetBd->setValue(_rxPidParams[currentPidAngle].bd);

	_ui.spinSetPtf1X->setValue(_rxPtf1Params[currentPtf1Source].x);
	_ui.spinSetPtf1Y->setValue(_rxPtf1Params[currentPtf1Source].y);
	_ui.spinSetPtf1Z->setValue(_rxPtf1Params[currentPtf1Source].z);

	_ui.spinRollOffset->setValue(_rxRollOffset);
	_ui.spinPitchOffset->setValue(_rxPitchOffset);

	for (int i = 0; i < _ui.comboSetImuType->count(); i++)
	{
		if (Maint::IMU_TYPE(_ui.comboSetImuType->itemData(i).toInt()) == _rxImuType)
		{
			_ui.comboSetImuType->setCurrentIndex(i);
			break;
		}
	}

	_ui.spinDescend->setValue(_rxThrottleParams[0]);
	_ui.spinHovering->setValue(_rxThrottleParams[1]);
	_ui.spinClimb->setValue(_rxThrottleParams[2]);
	
}


void MaintenanceWindow::OnSpinSetMotorsValue(int newValue)
{
	if (_maintHandler && _ui.checkSetMotorsOnChange->isChecked())
	{
		int motorNo = _ui.comboSetMotors->currentIndex() + 1;
		uint16_t data = static_cast<uint16_t>(newValue & 0xFFFF);

		if (motorNo < 6)
		{
			_maintHandler->TxSetMotors(motorNo, newValue);
		}
		else
		{
			_maintHandler->TxControlMotors(newValue > 0);
		}
	}
}


void MaintenanceWindow::OnRxRawData(bool valid, quint8* data, int size)
{
	int64_t curMillis = QDateTime::currentMSecsSinceEpoch();

	QString dataString("0xFF ");

	for (int i = 0; i < size; i++)
	{
		QString hex = QString::number(data[i], 16).toUpper();

		if (hex.length() == 1)
		{
			hex = "0" + hex;
		}

		dataString += "0x" + hex + " ";
	}

	_ui.textEditRxData->setText(dataString);

	if (!valid)
	{
		_ui.lblRxData->setStyleSheet("background-color: #FFFF00");
	}
	else
	{
		_ui.lblRxData->setStyleSheet("background-color: #00FF00");

		_rxCounter += 1;
		_ui.lblRxCount->setText(QString("Count: %1").arg(_rxCounter));
		checkPlot("RX_COUNT", _rxCounter);

		if (_rxT0millis < 0)
		{
			_rxT0millis = curMillis;
		}
		else
		{
			qint64 delta_t_millis = (curMillis - _rxT0millis);
			double dt_seconds = static_cast<double>(delta_t_millis) * 1e-3;
			double freq = static_cast<double>(_rxCounter) / dt_seconds;

			_ui.lblRxFreq->setText(QString("Frequency: %1 Hz").arg(freq));
			checkPlot("FREQUENCY", freq);
		}
	}
}


void MaintenanceWindow::OnTxRawData(quint8* data, int size)
{
	static qint64 lastTxTime = 0;
	qint64 currTxTime = QDateTime::currentMSecsSinceEpoch();

	if (size > 0)
	{
		_ui.lblTxData->setStyleSheet("background-color: #00FF00");
		QString dataString;
		for (int i = 0; i < size; i++)
		{
			QString hex = QString::number(data[i], 16).toUpper();

			if (hex.length() == 1)
			{
				hex = "0" + hex;
			}

			dataString += "0x" + hex + " ";
		}

		_ui.textEditTxData->setText(dataString);
	}

	if (lastTxTime == 0)
	{
		lastTxTime = currTxTime;
	}
	else
	{
		qint64 delta = currTxTime - lastTxTime;
		lastTxTime = size > 0 ? currTxTime : lastTxTime;

		if (delta > qint64(2 * _txDelayMillis))
		{
			_ui.lblTxData->setStyleSheet("background-color: #FF0000");
			lastTxTime = 0;
		}
	}

}


void MaintenanceWindow::OnReceivedRawAccelX(float data)
{
	_ui.checkRxRawAccX->setChecked(true);
	_ui.lineRxRawAccX->setText(QString::number(data));

	checkPlot("RAW_ACC_X", data);
}


void MaintenanceWindow::OnReceivedRawAccelY(float data)
{
	_ui.checkRxRawAccY->setChecked(true);
	_ui.lineRxRawAccY->setText(QString::number(data));

	checkPlot("RAW_ACC_Y", data);
}


void MaintenanceWindow::OnReceivedRawAccelZ(float data)
{
	_ui.checkRxRawAccZ->setChecked(true);
	_ui.lineRxRawAccZ->setText(QString::number(data));

	checkPlot("RAW_ACC_Z", data);
}


void MaintenanceWindow::OnReceivedRawGyroX(float data)
{
	_ui.checkRxRawGyroX->setChecked(true);
	_ui.lineRxRawGyroX->setText(QString::number(data));

	checkPlot("RAW_GYRO_X", data);
}


void MaintenanceWindow::OnReceivedRawGyroY(float data)
{
	_ui.checkRxRawGyroY->setChecked(true);
	_ui.lineRxRawGyroY->setText(QString::number(data));

	checkPlot("RAW_GYRO_Y", data);
}


void MaintenanceWindow::OnReceivedRawGyroZ(float data)
{
	_ui.checkRxRawGyroZ->setChecked(true);
	_ui.lineRxRawGyroZ->setText(QString::number(data));

	checkPlot("RAW_GYRO_Z", data);
}


void MaintenanceWindow::OnReceivedRawMagnX(float data)
{
	_ui.checkRxRawMagnX->setChecked(true);
	_ui.lineRxRawMagnX->setText(QString::number(data));

	checkPlot("RAW_MAGN_X", data);
}


void MaintenanceWindow::OnReceivedRawMagnY(float data)
{
	_ui.checkRxRawMagnY->setChecked(true);
	_ui.lineRxRawMagnY->setText(QString::number(data));

	checkPlot("RAW_MAGN_Y", data);
}


void MaintenanceWindow::OnReceivedRawMagnZ(float data)
{
	_ui.checkRxRawMagnZ->setChecked(true);
	_ui.lineRxRawMagnZ->setText(QString::number(data));

	checkPlot("RAW_MAGN_Z", data);
}


void MaintenanceWindow::OnReceivedFilteredAccelX(float data)
{
	_ui.checkRxFiltAccX->setChecked(true);
	_ui.lineRxFiltAccX->setText(QString::number(data));

	checkPlot("FILTERED_ACC_X", data);
}


void MaintenanceWindow::OnReceivedFilteredAccelY(float data)
{
	_ui.checkRxFiltAccY->setChecked(true);
	_ui.lineRxFiltAccY->setText(QString::number(data));

	checkPlot("FILTERED_ACC_Y", data);
}


void MaintenanceWindow::OnReceivedFilteredAccelZ(float data)
{
	_ui.checkRxFiltAccZ->setChecked(true);
	_ui.lineRxFiltAccZ->setText(QString::number(data));

	checkPlot("FILTERED_ACC_Z", data);
}


void MaintenanceWindow::OnReceivedFilteredGyroX(float data)
{
	_ui.checkRxFiltGyroX->setChecked(true);
	_ui.lineRxFiltGyroX->setText(QString::number(data));

	checkPlot("FILTERED_GYRO_X", data);
}


void MaintenanceWindow::OnReceivedFilteredGyroY(float data)
{
	_ui.checkRxFiltGyroY->setChecked(true);
	_ui.lineRxFiltGyroY->setText(QString::number(data));

	checkPlot("FILTERED_GYRO_Y", data);
}


void MaintenanceWindow::OnReceivedFilteredGyroZ(float data)
{
	_ui.checkRxFiltGyroZ->setChecked(true);
	_ui.lineRxFiltGyroZ->setText(QString::number(data));

	checkPlot("FILTERED_GYRO_Z", data);
}


void MaintenanceWindow::OnReceivedFilteredMagnX(float data)
{
	_ui.checkRxFiltMagnX->setChecked(true);
	_ui.lineRxFiltMagnX->setText(QString::number(data));

	checkPlot("FILTERED_MAGN_X", data);
}


void MaintenanceWindow::OnReceivedFilteredMagnY(float data)
{
	_ui.checkRxFiltMagnY->setChecked(true);
	_ui.lineRxFiltMagnY->setText(QString::number(data));

	checkPlot("FILTERED_MAGN_Y", data);
}


void MaintenanceWindow::OnReceivedFilteredMagnZ(float data)
{
	_ui.checkRxFiltMagnZ->setChecked(true);
	_ui.lineRxFiltMagnZ->setText(QString::number(data));

	checkPlot("FILTERED_MAGN_Z", data);
}


void MaintenanceWindow::OnReceivedThrottleSgn(uint16_t data)
{
	_ui.checkRxThrottleSignal->setChecked(true);
	_ui.lineRxThrottleSignal->setText(QString::number(data));

	checkPlot("THROTTLE_SIGNAL", data);
}


void MaintenanceWindow::OnReceivedRollSgn(uint16_t data)
{
	_ui.checkRxRollSignal->setChecked(true);
	_ui.lineRxRollSignal->setText(QString::number(data));

	checkPlot("ROLL_SIGNAL", data);
}


void MaintenanceWindow::OnReceivedPitchSgn(uint16_t data)
{
	_ui.checkRxPitchSignal->setChecked(true);
	_ui.lineRxPitchSignal->setText(QString::number(data));

	checkPlot("PITCH_SIGNAL", data);
}


void MaintenanceWindow::OnReceivedCmdThr(uint16_t data)
{
	_ui.checkRxCmdThrottle->setChecked(true);
	_ui.lineRxCmdThrottle->setText(QString::number(data));

	checkPlot("CMD_THROTTLE", data);
}


void MaintenanceWindow::OnReceivedCmdRoll(float data)
{
	_ui.checkRxCmdRoll->setChecked(true);
	_ui.lineRxCmdRoll->setText(QString::number(data));

	checkPlot("CMD_ROLL", data);
}


void MaintenanceWindow::OnReceivedCmdPitch(float data)
{
	_ui.checkRxCmdPitch->setChecked(true);
	_ui.lineRxCmdPitch->setText(QString::number(data));

	checkPlot("CMD_PITCH", data);
}

#include <cmath>
static void rotateRollPitch(double roll, double pitch, double yaw, double& newRoll, double& newPitch, double& newYaw) {
	// Angolo di rotazione in radianti (45°)
	const double angleZ = M_PI / 4.0;

	// Calcola seno e coseno dell'angolo
	double cosZ = std::cos(angleZ);
	double sinZ = std::sin(angleZ);

	// Ruota roll e pitch rispetto all'asse Z
	newRoll = roll * cosZ - pitch * sinZ;
	newPitch = roll * sinZ + pitch * cosZ;

	// Lo yaw rimane invariato
	newYaw = yaw;
}


void MaintenanceWindow::OnReceivedBodyRoll(float data)
{
	_rxRoll = data;

	double yaw = 0.0f;
	double newRoll = 0.0f;
	double newPitch = 0.0f;
	double newYaw = 0.0f;
	rotateRollPitch(_rxRoll, _rxPitch, yaw, newRoll, newPitch, newYaw);
	//_rxPitch = newPitch;

	_ui.checkRxBodyRoll->setChecked(true);
	_ui.lineRxBodyRoll->setText(QString::number(newRoll));
	_ui.pfdRollPitch->UpdateRoll(newRoll);

	checkPlot("BODY_ROLL", newRoll);
}


void MaintenanceWindow::OnReceivedBodyPitch(float data)
{
	_rxPitch = data;

	double yaw = 0.0f;
	double newRoll = 0.0f;
	double newPitch = 0.0f;
	double newYaw = 0.0f;
	rotateRollPitch(_rxRoll, _rxPitch, yaw, newRoll, newPitch, newYaw);

	_ui.checkRxBodyPitch->setChecked(true);
	_ui.lineRxBodyPitch->setText(QString::number(newPitch));
	_ui.pfdRollPitch->UpdatePitch(newPitch);

	checkPlot("BODY_PITCH", newPitch);
}


void MaintenanceWindow::OnReceivedBodyYaw(float data)
{
	_ui.checkRxBodyYaw->setChecked(true);
	_ui.lineRxBodyYaw->setText(QString::number(data));
	_ui.pfdHeading->UpdateHeading(data);

	checkPlot("BODY_YAW", data);
}


void MaintenanceWindow::OnReceivedRollPidErr(float data)
{
	_ui.checkRxRollPidErr->setChecked(true);
	_ui.lineRxRollPidErr->setText(QString::number(data));

	checkPlot("ROLL_PID_ERR", data);
}

void MaintenanceWindow::OnReceivedRollPidP(float data)
{
	_ui.checkRxRollPidP->setChecked(true);
	_ui.lineRxRollPidP->setText(QString::number(data));

	checkPlot("ROLL_PID_P", data);
}

void MaintenanceWindow::OnReceivedRollPidI(float data)
{
	_ui.checkRxRollPidI->setChecked(true);
	_ui.lineRxRollPidI->setText(QString::number(data));

	checkPlot("ROLL_PID_I", data);
}

void MaintenanceWindow::OnReceivedRollPidD(float data)
{
	_ui.checkRxRollPidD->setChecked(true);
	_ui.lineRxRollPidD->setText(QString::number(data));

	checkPlot("ROLL_PID_D", data);
}

void MaintenanceWindow::OnReceivedRollPidU(float data)
{
	_ui.checkRxRollPidU->setChecked(true);
	_ui.lineRxRollPidU->setText(QString::number(data));

	checkPlot("ROLL_PID_U", data);
}

void MaintenanceWindow::OnReceivedPitchPidErr(float data)
{
	_ui.checkRxPitchPidErr->setChecked(true);
	_ui.lineRxPitchPidErr->setText(QString::number(data));

	checkPlot("PITCH_PID_ERR", data);
}

void MaintenanceWindow::OnReceivedPitchPidP(float data)
{
	_ui.checkRxPitchPidP->setChecked(true);
	_ui.lineRxPitchPidP->setText(QString::number(data));

	checkPlot("PITCH_PID_P", data);
}

void MaintenanceWindow::OnReceivedPitchPidI(float data)
{
	_ui.checkRxPitchPidI->setChecked(true);
	_ui.lineRxPitchPidI->setText(QString::number(data));

	checkPlot("PITCH_PID_I", data);
}

void MaintenanceWindow::OnReceivedPitchPidD(float data)
{
	_ui.checkRxPitchPidD->setChecked(true);
	_ui.lineRxPitchPidD->setText(QString::number(data));

	checkPlot("PITCH_PID_D", data);
}

void MaintenanceWindow::OnReceivedPitchPidU(float data)
{
	_ui.checkRxPitchPidU->setChecked(true);
	_ui.lineRxPitchPidU->setText(QString::number(data));

	checkPlot("PITCH_PID_U", data);
}

void MaintenanceWindow::OnReceivedYawPidErr(float data)
{
	_ui.checkRxYawPidErr->setChecked(true);
	_ui.lineRxYawPidErr->setText(QString::number(data));

	checkPlot("YAW_PID_ERR", data);
}

void MaintenanceWindow::OnReceivedYawPidP(float data)
{
	_ui.checkRxYawPidP->setChecked(true);
	_ui.lineRxYawPidP->setText(QString::number(data));

	checkPlot("YAW_PID_P", data);
}

void MaintenanceWindow::OnReceivedYawPidI(float data)
{
	_ui.checkRxYawPidI->setChecked(true);
	_ui.lineRxYawPidI->setText(QString::number(data));

	checkPlot("YAW_PID_I", data);
}

void MaintenanceWindow::OnReceivedYawPidD(float data)
{
	_ui.checkRxYawPidD->setChecked(true);
	_ui.lineRxYawPidD->setText(QString::number(data));

	checkPlot("YAW_PID_D", data);
}

void MaintenanceWindow::OnReceivedYawPidU(float data)
{
	_ui.checkRxYawPidU->setChecked(true);
	_ui.lineRxYawPidU->setText(QString::number(data));

	checkPlot("YAW_PID_U", data);
}

void MaintenanceWindow::OnReceivedMotor1(uint16_t data)
{
	_ui.checkRxMotor1Signal->setChecked(true);
	_ui.lineRxMotor1Signal->setText(QString::number(data));

	checkPlot("MOTOR_1", data);
}


void MaintenanceWindow::OnReceivedMotor2(uint16_t data)
{
	_ui.checkRxMotor2Signal->setChecked(true);
	_ui.lineRxMotor2Signal->setText(QString::number(data));

	checkPlot("MOTOR_2", data);
}


void MaintenanceWindow::OnReceivedMotor3(uint16_t data)
{
	_ui.checkRxMotor3Signal->setChecked(true);
	_ui.lineRxMotor3Signal->setText(QString::number(data));

	checkPlot("MOTOR_3", data);
}


void MaintenanceWindow::OnReceivedMotor4(uint16_t data)
{
	_ui.checkRxMotor4Signal->setChecked(true);
	_ui.lineRxMotor4Signal->setText(QString::number(data));

	checkPlot("MOTOR_4", data);
}


void MaintenanceWindow::OnReceivedMotorsArmed(uint8_t data)
{
	_ui.checkRxMotorsArmed->setChecked(true);
	_ui.lineRxMotorsArmed->setText(QString::number(data));

	checkPlot("MOTORS_ARMED", data);
}


void MaintenanceWindow::OnReceivedCbit(uint32_t data)
{
	Maint::CBIT_TAG* cbit = reinterpret_cast<Maint::CBIT_TAG*>(&data);

	_ui.checkRxCbit->setChecked(true);
	_ui.lineRxCbit->setText(QString::number(data));
}



void MaintenanceWindow::OnReceivedMotorsParams(uint32_t motor_no, bool enabled, uint16_t min_signal, uint16_t max_signal)
{
	_ui.checkTxMotorParams->setChecked(false);

	if (_rxMotorParams.keys().indexOf(motor_no) != -1)
	{
		_rxMotorParams[motor_no] = { enabled, min_signal, max_signal };
	}
}

void MaintenanceWindow::OnReceivedJsParams(uint32_t channel_no, float alpha, float beta)
{
	_ui.checkTxJsParams->setChecked(false);

	if (_rxJsParams.keys().indexOf(channel_no) != -1)
	{
		_rxJsParams[channel_no] = { alpha, beta };
	}
}

void MaintenanceWindow::OnReceivedPidParams(uint32_t angle_no, float kp, float ki, float kt, float sat, float ad, float bd)
{
	_ui.checkTxPidParams->setChecked(false);

	if (_rxPidParams.keys().indexOf(angle_no) != -1)
	{
		_rxPidParams[angle_no] = { kp, ki, kt, sat, ad, bd };
	}
}

void MaintenanceWindow::OnReceivedPtf1Params(uint32_t source_no, float x, float y, float z)
{
	_ui.checkTxPtf1Params->setChecked(false);

	if (_rxPtf1Params.keys().indexOf(source_no) != -1)
	{
		_rxPtf1Params[source_no] = { x, y, z };
	}
}


void MaintenanceWindow::OnReceivedImuType(uint8_t imu_type)
{
	_rxImuType = Maint::IMU_TYPE(imu_type);
	_ui.lineSetI2cAddress->setText(QString::number(_imuTypeToI2CAddr[_rxImuType], 16));
	_ui.lineSetI2cAddressW->setText(QString::number(_imuTypeToI2CAddr[_rxImuType], 16));
}


void MaintenanceWindow::OnReceivedI2CRead(uint8_t imu_type)
{
	_ui.checkRxI2CRead->setChecked(true);
	_ui.lineRxI2CRead->setText(QString::number(imu_type, 16));
}


void MaintenanceWindow::OnReceivedSwVer(uint8_t major_v, uint8_t minor_v, uint8_t stage_v, uint8_t rel_type)
{
	_ui.checkRxSwVer->setChecked(true);
	_ui.lineRxSwVer->setText(QString("%1.%2.%3-%4").arg(major_v).arg(minor_v).arg(stage_v).arg(
		Maint::REL_TYPE(rel_type) == Maint::REL_TYPE::BETA    ? "B" :
		Maint::REL_TYPE(rel_type) == Maint::REL_TYPE::RELEASE ? "R" : "?"));
}


void MaintenanceWindow::OnReceivedImuOffset(float roll_offset, float pitch_offset)
{
	_ui.checkRxRollOffset->setChecked(true);
	_ui.checkRxPitchOffset->setChecked(true);

	_rxRollOffset = roll_offset;
	_rxPitchOffset = pitch_offset;

	_ui.lineRxRollOffset->setText(QString::number(_rxRollOffset));
	_ui.lineRxPitchOffset->setText(QString::number(_rxPitchOffset));
}


void MaintenanceWindow::OnReceivedThrottleParams(uint16_t descend, uint16_t hovering, uint16_t climb)
{
	_rxThrottleParams[0] = descend;
	_rxThrottleParams[1] = hovering;
	_rxThrottleParams[2] = climb;

	_ui.checkTxThrottleParams->setChecked(false);
}


void MaintenanceWindow::OnActionOpen()
{
	QString filePath = QFileDialog::getOpenFileName(
		nullptr,
		"Open Log File",
		QDir::currentPath(),
		"Log Files (log-*.txt);;All Files (*.*)");

	if (!filePath.isEmpty())
	{
		auto _run = QtConcurrent::run([this, filePath]() {
			_maintHandler->CreateMatlabMatrix(filePath.toStdString().c_str());
			});
	}
	else {
		QMessageBox::warning(this, "Error", QString("No file selected"));
	}
}