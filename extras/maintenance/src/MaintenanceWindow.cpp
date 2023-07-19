#include "MaintenanceWindow.h"
#include <qserialport.h>
#include <qdatetime.h>
#include <qmessagebox.h>


MaintenanceWindow::MaintenanceWindow()
{
	_ui.setupUi(this);

	_maintHandler = nullptr;

	_defaultPlotSpan =
	{
		{"NONE", 16000},
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
		{"ROLL_PID_ERR", 16000},
		{"ROLL_PID_P", 16000},
		{"ROLL_PID_I", 16000},
		{"ROLL_PID_D", 16000},
		{"ROLL_PID_U", 16000},
		{"PITCH_PID_ERR", 16000},
		{"PITCH_PID_P", 16000},
		{"PITCH_PID_I", 16000},
		{"PITCH_PID_D", 16000},
		{"PITCH_PID_U", 16000},
		{"YAW_PID_ERR", 16000},
		{"YAW_PID_P", 16000},
		{"YAW_PID_I", 16000},
		{"YAW_PID_D", 16000},
		{"YAW_PID_U", 16000},
		{"MOTOR_1", 4000},
		{"MOTOR_2", 4000},
		{"MOTOR_3", 4000},
		{"MOTOR_4", 4000},
		{"MOTORS_ARMED", 2}
	};

	autoScanComPorts();

	connect(_ui.btnOpenSerialPort, SIGNAL(clicked()), this, SLOT(OnBtnOpenSerialPort()));
	connect(_ui.btnSendMaintenanceCommand, SIGNAL(clicked()), this, SLOT(OnBtnSendMaintenanceCommand()));
	connect(_ui.spinSetMaintenanceValue, SIGNAL(valueChanged(int)), this, SLOT(OnSpinSetMaintenanceValue(int)));

	connect(_ui.plotTimeSlider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotSliderValueChanged(int)));
	connect(_ui.plotTrack1Slider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotTrack1ValueChanged(int)));
	connect(_ui.plotTrack2Slider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotTrack2ValueChanged(int)));
	connect(_ui.plotTrack3Slider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotTrack3ValueChanged(int)));

	connect(_ui.comboSelTrack1, SIGNAL(currentTextChanged(const QString&)), this, SLOT(OnComboTrack1TextChanged(const QString&)));
	connect(_ui.comboSelTrack2, SIGNAL(currentTextChanged(const QString&)), this, SLOT(OnComboTrack2TextChanged(const QString&)));
	connect(_ui.comboSelTrack3, SIGNAL(currentTextChanged(const QString&)), this, SLOT(OnComboTrack3TextChanged(const QString&)));

	connect(_ui.checkTxRawAccX, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxRawAccY, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxRawAccZ, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxRawGyroX, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxRawGyroY, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxRawGyroZ, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxRawMagnX, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxRawMagnY, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxRawMagnZ, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));

	connect(_ui.checkTxFiltAccX, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxFiltAccY, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxFiltAccZ, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxFiltGyroX, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxFiltGyroY, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxFiltGyroZ, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxFiltMagnX, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxFiltMagnY, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxFiltMagnZ, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));

	connect(_ui.checkTxThrottleSignal, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxRollSignal, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxPitchSignal, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));

	connect(_ui.checkTxCmdThrottle, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxCmdRoll, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxCmdPitch, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));

	connect(_ui.checkTxBodyRoll, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxBodyPitch, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxBodyYaw, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));

	connect(_ui.checkTxRollPidErr, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxRollPidP, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxRollPidI, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxRollPidD, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxRollPidU, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));

	connect(_ui.checkTxPitchPidErr, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxPitchPidP, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxPitchPidI, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxPitchPidD, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxPitchPidU, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));

	connect(_ui.checkTxYawPidErr, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxYawPidP, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxYawPidI, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxYawPidD, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxYawPidU, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));


	connect(_ui.checkTxMotor1Signal, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxMotor2Signal, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxMotor3Signal, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxMotor4Signal, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxMotorsArmed, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxCbit, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
}


void MaintenanceWindow::autoScanComPorts()
{
	for (int i = 0; i < 100; i++)
	{
		QString portName = QString("COM%1").arg(i);
		QSerialPort* serialPort = new QSerialPort(portName);
		if (serialPort->open(QSerialPort::ReadWrite))
		{
			serialPort->close();
			serialPort->deleteLater();

			_ui.comboSelPort->addItem(portName);
		}
	}
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


void MaintenanceWindow::OnBtnOpenSerialPort()
{
	_maintHandler = new Maint::Maintenance(_ui.comboSelPort->currentText());

	if (_maintHandler->Open())
	{
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

		connect(_maintHandler, SIGNAL(receivedThrottleSgn(uint32_t)), this, SLOT(OnReceivedThrottleSgn(uint32_t)));
		connect(_maintHandler, SIGNAL(receivedPitchSgn(uint32_t)), this, SLOT(OnReceivedPitchSgn(uint32_t)));
		connect(_maintHandler, SIGNAL(receivedRollSgn(uint32_t)), this, SLOT(OnReceivedRollSgn(uint32_t)));
		connect(_maintHandler, SIGNAL(receivedCmdThr(float)), this, SLOT(OnReceivedCmdThr(float)));
		connect(_maintHandler, SIGNAL(receivedCmdPitch(float)), this, SLOT(OnReceivedCmdPitch(float)));
		connect(_maintHandler, SIGNAL(receivedCmdRoll(float)), this, SLOT(OnReceivedCmdRoll(float)));

		connect(_maintHandler, SIGNAL(receivedBodyRoll(float)), this, SLOT(OnReceivedBodyRoll(float)));
		connect(_maintHandler, SIGNAL(receivedBodyPitch(float)), this, SLOT(OnReceivedBodyPitch(float)));
		connect(_maintHandler, SIGNAL(receivedBodyYaw(float)), this, SLOT(OnReceivedBodyYaw(float)));

		connect(_maintHandler, SIGNAL(receivedMotor1(uint32_t)), this, SLOT(OnReceivedMotor1(uint32_t)));
		connect(_maintHandler, SIGNAL(receivedMotor2(uint32_t)), this, SLOT(OnReceivedMotor2(uint32_t)));
		connect(_maintHandler, SIGNAL(receivedMotor3(uint32_t)), this, SLOT(OnReceivedMotor3(uint32_t)));
		connect(_maintHandler, SIGNAL(receivedMotor4(uint32_t)), this, SLOT(OnReceivedMotor4(uint32_t)));
		connect(_maintHandler, SIGNAL(receivedMotorsArmed(uint32_t)), this, SLOT(OnReceivedMotorsArmed(uint32_t)));

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

		connect(_maintHandler, SIGNAL(txRawData(quint8*, int)), this, SLOT(OnTxRawData(quint8*, int)));
		connect(_maintHandler, SIGNAL(rxRawData(bool, quint8*, int)), this, SLOT(OnRxRawData(bool, quint8*, int)));


		_maintHandler->EnableTx();

		_ui.btnOpenSerialPort->setEnabled(false);
		_ui.comboSelPort->setEnabled(false);
		_ui.groupBoxTx->setEnabled(true);
		_ui.TxMaintenanceGroup->setEnabled(true);
	}
	else
	{
		_maintHandler->deleteLater();
		QMessageBox::warning(this, "Error", QString("Cannot open serial port %1").arg(_ui.comboSelPort->currentText()));
	}
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

	if (_maintHandler)
	{
		_maintHandler->SetTxHeader(header);
	}
}


void MaintenanceWindow::OnBtnSendMaintenanceCommand()
{
	if (_maintHandler)
	{
		uint32_t data = _ui.spinSetMaintenanceValue->value();

		_maintHandler->TxMaintenanceCommand(Maint::MAINT_CMD_ID(_ui.comboSetCommandId->currentIndex() + 1), data);
	}
}


void MaintenanceWindow::OnSpinSetMaintenanceValue(int newValue)
{
	if (_maintHandler && _ui.checkSpinOnChange->isChecked())
	{
		_maintHandler->TxMaintenanceCommand(Maint::MAINT_CMD_ID(_ui.comboSetCommandId->currentIndex() + 1), newValue);
	}
}


void MaintenanceWindow::OnRxRawData(bool valid, quint8* data, int size)
{
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


void MaintenanceWindow::OnReceivedThrottleSgn(uint32_t data)
{
	_ui.checkRxThrottleSignal->setChecked(true);
	_ui.lineRxThrottleSignal->setText(QString::number(data));

	checkPlot("THROTTLE_SIGNAL", data);
}


void MaintenanceWindow::OnReceivedRollSgn(uint32_t data)
{
	_ui.checkRxRollSignal->setChecked(true);
	_ui.lineRxRollSignal->setText(QString::number(data));

	checkPlot("ROLL_SIGNAL", data);
}


void MaintenanceWindow::OnReceivedPitchSgn(uint32_t data)
{
	_ui.checkRxPitchSignal->setChecked(true);
	_ui.lineRxPitchSignal->setText(QString::number(data));

	checkPlot("PITCH_SIGNAL", data);
}


void MaintenanceWindow::OnReceivedCmdThr(float data)
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


void MaintenanceWindow::OnReceivedBodyRoll(float data)
{
	_ui.checkRxBodyRoll->setChecked(true);
	_ui.lineRxBodyRoll->setText(QString::number(data));

	checkPlot("BODY_ROLL", data);
}


void MaintenanceWindow::OnReceivedBodyPitch(float data)
{
	_ui.checkRxBodyPitch->setChecked(true);
	_ui.lineRxBodyPitch->setText(QString::number(data));

	checkPlot("BODY_PITCH", data);
}


void MaintenanceWindow::OnReceivedBodyYaw(float data)
{
	_ui.checkRxBodyYaw->setChecked(true);
	_ui.lineRxBodyYaw->setText(QString::number(data));

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

void MaintenanceWindow::OnReceivedMotor1(uint32_t data)
{
	_ui.checkRxMotor1Signal->setChecked(true);
	_ui.lineRxMotor1Signal->setText(QString::number(data));

	checkPlot("MOTOR_1", data);
}


void MaintenanceWindow::OnReceivedMotor2(uint32_t data)
{
	_ui.checkRxMotor2Signal->setChecked(true);
	_ui.lineRxMotor2Signal->setText(QString::number(data));

	checkPlot("MOTOR_2", data);
}


void MaintenanceWindow::OnReceivedMotor3(uint32_t data)
{
	_ui.checkRxMotor3Signal->setChecked(true);
	_ui.lineRxMotor3Signal->setText(QString::number(data));

	checkPlot("MOTOR_3", data);
}


void MaintenanceWindow::OnReceivedMotor4(uint32_t data)
{
	_ui.checkRxMotor4Signal->setChecked(true);
	_ui.lineRxMotor4Signal->setText(QString::number(data));

	checkPlot("MOTOR_4", data);
}


void MaintenanceWindow::OnReceivedMotorsArmed(uint32_t data)
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