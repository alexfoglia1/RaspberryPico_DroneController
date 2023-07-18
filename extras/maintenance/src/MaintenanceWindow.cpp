#include "MaintenanceWindow.h"
#include <qserialport.h>


MaintenanceWindow::MaintenanceWindow()
{
	_ui.setupUi(this);

	_maintHandler = nullptr;

	autoScanComPorts();

	connect(_ui.btnOpenSerialPort, SIGNAL(clicked()), this, SLOT(OnBtnOpenSerialPort()));

	connect(_ui.checkTxThrottleSignal, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxRollSignal, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxPitchSignal, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));

	connect(_ui.checkTxCmdThrottle, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxCmdRoll, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxCmdPitch, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));


	connect(_ui.checkTxMotor1Signal, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxMotor2Signal, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxMotor3Signal, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxMotor4Signal, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
	connect(_ui.checkTxMotorsArmed, SIGNAL(clicked()), this, SLOT(OnHeaderChanged()));
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


void MaintenanceWindow::OnBtnOpenSerialPort()
{
	_maintHandler = new Maint::Maintenance(_ui.comboSelPort->currentText());

	connect(_maintHandler, SIGNAL(receivedThrottleSgn(uint32_t)), this, SLOT(OnReceivedThrottleSgn(uint32_t)));
	connect(_maintHandler, SIGNAL(receivedPitchSgn(uint32_t)), this, SLOT(OnReceivedPitchSgn(uint32_t)));
	connect(_maintHandler, SIGNAL(receivedRollSgn(uint32_t)), this, SLOT(OnReceivedRollSgn(uint32_t)));
	connect(_maintHandler, SIGNAL(receivedCmdThr(float)), this, SLOT(OnReceivedCmdThr(float)));
	connect(_maintHandler, SIGNAL(receivedCmdPitch(float)), this, SLOT(OnReceivedCmdPitch(float)));
	connect(_maintHandler, SIGNAL(receivedCmdRoll(float)), this, SLOT(OnReceivedCmdRoll(float)));

	connect(_maintHandler, SIGNAL(receivedMotor1(uint32_t)), this, SLOT(OnReceivedMotor1(uint32_t)));
	connect(_maintHandler, SIGNAL(receivedMotor2(uint32_t)), this, SLOT(OnReceivedMotor2(uint32_t)));
	connect(_maintHandler, SIGNAL(receivedMotor3(uint32_t)), this, SLOT(OnReceivedMotor3(uint32_t)));
	connect(_maintHandler, SIGNAL(receivedMotor4(uint32_t)), this, SLOT(OnReceivedMotor4(uint32_t)));
	connect(_maintHandler, SIGNAL(receivedMotorsArmed(uint32_t)), this, SLOT(OnReceivedMotorsArmed(uint32_t)));


	_maintHandler->Open();
	_maintHandler->EnableTx();

	_ui.btnOpenSerialPort->setEnabled(false);
	_ui.comboSelPort->setEnabled(false);
	_ui.groupBoxTx->setEnabled(true);
	_ui.TxMaintenanceGroup->setEnabled(true);
}


void MaintenanceWindow::OnHeaderChanged()
{
	Maint::MAINT_HEADER_T header;
	header.All = 0x00;

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
		_ui.checkRxThrottleSignal->setChecked(false);
		_ui.lineRxThrottleSignal->setText("");
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

	if (_maintHandler)
	{
		_maintHandler->SetTxHeader(header);
	}

}


void MaintenanceWindow::OnReceivedThrottleSgn(uint32_t data)
{
	_ui.checkRxThrottleSignal->setChecked(true);
	_ui.lineRxThrottleSignal->setText(QString::number(data));
}


void MaintenanceWindow::OnReceivedRollSgn(uint32_t data)
{
	_ui.checkRxRollSignal->setChecked(true);
	_ui.lineRxRollSignal->setText(QString::number(data));
}


void MaintenanceWindow::OnReceivedPitchSgn(uint32_t data)
{
	_ui.checkRxPitchSignal->setChecked(true);
	_ui.lineRxPitchSignal->setText(QString::number(data));
}


void MaintenanceWindow::OnReceivedCmdThr(float data)
{
	_ui.checkRxCmdThrottle->setChecked(true);
	_ui.lineRxCmdThrottle->setText(QString::number(data));
}


void MaintenanceWindow::OnReceivedCmdRoll(float data)
{
	_ui.checkRxCmdRoll->setChecked(true);
	_ui.lineRxCmdRoll->setText(QString::number(data));
}


void MaintenanceWindow::OnReceivedCmdPitch(float data)
{
	_ui.checkRxCmdPitch->setChecked(true);
	_ui.lineRxCmdPitch->setText(QString::number(data));
}


void MaintenanceWindow::OnReceivedMotor1(uint32_t data)
{
	_ui.checkRxMotor1Signal->setChecked(true);
	_ui.lineRxMotor1Signal->setText(QString::number(data));
}


void MaintenanceWindow::OnReceivedMotor2(uint32_t data)
{
	_ui.checkRxMotor2Signal->setChecked(true);
	_ui.lineRxMotor2Signal->setText(QString::number(data));
}


void MaintenanceWindow::OnReceivedMotor3(uint32_t data)
{
	_ui.checkRxMotor3Signal->setChecked(true);
	_ui.lineRxMotor3Signal->setText(QString::number(data));
}


void MaintenanceWindow::OnReceivedMotor4(uint32_t data)
{
	_ui.checkRxMotor4Signal->setChecked(true);
	_ui.lineRxMotor4Signal->setText(QString::number(data));
}


void MaintenanceWindow::OnReceivedMotorsArmed(uint32_t data)
{
	_ui.checkRxMotorsArmed->setChecked(true);
	_ui.lineRxMotorsArmed->setText(QString::number(data));
}