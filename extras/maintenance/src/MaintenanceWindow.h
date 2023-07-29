#ifndef MAINT_WIN_H
#define MAINT_WIN_H
#include <qmainwindow.h>

#include "maint.h"
#include "ui_MaintenanceGui.h"


class MaintenanceWindow : public QMainWindow
{
	Q_OBJECT
public:
	MaintenanceWindow();

private:
	Ui_MaintenanceGui _ui;
	Maint::Maintenance* _maintHandler;
	const int _txDelayMillis = 20;
	QMap<QString, int> _defaultPlotSpan;

	void autoScanComPorts();
	void checkPlot(QString expectedText, double value);

private slots:
	void OnBtnOpenSerialPort();
	void OnPlotSliderValueChanged(int newValue);
	void OnPlotTrack1ValueChanged(int newValue);
	void OnPlotTrack2ValueChanged(int newValue);
	void OnPlotTrack3ValueChanged(int newValue);
	void OnComboTrack1TextChanged(const QString& newText);
	void OnComboTrack2TextChanged(const QString& newText);
	void OnComboTrack3TextChanged(const QString& newText);
	void OnHeaderChanged();

	void OnBtnSendMaintenanceCommand();
	void OnBtnSendMaintenanceParams();
	void OnBtnFlashWrite();
	void OnSpinSetMaintenanceValue(int newValue);

	void OnRxRawData(bool valid, quint8* data, int size);
	void OnTxRawData(quint8* data, int size);

	void OnReceivedRawAccelX(float data);
	void OnReceivedRawAccelY(float data);
	void OnReceivedRawAccelZ(float data);
	void OnReceivedRawGyroX(float data);
	void OnReceivedRawGyroY(float data);
	void OnReceivedRawGyroZ(float data);
	void OnReceivedRawMagnX(float data);
	void OnReceivedRawMagnY(float data);
	void OnReceivedRawMagnZ(float data);

	void OnReceivedFilteredAccelX(float data);
	void OnReceivedFilteredAccelY(float data);
	void OnReceivedFilteredAccelZ(float data);
	void OnReceivedFilteredGyroX(float data);
	void OnReceivedFilteredGyroY(float data);
	void OnReceivedFilteredGyroZ(float data);
	void OnReceivedFilteredMagnX(float data);
	void OnReceivedFilteredMagnY(float data);
	void OnReceivedFilteredMagnZ(float data);

	void OnReceivedThrottleSgn(uint32_t data);
	void OnReceivedPitchSgn(uint32_t data);
	void OnReceivedRollSgn(uint32_t data);
	void OnReceivedCmdThr(float data);
	void OnReceivedCmdPitch(float data);
	void OnReceivedCmdRoll(float data);

	void OnReceivedBodyRoll(float data);
	void OnReceivedBodyPitch(float data);
	void OnReceivedBodyYaw(float data);

	void OnReceivedRollPidErr(float data);
	void OnReceivedRollPidP(float data);
	void OnReceivedRollPidI(float data);
	void OnReceivedRollPidD(float data);
	void OnReceivedRollPidU(float data);
	void OnReceivedPitchPidErr(float data);
	void OnReceivedPitchPidP(float data);
	void OnReceivedPitchPidI(float data);
	void OnReceivedPitchPidD(float data);
	void OnReceivedPitchPidU(float data);
	void OnReceivedYawPidErr(float data);
	void OnReceivedYawPidP(float data);
	void OnReceivedYawPidI(float data);
	void OnReceivedYawPidD(float data);
	void OnReceivedYawPidU(float data);

	void OnReceivedMotor1(uint32_t data);
	void OnReceivedMotor2(uint32_t data);
	void OnReceivedMotor3(uint32_t data);
	void OnReceivedMotor4(uint32_t data);
	void OnReceivedMotorsArmed(uint32_t data);
	void OnReceivedCbit(uint32_t data);
};

#endif