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
	const int _txDelayMillis = 100;

	void autoScanComPorts();

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

	void OnRxRawData(bool valid, quint8* data, int size);
	void OnTxRawData(quint8* data, int size);

	void OnReceivedThrottleSgn(uint32_t data);
	void OnReceivedPitchSgn(uint32_t data);
	void OnReceivedRollSgn(uint32_t data);
	void OnReceivedCmdThr(float data);
	void OnReceivedCmdPitch(float data);
	void OnReceivedCmdRoll(float data);

	void OnReceivedMotor1(uint32_t data);
	void OnReceivedMotor2(uint32_t data);
	void OnReceivedMotor3(uint32_t data);
	void OnReceivedMotor4(uint32_t data);
	void OnReceivedMotorsArmed(uint32_t data);
};

#endif