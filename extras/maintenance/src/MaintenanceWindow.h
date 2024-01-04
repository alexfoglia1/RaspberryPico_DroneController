#ifndef MAINT_WIN_H
#define MAINT_WIN_H
#include <qmainwindow.h>
#include <QJoysticks.h>

#include "maint.h"
#include "JoystickBridge.h"
#include "ui_MaintenanceGui.h"
#include "ui_AutoscanComPortsGui.h"


typedef struct
{
	bool enabled;
	uint32_t min_signal;
	uint32_t max_signal;
} motors_params_format;

typedef struct
{
	float alpha;
	float beta;
} js_params_format;

typedef struct
{
	float kp;
	float ki;
	float kt;
	float sat;
	float ad;
	float bd;
} pid_params_format;

typedef struct
{
	float x;
	float y;
	float z;
} ptf1_params_format;

class MaintenanceWindow : public QMainWindow
{
	Q_OBJECT
public:
	MaintenanceWindow();

private:
	Ui_MaintenanceGui _ui;
	Ui_AutoscanComPortsGui _progressUi;
	JoystickBridge _js;
	Maint::Maintenance* _maintHandler;

	const int _txDelayMillis = 25;
	QMap<QString, int> _defaultPlotSpan;
	QMap<Maint::IMU_TYPE, QString> _imuTypeToString;
	QMap<Maint::IMU_TYPE, uint32_t> _imuTypeToI2CAddr;
	QMap<int, motors_params_format> _rxMotorParams;
	QMap<int, js_params_format> _rxJsParams;
	QMap<int, pid_params_format> _rxPidParams;
	QMap<int, ptf1_params_format> _rxPtf1Params;
	float _rxRollOffset;
	float _rxPitchOffset;
	float _rxRoll;
	float _rxPitch;
	Maint::IMU_TYPE _rxImuType;
	QMainWindow _autoscanProgressWindow;
	uint32_t _rxCounter;
	int64_t _rxT0millis;
	bool _jsIsControlling;
	bool _jsIsArmed;

	void autoScanComPorts();
	void checkPlot(QString expectedText, double value);

private slots:
	void OnOverrideRadio(bool radioOverride);
	void OnOverrideSignals(quint16 armedSignal, quint16 rollSignal, quint16 pitchSignal, quint16 throttleSignal);

	void OnBtnOpenSerialPort();
	void OnBtnOpenBoot();
	void OnBtnRescanPorts();
	void OnClearLogs();
	void OnPicoDownlink();
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
	void OnBtnSendJsParams();
	void OnBtnSendPidParams();
	void OnBtnSendPtf1Params();
	void OnBtnSendImuType();
	void OnBtnFlashWrite();
	void OnBtnRefreshParams();
	void OnSpinSetMaintenanceValue(int newValue);
	void OnBtnI2CRead();
	void OnBtnI2CWrite();
	void OnBtnTxImuOffset();
	void OnBtnImuAutoOffset();

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

	void OnReceivedThrottleSgn(uint16_t data);
	void OnReceivedPitchSgn(uint16_t data);
	void OnReceivedRollSgn(uint16_t data);
	void OnReceivedCmdThr(uint16_t data);
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

	void OnReceivedMotor1(uint16_t data);
	void OnReceivedMotor2(uint16_t data);
	void OnReceivedMotor3(uint16_t data);
	void OnReceivedMotor4(uint16_t data);
	void OnReceivedMotorsArmed(uint8_t data);
	void OnReceivedCbit(uint32_t data);

	void OnReceivedMotorsParams(uint32_t motor_no, bool enabled, uint16_t min_signal, uint16_t max_signal);
	void OnReceivedJsParams(uint32_t channel_no, float alpha, float beta);
	void OnReceivedPidParams(uint32_t angle_no, float kp, float ki, float kt, float sat, float ad, float bd);
	void OnReceivedPtf1Params(uint32_t source_no, float x, float y, float z);
	void OnReceivedImuType(uint8_t imu_type);
	void OnReceivedI2CRead(uint8_t i2c_read);
	void OnReceivedSwVer(uint8_t major_v, uint8_t minor_v, uint8_t stage_v, uint8_t rel_type);
	void OnReceivedImuOffset(float, float);
};

#endif