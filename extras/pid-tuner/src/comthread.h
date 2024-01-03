#ifndef COMTHREAD_H
#define COMTHREAD_H

#include "proto.h"

#include <qthread.h>
#include <qserialport.h>

enum class ComThreadStatus
{
	INIT_SERIAL_PORT,
	TX_GET_STATUS,
	TX_GET_PID,
	TX_PID_ROLL,
	TX_PID_PITCH,
	TX_PID_YAW,
	TX_OVERRIDE_RADIO,
	TX_SET_ARMED,
	TX_SET_ROLL_PITCH_THROTTLE_SIGNAL,
	EXIT,
};


enum class RxStatus
{
	WAIT_SYNC,
	WAIT_HEADER,
	WAIT_PAYLOAD,
	WAIT_CHECKSUM
};


inline uint8_t checksum(uint8_t* buf, uint32_t size, bool firstSync = false)
{
	uint8_t cks = 0;
	uint32_t first = (firstSync) ? 1 : 0;
	for (uint32_t i = first; i < size; i++)
	{
		cks ^= buf[i];
	}
	return cks;
}


class ComThread : public QThread
{
	Q_OBJECT
public:
	ComThread(QObject* parent = nullptr);
	void setSerialPort(QString portName, QSerialPort::BaudRate baudRate);
	void setDelay(quint32 delay);
	void stopCom();
	void refreshPidParams();
	void writeRollPidParams(float kp, float ki, float kt, float sat, float ad, float bd);
	void writePitchPidParams(float kp, float ki, float kt, float sat, float ad, float bd);
	void writeYawPidParams(float kp, float ki, float kt, float sat, float ad, float bd);
	void overrideArmedSignal(quint16 armed_signal);
	void controlFlight(quint16 roll_signal, quint16 pitch_signal, quint16 throttle_signal);
	void overrideRadio(bool override);

protected:
	void run() override;

signals:
	void serialPortOpened();
	void serialPortClosed();
	void droneAlive();
	void droneDownlink();
	void droneAttitudeUpdate(float roll, float pitch, float yaw);
	void droneMotorsUpdate(quint32 m1, quint32 m2, quint32 m3, quint32 m4);
	void rollPidParamsUpdate(float kp, float ki, float kt, float sat, float ad, float bd);
	void pitchPidParamsUpdate(float kp, float ki, float kt, float sat, float ad, float bd);
	void yawPidParamsUpdate(float kp, float ki, float kt, float sat, float ad, float bd);

private:
	MaintenanceProtocolHdr get_status_msg;
	MaintenanceProtocolHdr get_pid_params_msg;
	MaintenanceProtocolHdr set_pid_roll_gain_msg;
	MaintenanceProtocolHdr set_pid_pitch_gain_msg;
	MaintenanceProtocolHdr set_pid_yaw_gain_msg;
	MaintenanceProtocolHdr override_radio_msg;
	MaintenanceProtocolHdr override_roll_pitch_throttle_msg;
	MaintenanceProtocolHdr override_armed_msg;

	qint64 _lastMessageFromDroneMSecs;
	quint32 _delay;
	ComThreadStatus _status;
	RxStatus _rxStatus;
	QString _serialPortName;
	QSerialPort::BaudRate _baudRate;
	QList<quint8> _txSetParams;

	quint8 _rxBuf[4192];
	quint32 _bytesReceived;
	quint32 _expectedPayloadSize;

	bool initSerialPort();
	void closeSerialPort();
	void txGet(MaintenanceProtocolHdr* hdr);
	void txSet(MaintenanceProtocolHdr* hdr);

	void checkDownlink();
	void updateFsm(quint8 byteIn);
	quint32 calculatePayloadSize(MaintenanceProtocolHdr* header);
	void dataIngest();

	void pushDataToPayload(quint8* data, int len);

	QSerialPort* _serialPort;

private slots:
	void onReadyRead();

};

#endif
