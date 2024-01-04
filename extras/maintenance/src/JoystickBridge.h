#ifndef JOYSTICK_BRIDGE_H
#define JOYSTICK_BRIDGE_H
#include <QObject>
#include <QJoysticks.h>

class JoystickBridge : public QObject
{
	Q_OBJECT
public:
	JoystickBridge(QObject* parent = nullptr);
	
signals:
	void overrideRadio(bool radioOverride);
	void overrideSignals(quint16 armedSignal, quint16 rollSignal, quint16 pitchSignal, quint16 throttleSignal);

private:
	QJoysticks* _js;
	bool _isOverride;

	quint16 _armedSignal;
	quint16 _rollSignal;
	quint16 _pitchSignal;
	quint16 _throttleSignal;

	qreal deadCenterZone(qreal axisValue, qreal deadCenter, qreal dczValue, qreal minAxisValue, qreal maxAxisValue);
	qreal mapValue(qreal value, qreal fromMin, qreal fromMax, qreal toMin, qreal toMax);
	qreal saturate(qreal value, qreal min, qreal max);


private slots:
	void onButtonEvent(const QJoystickButtonEvent& evt);
	void onAxisEvent(const QJoystickAxisEvent& evt);
};

#endif