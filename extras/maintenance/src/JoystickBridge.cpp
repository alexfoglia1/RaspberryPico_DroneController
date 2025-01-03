#include "JoystickBridge.h"

#ifdef __linux__
#include <cmath>
#endif

JoystickBridge::JoystickBridge(QObject* parent) : QObject(parent)
{
	_isOverride = false;

	_js = QJoysticks::getInstance();

	_armedSignal = 1000;
	_rollSignal = 1000;
	_pitchSignal = 1000;
	_throttleSignal = 1000;

	connect(_js, &QJoysticks::buttonEvent, this, &JoystickBridge::onButtonEvent);
	connect(_js, &QJoysticks::axisEvent, this, &JoystickBridge::onAxisEvent);
}


qreal JoystickBridge::deadCenterZone(qreal axisValue, qreal deadCenter, qreal dczValue, qreal minAxisValue, qreal maxAxisValue)
{
	if (fabs(axisValue) < deadCenter)
	{
		return dczValue;
	}
	else
	{
		double axisSpan = maxAxisValue - minAxisValue;

		if (minAxisValue < 0)
		{
			if (axisValue < 0)
			{
				return mapValue(axisValue, minAxisValue, -deadCenter, 1000.0, 1500.0);
			}
			else
			{
				return mapValue(axisValue, deadCenter, maxAxisValue, 1500.0, 2000.0);
			}
		}
		else
		{
			return mapValue(axisValue, minAxisValue + deadCenter, maxAxisValue, 1000.0, 2000.0);
		}
	}
}


qreal JoystickBridge::saturate(qreal value, qreal min, qreal max)
{
	return value < min ? min : value > max ? max : value;
}


qreal JoystickBridge::mapValue(qreal value, qreal fromMin, qreal fromMax, qreal toMin, qreal toMax)
{
	qreal fromSpan = fromMax - fromMin;
	qreal fromPercentage = ((qreal)value - (qreal)fromMin) / (qreal)fromSpan;

	int toSpan = toMax - toMin;

	return toMin + fromPercentage * toSpan;
}


void JoystickBridge::onButtonEvent(const QJoystickButtonEvent& evt)
{
	if (evt.button == 0 && evt.pressed)
	{
		_isOverride = !_isOverride;
		emit overrideRadio(_isOverride);
	}
	else if (evt.button == 1 && evt.pressed && _isOverride)
	{
		_armedSignal = _armedSignal > 1500 ? 1000 : 2000;

		emit overrideSignals(_armedSignal, _rollSignal, _pitchSignal, _throttleSignal);
	}
	else if (evt.button == 11 && evt.pressed && _isOverride)
	{
		if (_throttleSignal <= 1990)
		{
			_throttleSignal += 10;
			emit overrideSignals(_armedSignal, _rollSignal, _pitchSignal, _throttleSignal);
		}
	}
	else if (evt.button == 12 && evt.pressed && _isOverride)
	{
		if (_throttleSignal >= 1010)
		{
			_throttleSignal -= 10;
			emit overrideSignals(_armedSignal, _rollSignal, _pitchSignal, _throttleSignal);
		}
	}
	
}


void JoystickBridge::onAxisEvent(const QJoystickAxisEvent& evt)
{
	qreal evtValue = saturate(evt.value, -1.0, 1.0);
	//printf("evt.axis(%d)\n", evt.axis);

	bool isValidAxisEvent = (evt.axis == 5 || evt.axis == 2 || evt.axis == 3);
	if (isValidAxisEvent)
	{
		quint16 throttleSignal = _throttleSignal;
		quint16 rollSignal = _rollSignal;
		quint16 pitchSignal = _pitchSignal;

		if (evt.axis == 5)
		{
			if (evtValue < 0) evtValue = 0;
			qreal fSignal = mapValue(evtValue, 0, 1.0, 1000.0, 2000.0);// deadCenterZone(-evtValue, 0.1, 1500.0, -1.0, 1.0);

			_throttleSignal = (quint16)(fSignal);
		}
		else if (evt.axis == 2)
		{
			qreal fSignal = deadCenterZone(evtValue, 0.1, 1500.0, -1.0, 1.0);
			_rollSignal = (quint16)(fSignal);
		}
		else if (evt.axis == 3)
		{
			qreal fSignal = deadCenterZone(-evtValue, 0.1, 1500.0, -1.0, 1.0);
			_pitchSignal = (quint16)(fSignal);
		}

		bool axisUpdate = ((throttleSignal != _throttleSignal) || (rollSignal != _rollSignal) || (pitchSignal != _pitchSignal));
		if (axisUpdate && _isOverride)
		{
			emit overrideSignals(_armedSignal, _rollSignal, _pitchSignal, _throttleSignal);
		}
		
	}
}
