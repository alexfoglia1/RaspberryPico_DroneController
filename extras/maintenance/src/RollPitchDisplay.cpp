#include "RollPitchDisplay.h"
#include <qpainter.h>
#include <qpainterpath.h>

#ifdef __linux__
#include <cmath>
#endif

RollPitchDisplay::RollPitchDisplay(QWidget* parent) : QLabel(parent)
{
	_bg = nullptr;

	_roll = 0.0f;
	_pitch = 0.0f;
	_overRoll = false;
	_overPitch = false;
}


void RollPitchDisplay::UpdateRoll(float roll)
{
	_roll = angleSaturation(roll, -60, 60);
	_overRoll = (_roll != roll);

	updatePixmap(rect());
}


void RollPitchDisplay::UpdatePitch(float pitch)
{
	_pitch = angleSaturation(pitch, -60, 60);
	_overPitch = (_pitch != pitch);

	updatePixmap(rect());
}


float RollPitchDisplay::angleSaturation(float angle, float min, float max)
{
	return angle < min ? min : angle > max ? max : angle;
}

void RollPitchDisplay::updatePixmap(QRect rectangle, bool repaint)
{
	int height = rectangle.height();
	int width = rectangle.width();

	if (!_bg)
	{
		_bg = new QPixmap(width, height);
	}
	_bg->fill(QColor(0x00, 0x00, 0x00));

	QPainter p(_bg);
	p.setRenderHint(QPainter::Antialiasing);

	QPainterPath pathTop;
	pathTop.setFillRule(Qt::WindingFill);
	pathTop.addRoundedRect(QRectF(5, 5, width - 10, height / 2), 15, 15);
	pathTop.addRect(QRectF(5, height/2 - 20, width - 10, 25));
	p.fillPath(pathTop, QBrush(QColor(0, 128, 255)));

	QPainterPath pathBottom;
	pathBottom.setFillRule(Qt::WindingFill);
	pathBottom.addRect(QRectF(5, height / 2, width - 10, 20));
	pathBottom.addRoundedRect(QRectF(5, height/2, width - 10, height/2 - 4), 15, 15);
	p.fillPath(pathBottom, QBrush(QColor(128, 64, 0)));

	QPen pen(Qt::white, 2);
	p.setPen(pen);
	p.drawLine(6, height / 2 - 1, width - 6, height / 2 - 1);

	displayAttitude(p, rectangle);

	if (repaint)
	{
		this->repaint();
	}
}


float RollPitchDisplay::degToRadians(float angleDegrees)
{
	return angleDegrees * PI / 180.f;
}


void RollPitchDisplay::displayAttitude(QPainter& p, QRect rectangle)
{
	int height = rectangle.height();
	int width = rectangle.width();
	QPoint center(width / 2, height / 2);
	int notchCenterOffset = 15;
	int notchWidth = width / 3 + 5;
	int notchHeight = 8;
	int centerNotchSize = 8;
	int whiskerSize = notchHeight;
	int rollIndicatorNotchLength = 10;
	int pitchIndicatorNotchLength = 20;

	float arcRadius = notchCenterOffset + notchWidth - 5;
	QPen penRollAngles(_overRoll ? Qt::red : Qt::white, 2);
	p.setPen(penRollAngles);

	for (float angleToDisplay = 22.5; angleToDisplay <= 157.5; angleToDisplay += 22.5)
	{
		float x0 = center.x() + cos(degToRadians(angleToDisplay + 180)) * arcRadius;
		float y0 = center.y() + sin(degToRadians(angleToDisplay + 180)) * arcRadius;
		float xf = center.x() + cos(degToRadians(angleToDisplay + 180)) * (arcRadius + rollIndicatorNotchLength);
		float yf = center.y() + sin(degToRadians(angleToDisplay + 180)) * (arcRadius + rollIndicatorNotchLength);

		p.drawLine(x0, y0, xf, yf);
	}

	QPen penPitchAngles(_overPitch ? Qt::red : Qt::white, 2);
	p.setPen(penPitchAngles);
	float pitchSpan = 180.f;
	float yScale = static_cast<float>(height) / pitchSpan;
	int i = 0;
	for (float angleToDisplay = -67.5; angleToDisplay <= 45; angleToDisplay += 22.5)
	{
		float x0 = center.x() - ((i % 2 == 0) ? (pitchIndicatorNotchLength / 2) : (pitchIndicatorNotchLength / 4));
		float y0 = center.y() - angleToDisplay * yScale;
		float xf = center.x() + ((i % 2 == 0) ? (pitchIndicatorNotchLength / 2) : (pitchIndicatorNotchLength / 4));
		float yf = y0;
		
		p.drawLine(x0, y0 - 1, xf, yf - 1);
		i++;
	}

	center.setY(center.y() - _pitch * yScale);

	p.translate(center.x(), center.y());
	p.rotate(_roll);

	QPainterPath horiNotchLeftBorder;
	horiNotchLeftBorder.setFillRule(Qt::WindingFill);
	horiNotchLeftBorder.addRect(-notchWidth - notchCenterOffset, -notchHeight / 2 - 1, notchWidth, notchHeight);
	horiNotchLeftBorder.addRect(-notchCenterOffset - whiskerSize, -notchHeight / 2 - 1, whiskerSize, 2 * whiskerSize + notchHeight);

	QPainterPath horiNotchLeftShape;
	horiNotchLeftShape.setFillRule(Qt::WindingFill);
	horiNotchLeftShape.addRect(-notchWidth - notchCenterOffset + 1 , -notchHeight / 2, notchWidth - 1, notchHeight - 1);
	horiNotchLeftShape.addRect(-notchCenterOffset - whiskerSize + 1, -notchHeight / 2, whiskerSize - 1, 2 * whiskerSize + notchHeight - 1);

	QPen penLeft(Qt::white, 2);
	p.setPen(penLeft);
	p.drawPath(horiNotchLeftBorder);
	p.fillPath(horiNotchLeftShape, _overRoll ? Qt::red : Qt::black);

	QPainterPath horiNotchRightBorder;
	horiNotchRightBorder.setFillRule(Qt::WindingFill);
	horiNotchRightBorder.addRect(notchCenterOffset, -notchHeight / 2 - 1, notchWidth, notchHeight);
	horiNotchRightBorder.addRect(notchCenterOffset, -notchHeight / 2 - 1, whiskerSize, 2 * whiskerSize + notchHeight);

	QPainterPath horiNotchRightShape;
	horiNotchRightShape.setFillRule(Qt::WindingFill);
	horiNotchRightShape.addRect(notchCenterOffset + 1, -notchHeight / 2, notchWidth - 1, notchHeight - 1);
	horiNotchRightShape.addRect(notchCenterOffset + 1, -notchHeight / 2, whiskerSize - 1, 2 * whiskerSize + notchHeight - 1);

	QPen penRight(Qt::white, 2);
	p.setPen(penRight);
	p.drawPath(horiNotchRightBorder);
	p.fillPath(horiNotchRightShape, _overRoll ? Qt::red : Qt::black);

	QPainterPath centerNotch;
	centerNotch.setFillRule(Qt::WindingFill);
	centerNotch.addRect(-centerNotchSize / 2, -notchHeight / 2 - 1, centerNotchSize, centerNotchSize);
	p.fillPath(centerNotch, _overRoll ? Qt::red : Qt::black);

	QPen penCenter(Qt::white, 2);
	p.setPen(penCenter);
	p.drawPath(centerNotch);

}


void RollPitchDisplay::paintEvent(QPaintEvent* evt)
{
	QLabel::paintEvent(evt);

	QRect rectangle = rect();

	if (_bg)
	{
		QPainter painter(this);
		painter.drawPixmap(rectangle, *_bg);
	}
	else
	{
		updatePixmap(rectangle, false);
	}
}
