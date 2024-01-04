#include "HeadingDisplay.h"
#include <qpainter.h>
#include <qpainterpath.h>

#ifdef __linux__
#include <cmath>
#endif

HeadingDisplay::HeadingDisplay(QWidget* parent) : QLabel(parent)
{
	_bg = nullptr;

	_heading = 0.0f;
}


void HeadingDisplay::ForceRepaint()
{
    updatePixmap(rect());
}


void HeadingDisplay::UpdateHeading(float heading)
{
	_heading = angleSaturation(heading, -180, 180);

	updatePixmap(rect());
}


float HeadingDisplay::angleSaturation(float angle, float min, float max)
{
	return angle < min ? min : angle > max ? max : angle;
}

void HeadingDisplay::updatePixmap(QRect rectangle, bool repaint)
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

	QPainterPath bgPath;
	bgPath.setFillRule(Qt::WindingFill);
	bgPath.addRoundedRect(QRectF(5, 5, width - 10, height - 10), 15, 15);
	bgPath.addRect(QRectF(5, height / 2 - 20, width - 10, 25));
	p.fillPath(bgPath, QBrush(QColor(50, 50, 50)));

	displayHeading(p, rectangle);

	if (repaint)
	{
		this->repaint();
	}
}


float HeadingDisplay::degToRadians(float angleDegrees)
{
	return angleDegrees * PI / 180.f;
}


void HeadingDisplay::displayHeading(QPainter& p, QRect rectangle)
{
	int height = rectangle.height();
	int width = rectangle.width();
	QPoint center(width / 2, height / 2);

	int spriteSize = 20;
	int propellerSize = 6;

	float arcRadius = width / 2 - 20;
	int headingNotchLength = 10;
	QPen penHeadingAngles(Qt::white, 2);
	p.setPen(penHeadingAngles);
	QFont displayFont = QFont("Courier New", 14, 1, false);
	displayFont.setBold(true);

	p.setFont(displayFont);

	for (float angleToDisplay = 0; angleToDisplay <= 360.0; angleToDisplay += 10)
	{
		float x0 = center.x() + cos(degToRadians(angleToDisplay + 180)) * arcRadius;
		float y0 = center.y() + sin(degToRadians(angleToDisplay + 180)) * arcRadius;
		float xf = center.x() + cos(degToRadians(angleToDisplay + 180)) * (arcRadius + headingNotchLength);
		float yf = center.y() + sin(degToRadians(angleToDisplay + 180)) * (arcRadius + headingNotchLength);

		p.drawLine(x0, y0, xf, yf);

		if (fabs(angleToDisplay) < 1)
		{
			p.drawText(xf + 14, yf + 5, "W");
		}

		if (fabs(angleToDisplay - 90) < 1)
		{
			p.drawText(xf - 5, yf + 28, "N");
		}

		if (fabs(angleToDisplay - 180) < 1)
		{
			p.drawText(xf - 28, yf + 5, "E");
		}

		if (fabs(angleToDisplay - 270) < 1)
		{
			p.drawText(xf - 5, yf - 14, "S");
		}
	}

	p.translate(center.x(), center.y());
	p.rotate(_heading);

	QPen spritePen(Qt::white, 2);
	p.setPen(spritePen);
	p.drawLine(-spriteSize, spriteSize, spriteSize, -spriteSize);
	p.drawLine(-spriteSize, -spriteSize, spriteSize, spriteSize);
	
	QPainterPath m4Path;
	m4Path.setFillRule(Qt::WindingFill);
	m4Path.addEllipse(QPointF(-spriteSize - propellerSize/2 - 1, spriteSize + propellerSize/2 + 1), propellerSize, propellerSize);
	p.drawPath(m4Path);

	QPainterPath m3Path;
	m3Path.setFillRule(Qt::WindingFill);
	m3Path.addEllipse(QPointF(spriteSize + propellerSize / 2 + 1, spriteSize + propellerSize / 2 + 1), propellerSize, propellerSize);
	p.drawPath(m3Path);

	QPainterPath m2Path;
	m2Path.setFillRule(Qt::WindingFill);
	m2Path.addEllipse(QPointF(spriteSize + propellerSize / 2 + 1, -spriteSize - propellerSize / 2 - 1), propellerSize, propellerSize);
	p.drawPath(m2Path);

	QPainterPath m1Path;
	m1Path.setFillRule(Qt::WindingFill);
	m1Path.addEllipse(QPointF(-spriteSize - propellerSize / 2 - 1, -spriteSize - propellerSize / 2 - 1), propellerSize, propellerSize);
	p.drawPath(m1Path);

	int arrowHeight = 14;
	int arrowBase = 8;
	int arrowCenterDistance = spriteSize + propellerSize;

	qreal startPointX1 = -arrowBase / 2;
	qreal startPointY1 = -arrowCenterDistance;
        
	qreal endPointX1 = arrowBase / 2;
	qreal endPointY1 = -arrowCenterDistance;

	qreal startPointX2 = 0.0;
	qreal startPointY2 = 0.0;
       
	qreal endPointX2 = 0;
	qreal endPointY2 = -arrowCenterDistance - arrowHeight;

	QPainterPath arrowPath;
	arrowPath.setFillRule(Qt::WindingFill);
	arrowPath.moveTo(startPointX1, startPointY1);
	arrowPath.lineTo(endPointX1, endPointY1);
	arrowPath.lineTo(endPointX2, endPointY2);
	arrowPath.lineTo(startPointX1, startPointY1);

	p.fillPath(arrowPath, Qt::white);
}


void HeadingDisplay::paintEvent(QPaintEvent* evt)
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
