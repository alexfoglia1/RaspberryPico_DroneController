#include "PlotWidget.h"

#include <qpainter.h>
#include <qdatetime.h>

#ifdef __linux__
#include <math.h>
#endif


PlotWidget::PlotWidget(QWidget* parent) : QLabel(parent)
{
	_xSpan = 250;
	_samplesPerSecond = 50;

	_ySpan[0] = 4000;
	_ySpan[1] = 4000;
	_ySpan[2] = 4000;

	_maxY[0] = -_ySpan[0];
	_minY[0] = _ySpan[0];
	_maxY[1] = -_ySpan[1];
	_minY[1] = _ySpan[1];
	_maxY[2] = -_ySpan[2];
	_minY[2] = _ySpan[2];

	_trackColors[0] = QColor(0x55, 0xff, 0xff);
	_trackColors[1] = QColor(0xff, 0x55, 0xff);
	_trackColors[2] = QColor(0xff, 0xff, 0x55);

	_bg = nullptr;
}


void PlotWidget::UpdateSamplesPerSecond(double newValue)
{
	_samplesPerSecond = newValue;

	repaint();
}


void PlotWidget::SetYSpan(int track, int ySpan)
{
	if (track >= 0 && track < 3)
	{
		_ySpan[track] = ySpan;
	}

	updatePixmap(rect());
}


void PlotWidget::SetXSpan(int xSpan)
{
	int dXspan = _xSpan - xSpan;
	
	_xSpan = xSpan;
	
	if (dXspan > 0)
	{
		for (int i = 0; i < 3; i++)
		{
			size_t samplesToRemove = (size_t)(dXspan);
			size_t samplesSize = _values[i].size();
			
			samplesToRemove = samplesToRemove < samplesSize ? samplesToRemove : samplesSize;
			for (int j = 0; j < samplesToRemove; j++)
			{
				_values[i].pop_front();
			}
		}
	}

	updatePixmap(rect());
}


void PlotWidget::AddValue(int track, double val)
{
	qint64 t = QDateTime::currentMSecsSinceEpoch();

	if (track >= 0 && track < 3 && val >= -_ySpan[track]/2 && val <= _ySpan[track]/2)
	{
		if (_values[track].size() < _xSpan)
		{
			_values[track].push_back(val);
		}
		else
		{
			
			_values[track].pop_front();
			_values[track].push_back(val);
		}

		if (val > _maxY[track]) _maxY[track] = val;
		if (val < _minY[track]) _minY[track] = val;

		updatePixmap(rect());
	}
}


void PlotWidget::updatePixmap(QRect rectangle, bool repaint)
{
	qint64 t = QDateTime::currentMSecsSinceEpoch();

	int height = rectangle.height();
	int width = rectangle.width();
	int offsetX = width / 40;
	int offsetY = height / 40;
	double xScale = ((double)(width)-2.0 * offsetX) / (double)_xSpan;
	double yScales[3] = { ((double)(height)-2.0 * offsetY) / (double)_ySpan[0],
						((double)(height)-2.0 * offsetY) / (double)_ySpan[1],
						((double)(height)-2.0 * offsetY) / (double)_ySpan[2] };

	if (!_bg)
	{
		_bg = new QPixmap(width, height);
	}

	QPainter painter(_bg);
	_bg->fill(QColor(0x00, 0x00, 0x00));

	QFont painterFont = QFont("Courier New", 8, 1, false);
	painterFont.setBold(true);
	painter.setFont(painterFont);

	// Draw axis
	painter.setPen(QPen(QColor(0xff, 0xff, 0xff)));
	painter.drawLine(QPoint(offsetX, height / 2), QPoint(width - offsetX - 5, height / 2));
	painter.drawLine(QPoint(offsetX, offsetY + 5), QPoint(offsetX, height - offsetY - 5));

	// Draw grid
	painter.setPen(QPen(QColor(0x55, 0x55, 0x55)));
	for (int y = height / 2 + (height - 2 * offsetY) / 10; y <= height - offsetY; y += (height - 2 * offsetY) / 10)
	{
		painter.drawLine(QPoint(offsetX + 1, y), QPoint(width - offsetX - 5, y));
		painter.drawLine(QPoint(offsetX + 1, height - y), QPoint(width - offsetX - 5, height - y));

		double yValUp[3] = { (qint64(y) - qint64(offsetY)) / yScales[0],
							   (qint64(y) - qint64(offsetY)) / yScales[1],
							   (qint64(y) - qint64(offsetY)) / yScales[2] };

		double yValDown[3] = { (qint64(height) - qint64(y) - qint64(offsetY)) / yScales[0],
							  (qint64(height) - qint64(y) - qint64(offsetY)) / yScales[1],
							  (qint64(height) - qint64(y) - qint64(offsetY)) / yScales[2] };

		for (int i = 0; i < 3; i++)
		{
			yValDown[i] = round((yValDown[i] - _ySpan[i] / 2));
			yValUp[i] = round((yValUp[i] - _ySpan[i] / 2));
		}

		painter.setPen(QPen(_trackColors[0]));
		painter.drawText(QPoint(offsetX + 1, y - 5), QString::number(yValDown[0]));
		painter.setPen(QPen(_trackColors[1]));
		painter.drawText(QPoint(offsetX + 1, y + 5), QString::number(yValDown[1]));
		painter.setPen(QPen(_trackColors[2]));
		painter.drawText(QPoint(offsetX + 1, y + 15), QString::number(yValDown[2]));
		painter.setPen(QPen(QColor(0x55, 0x55, 0x55)));

		painter.setPen(QPen(_trackColors[0]));
		painter.drawText(QPoint(offsetX + 1, height - y - 5), QString::number(yValUp[0]));
		painter.setPen(QPen(_trackColors[1]));
		painter.drawText(QPoint(offsetX + 1, height - y + 5), QString::number(yValUp[1]));
		painter.setPen(QPen(_trackColors[2]));
		painter.drawText(QPoint(offsetX + 1, height - y + 15), QString::number(yValUp[2]));
		painter.setPen(QPen(QColor(0x55, 0x55, 0x55)));
	}

	for (int i = 0; i < 3; i++)
	{
		painter.setPen(QPen(_trackColors[i]));
		painter.drawText(QPoint(offsetX + 80, (i+1)*30), QString("MIN(%1),    MAX(%2)").arg(_minY[i]).arg(_maxY[i]));
	}


	for (int x = offsetX; x <= width - offsetX; x += (width - 2 * offsetX) / 10)
	{
		if (x > offsetX)
		{
			painter.drawLine(QPoint(x, offsetY + 5), QPoint(x, height - offsetY - 5));
		}

		const double SAMPLES_PER_SECOND = _samplesPerSecond;
		double xSeconds = round(((qint64(x) - offsetX) / xScale) / SAMPLES_PER_SECOND * 10.0) / 10.0;

		painter.setPen(QPen(QColor(0xff, 0xff, 0xff)));
		painter.drawText(QPoint(x + 2, height / 2 + 15), QString::number(xSeconds));
		painter.setPen(QPen(QColor(0x55, 0x55, 0x55)));
	}

	// Draw values
	for (int track = 0; track < 3; track++)
	{
		double yScale = yScales[track];

		if (_values[track].size() > 1)
		{
			painter.setPen(QPen(_trackColors[track]));

			int i = 0;
			size_t listSize = _values[track].size();
			for (std::list<double>::iterator it = _values[track].begin(); i < listSize - 1; i++)
			{
				double curValue = *it;
				double nextValue = *(++it);

				double x0 = i * xScale;
				double y0 = curValue * yScale;
				double x1 = (i + 1.0) * xScale;
				double y1 = nextValue * yScale;

				painter.drawLine(offsetX + (int)x0, height / 2 - (int)y0, offsetX + (int)x1, height / 2 - (int)y1);
			}
		}
	}

	if (repaint) this->repaint();
}


void PlotWidget::ClearData(int track)
{
	if (track >= 0 && track < 3)
	{
		_values[track].clear();
		_maxY[track] = -_ySpan[track];
		_minY[track] = _ySpan[track];
	}

	updatePixmap(rect());
}


void PlotWidget::paintEvent(QPaintEvent* evt)
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




