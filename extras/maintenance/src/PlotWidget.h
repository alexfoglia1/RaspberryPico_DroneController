#ifndef PLOTWIDGET_H
#define PLOTWIDGET_H

#include <qlabel.h>
#include <qevent.h>
#include <list>


class PlotWidget : public QLabel
{
	Q_OBJECT

public:
	PlotWidget(QWidget* parent = nullptr);
	
	void UpdateSamplesPerSecond(double newValue);
	void SetYSpan(int track, int ySpan);
	void SetXSpan(int xSpan);
	void AddValue(int track, double val);
	void ClearData(int track);
    void ForceRepaint();

protected:
	void paintEvent(QPaintEvent* evt) override;

private:

	QPixmap* _bg;

	double _samplesPerSecond;
	int  _xSpan;
	int  _ySpan[3];
	std::list<double> _values[3];
	double _minY[3];
	double _maxY[3];
	QColor _trackColors[3];

	void updatePixmap(QRect rectangle, bool repaint=true);

};

#endif //PLOTWIDGET_H
