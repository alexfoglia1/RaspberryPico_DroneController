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
	int  _ySpan[4];
	std::list<double> _values[4];
	double _minY[4];
	double _maxY[4];
	QColor _trackColors[4];

	void updatePixmap(QRect rectangle, bool repaint=true);

};

#endif //PLOTWIDGET_H
