#ifndef HEADING_DISPLAY_H
#define HEADING_DISPLAY_H

#include <qlabel.h>
#define PI 3.14159265358979323846264338327950288419716939937510582f

class HeadingDisplay : public QLabel
{
	Q_OBJECT

public:
	HeadingDisplay(QWidget* parent = nullptr);

	void UpdateHeading(float heading);
    void ForceRepaint();

protected:
	void paintEvent(QPaintEvent* evt) override;

private:
	QPixmap* _bg;
	float _heading;

	void updatePixmap(QRect rectangle, bool repaint = true);
	void displayHeading(QPainter& p, QRect rectangle);
	float angleSaturation(float angle, float min, float max);
	float degToRadians(float angleDegrees);
};

#endif

