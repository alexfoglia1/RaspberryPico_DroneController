#ifndef RP_DISPLAY_H
#define RP_DISPLAY_H

#include <qlabel.h>
#define PI 3.14159265358979323846264338327950288419716939937510582f

class RollPitchDisplay : public QLabel
{
	Q_OBJECT

public:
	RollPitchDisplay(QWidget* parent = nullptr);

	void UpdateRoll(float roll);
	void UpdatePitch(float pitch);

protected:
	void paintEvent(QPaintEvent* evt) override;

private:
	QPixmap* _bg;
	float _roll;
	float _pitch;
	bool _overRoll;
	bool _overPitch;

	void updatePixmap(QRect rectangle, bool repaint = true);
	void displayAttitude(QPainter& p, QRect rectangle);
	float angleSaturation(float angle, float min, float max);
	float degToRadians(float angleDegrees);
};

#endif

