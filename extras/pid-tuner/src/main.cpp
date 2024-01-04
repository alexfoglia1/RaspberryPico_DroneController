#include <qapplication.h>

#include "tunerwindow.h"
#include "joystick.h"
#include "comthread.h"

uint64_t changeEndianness(const uint64_t n)
{
	uint64_t buf[2];
	buf[0] = n;
	buf[1] = n;

	uint8_t* dst = reinterpret_cast<uint8_t*>(&buf[0]);
	uint8_t* src = reinterpret_cast<uint8_t*>(&buf[1]);

	for (int i = 0; i < 8; i++)
	{
		dst[i] = src[7 - i];
	}

	return buf[0];
}

ComThread* txThread;

int main(int argc, char** argv)
{
	QApplication* pidTuner;
	pidTuner = new QApplication(argc, argv);
#if 0
	qRegisterMetaType<js_control_packet>();
	qRegisterMetaType<js_button>();
	qRegisterMetaType<js_axis>();

	TunerWindow* tunerWindow = new TunerWindow();

	Joystick* js = new Joystick();

	QObject::connect(js, SIGNAL(jsConnected(bool)), tunerWindow, SLOT(OnJsConnected(bool)));
	QObject::connect(js, SIGNAL(jsBtnPressed(js_button)), tunerWindow, SLOT(OnJsBtnPressed(js_button)));
	QObject::connect(js, SIGNAL(jsControl(js_control_packet)), tunerWindow, SLOT(OnJsControl(js_control_packet)));
	QObject::connect(js, SIGNAL(jsThreadExit()), tunerWindow, SLOT(OnJsThreadExit()));
	QObject::connect(js, SIGNAL(jsAxisMoved(js_axis, qint16)), tunerWindow, SLOT(OnJsAxisMoved(js_axis, qint16)));

	tunerWindow->setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
	tunerWindow->setVisible(true);

	js->start();
#endif

	txThread = new ComThread();
	txThread->setSerialPort("COM15", QSerialPort::Baud38400);
	txThread->setDelay(100);

	QObject foo;

	txThread->start();

	QTimer* t = new QTimer();
	t->setSingleShot(false);
	t->setInterval(1000);
	t->setTimerType(Qt::PreciseTimer);
	QObject::connect(t, &QTimer::timeout, &foo, []() {
		quint16 signal = 2000;
		scanf("%hu", &signal);
		printf("Overriding armed signal(%hu)\n", signal);
		//txThread->overrideRadio(true);
		txThread->overrideArmedSignal(signal);
	});
	t->start();

	return pidTuner->exec();
}