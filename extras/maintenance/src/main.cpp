#include <qserialport.h>
#include <qapplication.h>
#include <iostream>

#include "MaintenanceWindow.h"
#include "Joystick.h"


int main(int argc, char** argv)
{
	QApplication* maint;
	maint = new QApplication(argc, argv);

	qRegisterMetaType<js_control_packet>();
	qRegisterMetaType<js_button>();

	MaintenanceWindow* maintWindow = new MaintenanceWindow();
	maintWindow->setVisible(true);
	return maint->exec();
}