#include <qserialport.h>
#include <qapplication.h>
#include <iostream>

#include "MaintenanceWindow.h"
#include "maint.h"


int main(int argc, char** argv)
{
	QApplication* maint;
	maint = new QApplication(argc, argv);

    QFont font("Verdana", 8);
    qApp->setFont(font);

	MaintenanceWindow* maintWindow = new MaintenanceWindow();
	maintWindow->setVisible(true);

	return maint->exec();
}
