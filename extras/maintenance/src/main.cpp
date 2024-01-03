#include <qserialport.h>
#include <qapplication.h>
#include <iostream>

#include "MaintenanceWindow.h"
#include "Joystick.h"
#include "maint.h"


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


int main(int argc, char** argv)
{
	QApplication* maint;
	maint = new QApplication(argc, argv);

	qRegisterMetaType<js_control_packet>();
	qRegisterMetaType<js_button>();

	MaintenanceWindow* maintWindow = new MaintenanceWindow();
	maintWindow->setVisible(true);

	Maint::MAINT_HEADER_T header;
	header.All = 0;
	header.Bits.body_pitch = 1;
	printf("private static final long BODY_PITCH = %s;\n", QString("0x%1").arg(QString::number(changeEndianness(header.All), 16)).toStdString().c_str());
	header.All = 0;
	header.Bits.body_roll = 1;
	printf("private static final long BODY_ROLL = %s;\n", QString("0x%1").arg(QString::number(changeEndianness(header.All), 16)).toStdString().c_str());
	header.All = 0;
	header.Bits.body_yaw = 1;
	printf("private static final long BODY_YAW = %s;\n", QString("0x%1").arg(QString::number(changeEndianness(header.All), 16)).toStdString().c_str());
	header.All = 0;
	header.Bits.motor1 = 1;
	printf("private static final long MOTOR_1_SIGNAL = %s;\n", QString("0x%1").arg(QString::number(changeEndianness(header.All), 16)).toStdString().c_str());
	header.All = 0;
	header.Bits.motor2 = 1;
	printf("private static final long MOTOR_2_SIGNAL = %s;\n", QString("0x%1").arg(QString::number(changeEndianness(header.All), 16)).toStdString().c_str());
	header.All = 0;
	header.Bits.motor3 = 1;
	printf("private static final long MOTOR_3_SIGNAL = %s;\n", QString("0x%1").arg(QString::number(changeEndianness(header.All), 16)).toStdString().c_str());
	header.All = 0;
	header.Bits.motor4 = 1;
	printf("private static final long MOTOR_4_SIGNAL = %s;\n", QString("0x%1").arg(QString::number(changeEndianness(header.All), 16)).toStdString().c_str());

	return maint->exec();
}