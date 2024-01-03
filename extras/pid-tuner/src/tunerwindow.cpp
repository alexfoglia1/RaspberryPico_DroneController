#include "tunerwindow.h"


TunerWindow::TunerWindow(QWidget* parent) : QMainWindow(parent)
{
	_ui.setupUi(this);

	_navigationFsm =
	{
		{MenuNavigationStatus::NAVIGATE_WINDOW, js_button::RIGHT,    MenuNavigationStatus::NAVIGATE_WINDOW, &TunerWindow::navigateWindowRight},
		{MenuNavigationStatus::NAVIGATE_WINDOW, js_button::LEFT,     MenuNavigationStatus::NAVIGATE_WINDOW, &TunerWindow::navigateWindowLeft},
		{MenuNavigationStatus::NAVIGATE_WINDOW, js_button::CROSS,    MenuNavigationStatus::NAVIGATE_MENU,   &TunerWindow::selectMenu},
		{MenuNavigationStatus::NAVIGATE_WINDOW, js_button::CIRCLE,   MenuNavigationStatus::NAVIGATE_WINDOW, &TunerWindow::menuExit},
		{MenuNavigationStatus::NAVIGATE_WINDOW, js_button::SQUARE,   MenuNavigationStatus::NAVIGATE_WINDOW, &TunerWindow::refreshPidParams},
		{MenuNavigationStatus::NAVIGATE_WINDOW, js_button::TRIANGLE, MenuNavigationStatus::NAVIGATE_WINDOW, &TunerWindow::toggleMotorsArmed},
		{MenuNavigationStatus::NAVIGATE_WINDOW, js_button::L1,       MenuNavigationStatus::NAVIGATE_WINDOW, &TunerWindow::toggleOverrideRadio},

		{MenuNavigationStatus::NAVIGATE_MENU,   js_button::UP,       MenuNavigationStatus::NAVIGATE_MENU,   &TunerWindow::navigateMenuUp},
		{MenuNavigationStatus::NAVIGATE_MENU,   js_button::DOWN,     MenuNavigationStatus::NAVIGATE_MENU,   &TunerWindow::navigateMenuDown},
		{MenuNavigationStatus::NAVIGATE_MENU,   js_button::RIGHT,    MenuNavigationStatus::NAVIGATE_ENTRY,  &TunerWindow::selectEntry},
		{MenuNavigationStatus::NAVIGATE_MENU,   js_button::CIRCLE,   MenuNavigationStatus::NAVIGATE_WINDOW, &TunerWindow::menuExit},
		{MenuNavigationStatus::NAVIGATE_MENU,   js_button::SQUARE,   MenuNavigationStatus::NAVIGATE_MENU,   &TunerWindow::refreshPidParams},
		{MenuNavigationStatus::NAVIGATE_MENU,   js_button::TRIANGLE, MenuNavigationStatus::NAVIGATE_MENU,   &TunerWindow::toggleMotorsArmed},
		{MenuNavigationStatus::NAVIGATE_MENU,   js_button::L1,       MenuNavigationStatus::NAVIGATE_MENU,   &TunerWindow::toggleOverrideRadio},

		{MenuNavigationStatus::NAVIGATE_ENTRY,  js_button::UP,       MenuNavigationStatus::NAVIGATE_ENTRY,  &TunerWindow::navigateEntryUp},
		{MenuNavigationStatus::NAVIGATE_ENTRY,  js_button::DOWN,     MenuNavigationStatus::NAVIGATE_ENTRY,  &TunerWindow::navigateEntryDown},
		{MenuNavigationStatus::NAVIGATE_ENTRY,  js_button::CROSS,    MenuNavigationStatus::NAVIGATE_ENTRY,  &TunerWindow::selectEntryValue},
		{MenuNavigationStatus::NAVIGATE_ENTRY,  js_button::LEFT,     MenuNavigationStatus::NAVIGATE_MENU,   &TunerWindow::entryExit},
		{MenuNavigationStatus::NAVIGATE_ENTRY,  js_button::SQUARE,   MenuNavigationStatus::NAVIGATE_ENTRY,  &TunerWindow::refreshPidParams},
		{MenuNavigationStatus::NAVIGATE_ENTRY,  js_button::TRIANGLE, MenuNavigationStatus::NAVIGATE_ENTRY,  &TunerWindow::toggleMotorsArmed},
		{MenuNavigationStatus::NAVIGATE_ENTRY,  js_button::L1,       MenuNavigationStatus::NAVIGATE_ENTRY,  &TunerWindow::toggleOverrideRadio},

	};

	_ui.serialCommMenu->setName("COMM");
	_commMenuEntries =
	{
		{quint32(MenuEntryKey::MENU_KEY_SERIAL_PORT),  "PORT",     MenuWidgetValueType::LIST,   QVariant(),       MenuWidgetValueType::LED,  QVariant(), QVariant(0), quint8(MenuWidgetLedColor::RED)},
		{quint32(MenuEntryKey::MENU_KEY_BAUD_RATE),    "BAUD",     MenuWidgetValueType::LIST,   QVariant(),       MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_DRONE_STATUS), "DRONE",    MenuWidgetValueType::STRING, QVariant("FAIL"), MenuWidgetValueType::LED,  QVariant(), QVariant(),  quint8(MenuWidgetLedColor::RED)},
		{quint32(MenuEntryKey::MENU_KEY_JS_STATUS),    "JOYSTICK", MenuWidgetValueType::STRING, QVariant("FAIL"), MenuWidgetValueType::LED,  QVariant(), QVariant(),  quint8(MenuWidgetLedColor::RED)}

	};

	for (auto& entry : _commMenuEntries)
	{
		_ui.serialCommMenu->addEntry(entry);
	}

	// TODO : Autoscan
	QStringList ports;
	for (int i = 0; i < 100; i++)
	{
		QString comPort = QString("COM%1").arg(i);
		ports.append(comPort);
	}
	SetCommMenuEntryValue1(MenuEntryKey::MENU_KEY_SERIAL_PORT, ports);
	SetCommMenuEntryDefaultValue1(MenuEntryKey::MENU_KEY_SERIAL_PORT, 26);

	QStringList baudRates =
	{
		"1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"
	};

	SetCommMenuEntryValue1(MenuEntryKey::MENU_KEY_BAUD_RATE, baudRates);
	SetCommMenuEntryDefaultValue1(MenuEntryKey::MENU_KEY_BAUD_RATE, 5);

	connect(_ui.serialCommMenu, SIGNAL(entryConfirmed(quint32, QVariant)), this, SLOT(OnMenuEntryUpdated(quint32, QVariant)));

	_ui.pidRollMenu->setName("ROLL");
	_rollMenuEntries =
	{
		{quint32(MenuEntryKey::MENU_KEY_ROLL_KP),  "KP",  MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_ROLL_KI),  "KI",  MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_ROLL_KT),  "KT",  MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_ROLL_AD),  "AD",  MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_ROLL_BD),  "BD",  MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_ROLL_SAT), "SAT", MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()}
	};

	for (auto& entry : _rollMenuEntries)
	{
		_ui.pidRollMenu->addEntry(entry);
	}

	_ui.pidPitchMenu->setName("PITCH");
	_pitchMenuEntries =
	{
		{quint32(MenuEntryKey::MENU_KEY_PITCH_KP),  "KP",  MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_PITCH_KI),  "KI",  MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_PITCH_KT),  "KT",  MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_PITCH_AD),  "AD",  MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_PITCH_BD),  "BD",  MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_PITCH_SAT), "SAT", MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()}
	};

	for (auto& entry : _pitchMenuEntries)
	{
		_ui.pidPitchMenu->addEntry(entry);
	}


	_ui.pidYawMenu->setName("YAW");
	_yawMenuEntries =
	{
		{quint32(MenuEntryKey::MENU_KEY_YAW_KP),  "KP",  MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_YAW_KI),  "KI",  MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_YAW_KT),  "KT",  MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_YAW_AD),  "AD",  MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_YAW_BD),  "BD",  MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()},
		{quint32(MenuEntryKey::MENU_KEY_YAW_SAT), "SAT", MenuWidgetValueType::DECIMAL, QVariant(0), MenuWidgetValueType::NONE, QVariant(), QVariant(0), QVariant()}
	};

	for (auto& entry : _yawMenuEntries)
	{
		_ui.pidYawMenu->addEntry(entry);
	}

	_currentNavigationStatus = MenuNavigationStatus::NAVIGATE_WINDOW;
	_comThread = nullptr;

	_menus = { _ui.serialCommMenu, _ui.pidRollMenu, _ui.pidPitchMenu, _ui.pidYawMenu };
	_curMenuIdx = -1;
	_selectedMenuWidget = nullptr;
	_motorsArmed = false;
	_radioOverride = false;
}


void TunerWindow::SetCommMenuEntryValue1(MenuEntryKey entryKey, QVariant newValue)
{
	_ui.serialCommMenu->setMenuItemValue1(quint32(entryKey), newValue);
}


void TunerWindow::SetCommMenuEntryValue2(MenuEntryKey entryKey, QVariant newValue)
{
	_ui.serialCommMenu->setMenuItemValue2(quint32(entryKey), newValue);
}


void TunerWindow::SetCommMenuEntryDefaultValue1(MenuEntryKey entryKey, QVariant newValue)
{
	_ui.serialCommMenu->setMenuItemDefaultValue1(quint32(entryKey), newValue);
}


void TunerWindow::SetCommMenuEntryDefaultValue2(MenuEntryKey entryKey, QVariant newValue)
{
	_ui.serialCommMenu->setMenuItemDefaultValue2(quint32(entryKey), newValue);
}


void TunerWindow::OnDroneAlive()
{
	SetCommMenuEntryValue2(MenuEntryKey::MENU_KEY_DRONE_STATUS, quint8(MenuWidgetLedColor::GREEN));
	SetCommMenuEntryValue1(MenuEntryKey::MENU_KEY_DRONE_STATUS, "OPER");
}


void TunerWindow::OnDroneDownlink()
{
	SetCommMenuEntryValue2(MenuEntryKey::MENU_KEY_DRONE_STATUS, quint8(MenuWidgetLedColor::RED));
	SetCommMenuEntryValue1(MenuEntryKey::MENU_KEY_DRONE_STATUS, "FAIL");
}



void TunerWindow::OnDroneAttitudeUpdate(float roll, float pitch, float yaw)
{
	_ui.pfdRollPitch->UpdatePitch(pitch);
	_ui.pfdRollPitch->UpdateRoll(roll);
	_ui.pfdHeading->UpdateHeading(yaw);

	_ui.lcdNumberRoll->display((double) roll);
	_ui.lcdNumberPitch->display((double) pitch);
	_ui.lcdNumberYaw->display((double) yaw);
}


void TunerWindow::OnDroneMotorsUpdate(quint32 m1, quint32 m2, quint32 m3, quint32 m4)
{
	_ui.lcdNumberM1->display((int) m1);
	_ui.lcdNumberM2->display((int) m2);
	_ui.lcdNumberM3->display((int) m3);
	_ui.lcdNumberM4->display((int) m4);
}

void TunerWindow::OnRollPidParams(float kp, float ki, float kt, float sat, float ad, float bd)
{
	_ui.pidRollMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_ROLL_KP),  kp);
	_ui.pidRollMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_ROLL_KI),  ki);
	_ui.pidRollMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_ROLL_KT),  kt);
	_ui.pidRollMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_ROLL_SAT), sat);
	_ui.pidRollMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_ROLL_AD),  ad);
	_ui.pidRollMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_ROLL_BD),  bd);
}


void TunerWindow::OnPitchPidParams(float kp, float ki, float kt, float sat, float ad, float bd)
{
	_ui.pidPitchMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_PITCH_KP),  kp);
	_ui.pidPitchMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_PITCH_KI),  ki);
	_ui.pidPitchMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_PITCH_KT),  kt);
	_ui.pidPitchMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_PITCH_SAT), sat);
	_ui.pidPitchMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_PITCH_AD),  ad);
	_ui.pidPitchMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_PITCH_BD),  bd);
}


void TunerWindow::OnYawPidParams(float kp, float ki, float kt, float sat, float ad, float bd)
{
	_ui.pidYawMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_YAW_KP),  kp);
	_ui.pidYawMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_YAW_KI),  ki);
	_ui.pidYawMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_YAW_KT),  kt);
	_ui.pidYawMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_YAW_SAT), sat);
	_ui.pidYawMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_YAW_AD),  ad);
	_ui.pidYawMenu->setMenuItemValue1(quint32(MenuEntryKey::MENU_KEY_YAW_BD),  bd);
}

void TunerWindow::OnJsConnected(bool connected)
{
	SetCommMenuEntryValue2(MenuEntryKey::MENU_KEY_JS_STATUS, connected ? quint8(MenuWidgetLedColor::GREEN) : quint8(MenuWidgetLedColor::RED));
	SetCommMenuEntryValue1(MenuEntryKey::MENU_KEY_JS_STATUS, connected ? "OPER" : "FAIL");
}


void TunerWindow::OnJsBtnPressed(js_button btn)
{
	for (auto& fsmEntry : _navigationFsm)
	{
		if (fsmEntry.currentStatus == _currentNavigationStatus && fsmEntry.buttonPressed == btn)
		{
			if ((this->*fsmEntry.action)())
			{
				_currentNavigationStatus = fsmEntry.nextStatus;
			}
			break;
		}
	}
}


void TunerWindow::OnJsControl(js_control_packet controlPacket)
{
	quint16 roll_signal = Joystick::remapJsValue(controlPacket.r3_haxis, 1000, 2000);
	quint16 pitch_signal = Joystick::remapJsValue(controlPacket.r3_vaxis, 1000, 2000);
	quint16 throttle_signal = Joystick::remapJsValue(controlPacket.r2_axis, 1000, 2000);

	if (_comThread)
	{
		_comThread->controlFlight(roll_signal, pitch_signal, throttle_signal);
	}
}


void TunerWindow::OnJsThreadExit()
{

}


void TunerWindow::OnJsAxisMoved(js_axis axis, qint16 val)
{
	if (js_axis::L3_Y_AXIS == axis)
	{
		if (_currentNavigationStatus == MenuNavigationStatus::NAVIGATE_ENTRY)
		{
			if (_selectedMenuWidget != nullptr)
			{
				_selectedMenuWidget->analogScrollEntryValue(val);
			}
		}
	}
}


bool TunerWindow::navigateWindowRight()
{
	if (_selectedMenuWidget != nullptr) _selectedMenuWidget->setHighlighted(false);

	if (_curMenuIdx < 3)
	{
		_curMenuIdx++;
	}
	else
	{
		_curMenuIdx = 0;
	}

	_selectedMenuWidget = _menus[_curMenuIdx];
	_selectedMenuWidget->setHighlighted(true);

	return true;
}


bool TunerWindow::navigateWindowLeft()
{
	if (_selectedMenuWidget != nullptr) _selectedMenuWidget->setHighlighted(false);

	if (_curMenuIdx > 0)
	{
		_curMenuIdx--;
	}
	else
	{
		_curMenuIdx = 3;
	}

	_selectedMenuWidget = _menus[_curMenuIdx];
	_selectedMenuWidget->setHighlighted(true);

	return true;
}


bool TunerWindow::navigateMenuUp()
{
	if (_selectedMenuWidget)
		_selectedMenuWidget->menuUp();

	return (_selectedMenuWidget != nullptr);
}


bool TunerWindow::navigateMenuDown()
{
	if (_selectedMenuWidget)
		_selectedMenuWidget->menuDown();

	return (_selectedMenuWidget != nullptr);
}


bool TunerWindow::navigateEntryUp()
{
	if (_selectedMenuWidget)
		_selectedMenuWidget->entryUp();

	return (_selectedMenuWidget != nullptr);
}


bool TunerWindow::navigateEntryDown()
{
	if (_selectedMenuWidget)
		_selectedMenuWidget->entryDown();

	return (_selectedMenuWidget != nullptr);
}


bool TunerWindow::selectMenu()
{
	if (_selectedMenuWidget)
		_selectedMenuWidget->navigateMenu();

	return (_selectedMenuWidget != nullptr);
}


bool TunerWindow::selectEntry()
{
	if (_selectedMenuWidget)
		_selectedMenuWidget->navigateEntry();

	return (_selectedMenuWidget != nullptr);
}


bool TunerWindow::selectEntryValue()
{
	if (_selectedMenuWidget)
		_selectedMenuWidget->confirmEntryValue();

	return (_selectedMenuWidget != nullptr);
}


bool TunerWindow::entryExit()
{
	if (_selectedMenuWidget)
		_selectedMenuWidget->entryExit();

	return (_selectedMenuWidget != nullptr);
}


bool TunerWindow::menuExit()
{
	if (_selectedMenuWidget)
	{
		_selectedMenuWidget->setHighlighted(false);
		_selectedMenuWidget->menuExit();
	}

	_selectedMenuWidget = nullptr;
	return true;
}


bool TunerWindow::refreshPidParams()
{
	if (_comThread)
	{
		_comThread->refreshPidParams();
	}

	return true;
}


bool TunerWindow::toggleMotorsArmed()
{
	if (_comThread)
	{
		_motorsArmed = !_motorsArmed;
		_comThread->overrideArmedSignal(_motorsArmed ? quint16(2000) : quint16(1000));
	}

	return true;
}


bool TunerWindow::toggleOverrideRadio()
{
	if (_comThread)
	{
		_radioOverride = !_radioOverride;
		_comThread->overrideRadio(_radioOverride);
	}

	return true;
}



void TunerWindow::OnMenuEntryUpdated(quint32 entryKey, QVariant newValue)
{
	MenuEntryKey key = MenuEntryKey(entryKey);
	switch (key)
	{
		case MenuEntryKey::MENU_KEY_JS_STATUS:
			break;
		case MenuEntryKey::MENU_KEY_BAUD_RATE:
			break;
		case MenuEntryKey::MENU_KEY_DRONE_STATUS:
			break;
		case MenuEntryKey::MENU_KEY_SERIAL_PORT:
		{
			if (_comThread == nullptr)
			{
				_comThread = new ComThread();
				connect(_comThread, SIGNAL(serialPortOpened()), this, SLOT(OnSerialPortOpened()));
				connect(_comThread, SIGNAL(serialPortClosed()), this, SLOT(OnSerialPortClosed()));
				connect(_comThread, SIGNAL(droneAlive()), this, SLOT(OnDroneAlive()));
				connect(_comThread, SIGNAL(droneDownlink()), this, SLOT(OnDroneDownlink()));
				connect(_comThread, SIGNAL(droneAttitudeUpdate(float, float, float)), this, SLOT(OnDroneAttitudeUpdate(float, float, float)));
				connect(_comThread, SIGNAL(droneMotorsUpdate(quint32, quint32, quint32, quint32)), this, SLOT(OnDroneMotorsUpdate(quint32, quint32, quint32, quint32)));
				connect(_comThread, SIGNAL(rollPidParamsUpdate(float, float, float, float, float, float)), this, SLOT(OnRollPidParams(float, float, float, float, float, float)));
				connect(_comThread, SIGNAL(pitchPidParamsUpdate(float, float, float, float, float, float)), this, SLOT(OnPitchPidParams(float, float, float, float, float, float)));
				connect(_comThread, SIGNAL(yawPidParamsUpdate(float, float, float, float, float, float)), this, SLOT(OnYawPidParams(float, float, float, float, float, float)));

				int iBaudValue = _ui.serialCommMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_BAUD_RATE)).toString().toInt();
				QSerialPort::BaudRate baudRate = QSerialPort::BaudRate(iBaudValue);
				_comThread->setSerialPort(newValue.toString(), baudRate);
				_comThread->setDelay(250); // TODO menu
				_comThread->start();
			}
			else
			{
				_comThread->stopCom();
			}
			break;
		}
		case MenuEntryKey::MENU_KEY_ROLL_KP:
		case MenuEntryKey::MENU_KEY_ROLL_KI:
		case MenuEntryKey::MENU_KEY_ROLL_KT:
		case MenuEntryKey::MENU_KEY_ROLL_AD:
		case MenuEntryKey::MENU_KEY_ROLL_BD:
		case MenuEntryKey::MENU_KEY_ROLL_SAT:
			if (_comThread != nullptr)
			{
				float kp = _ui.pidRollMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_ROLL_KP)).toFloat();
				float ki = _ui.pidRollMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_ROLL_KI)).toFloat();
				float kt = _ui.pidRollMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_ROLL_KT)).toFloat();
				float sat = _ui.pidRollMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_ROLL_SAT)).toFloat();
				float ad = _ui.pidRollMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_ROLL_AD)).toFloat();
				float bd = _ui.pidRollMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_ROLL_BD)).toFloat();
				_comThread->writeRollPidParams(kp, ki, kt, sat, ad, bd);
			}
			break;
		case MenuEntryKey::MENU_KEY_PITCH_KP:
		case MenuEntryKey::MENU_KEY_PITCH_KI:
		case MenuEntryKey::MENU_KEY_PITCH_KT:
		case MenuEntryKey::MENU_KEY_PITCH_SAT:
		case MenuEntryKey::MENU_KEY_PITCH_AD:
		case MenuEntryKey::MENU_KEY_PITCH_BD:
			if (_comThread != nullptr)
			{
				float kp = _ui.pidPitchMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_PITCH_KP)).toFloat();
				float ki = _ui.pidPitchMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_PITCH_KI)).toFloat();
				float kt = _ui.pidPitchMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_PITCH_KT)).toFloat();
				float sat = _ui.pidPitchMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_PITCH_SAT)).toFloat();
				float ad = _ui.pidPitchMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_PITCH_AD)).toFloat();
				float bd = _ui.pidPitchMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_PITCH_BD)).toFloat();
				_comThread->writePitchPidParams(kp, ki, kt, sat, ad, bd);
			}
			break;
		case MenuEntryKey::MENU_KEY_YAW_KP:
		case MenuEntryKey::MENU_KEY_YAW_KI:
		case MenuEntryKey::MENU_KEY_YAW_KT:
		case MenuEntryKey::MENU_KEY_YAW_SAT:
		case MenuEntryKey::MENU_KEY_YAW_AD:
		case MenuEntryKey::MENU_KEY_YAW_BD:
			if (_comThread != nullptr)
			{
				float kp = _ui.pidYawMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_YAW_KP)).toFloat();
				float ki = _ui.pidYawMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_YAW_KI)).toFloat();
				float kt = _ui.pidYawMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_YAW_KT)).toFloat();
				float sat = _ui.pidYawMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_YAW_SAT)).toFloat();
				float ad = _ui.pidYawMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_YAW_AD)).toFloat();
				float bd = _ui.pidYawMenu->valueOf(quint32(MenuEntryKey::MENU_KEY_YAW_BD)).toFloat();
				_comThread->writeYawPidParams(kp, ki, kt, sat, ad, bd);
			}
			break;
	}
}


void TunerWindow::OnSerialPortOpened()
{
	SetCommMenuEntryValue2(MenuEntryKey::MENU_KEY_SERIAL_PORT, quint8(MenuWidgetLedColor::GREEN));
}


void TunerWindow::OnSerialPortClosed()
{
	SetCommMenuEntryValue2(MenuEntryKey::MENU_KEY_SERIAL_PORT, quint8(MenuWidgetLedColor::RED));

	disconnect(_comThread, SIGNAL(serialPortOpened()), this, SLOT(OnSerialPortOpened()));
	disconnect(_comThread, SIGNAL(serialPortClosed()), this, SLOT(OnSerialPortClosed()));
	disconnect(_comThread, SIGNAL(serialPortOpened()), this, SLOT(OnSerialPortOpened()));
	disconnect(_comThread, SIGNAL(serialPortClosed()), this, SLOT(OnSerialPortClosed()));
	disconnect(_comThread, SIGNAL(droneAlive()), this, SLOT(OnDroneAlive()));
	disconnect(_comThread, SIGNAL(droneDownlink()), this, SLOT(OnDroneDownlink()));
	disconnect(_comThread, SIGNAL(droneAttitudeUpdate(float, float, float)), this, SLOT(OnDroneAttitudeUpdate(float, float, float)));
	disconnect(_comThread, SIGNAL(droneMotorsUpdate(quint32, quint32, quint32, quint32)), this, SLOT(OnDroneMotorsUpdate(quint32, quint32, quint32, quint32)));
	disconnect(_comThread, SIGNAL(rollPidParamsUpdate(float, float, float, float, float, float)), this, SLOT(OnRollPidParams(float, float, float, float, float, float)));
	disconnect(_comThread, SIGNAL(pitchPidParamsUpdate(float, float, float, float, float, float)), this, SLOT(OnPitchPidParams(float, float, float, float, float, float)));
	disconnect(_comThread, SIGNAL(yawPidParamsUpdate(float, float, float, float, float, float)), this, SLOT(OnYawPidParams(float, float, float, float, float, float)));

	_comThread->deleteLater();
	_comThread = nullptr;
}