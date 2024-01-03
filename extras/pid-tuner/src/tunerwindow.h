#ifndef TUNERWINDOW_H
#define TUNERWINDOW_H

#include <qmainwindow.h>

#include "ui_tunergui.h"
#include "menuwidget.h"
#include "joystick.h"
#include "comthread.h"


enum class MenuEntryKey : quint32
{
	MENU_KEY_SERIAL_PORT = 0,
	MENU_KEY_BAUD_RATE,
	MENU_KEY_DRONE_STATUS,
	MENU_KEY_JS_STATUS,
	MENU_KEY_ROLL_KP,
	MENU_KEY_ROLL_KI,
	MENU_KEY_ROLL_KT,
	MENU_KEY_ROLL_SAT,
	MENU_KEY_ROLL_AD,
	MENU_KEY_ROLL_BD,
	MENU_KEY_PITCH_KP,
	MENU_KEY_PITCH_KI,
	MENU_KEY_PITCH_KT,
	MENU_KEY_PITCH_SAT,
	MENU_KEY_PITCH_AD,
	MENU_KEY_PITCH_BD,
	MENU_KEY_YAW_KP,
	MENU_KEY_YAW_KI,
	MENU_KEY_YAW_KT,
	MENU_KEY_YAW_SAT,
	MENU_KEY_YAW_AD,
	MENU_KEY_YAW_BD,
};


enum class MenuNavigationStatus : quint32
{
	NAVIGATE_WINDOW,
	NAVIGATE_MENU,
	NAVIGATE_ENTRY,
};


class TunerWindow : public QMainWindow
{
	Q_OBJECT
	typedef bool (TunerWindow::* nav_action)();

	typedef struct navigation_fsm_entry
	{
		MenuNavigationStatus currentStatus;
		js_button buttonPressed;
		MenuNavigationStatus nextStatus;
		nav_action action;
	};

public:
	TunerWindow(QWidget* parent = nullptr);
	void SetCommMenuEntryValue1(MenuEntryKey entryKey, QVariant newValue);
	void SetCommMenuEntryValue2(MenuEntryKey entryKey, QVariant newValue);
	void SetCommMenuEntryDefaultValue1(MenuEntryKey entryKey, QVariant newValue);
	void SetCommMenuEntryDefaultValue2(MenuEntryKey entryKey, QVariant newValue);

public slots:
	void OnJsConnected(bool connected);
	void OnJsBtnPressed(js_button btn);
	void OnJsControl(js_control_packet controlPacket);
	void OnJsThreadExit();
	void OnJsAxisMoved(js_axis axis, qint16 val);
	void OnMenuEntryUpdated(quint32 entryKey, QVariant newValue);
	void OnSerialPortOpened();
	void OnSerialPortClosed();
	void OnDroneAlive();
	void OnDroneDownlink();
	void OnDroneAttitudeUpdate(float roll, float pitch, float yaw);
	void OnDroneMotorsUpdate(quint32 m1, quint32 m2, quint32 m3, quint32 m4);
	void OnRollPidParams(float kp, float ki, float kt, float sat, float ad, float bd);
	void OnPitchPidParams(float kp, float ki, float kt, float sat, float ad, float bd);
	void OnYawPidParams(float kp, float ki, float kt, float sat, float ad, float bd);

private:
	Ui_TunerGui _ui;

	QList<MenuWidgetEntry> _commMenuEntries;
	QList<MenuWidgetEntry> _rollMenuEntries;
	QList<MenuWidgetEntry> _pitchMenuEntries;
	QList<MenuWidgetEntry> _yawMenuEntries;

	MenuNavigationStatus _currentNavigationStatus;
	MenuWidget* _selectedMenuWidget;
	QList<navigation_fsm_entry> _navigationFsm;
	ComThread* _comThread;
	QList<MenuWidget*> _menus;
	int _curMenuIdx;
	bool _motorsArmed;
	bool _radioOverride;

	bool navigateWindowRight();
	bool navigateWindowLeft();
	bool navigateMenuUp();
	bool navigateMenuDown();
	bool navigateEntryUp();
	bool navigateEntryDown();
	bool selectMenu();
	bool selectEntry();
	bool selectEntryValue();
	bool entryExit();
	bool menuExit();
	bool refreshPidParams();
	bool toggleMotorsArmed();
	bool toggleOverrideRadio();
};


#endif
