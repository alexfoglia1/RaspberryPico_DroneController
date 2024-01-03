#ifndef MENUWIDGET_H
#define MENUWIDGET_H

#include <qlabel.h>
#include <qvariant.h>
#include <qlist.h>


enum class MenuWidgetValueType
{
	INTEGER,
	DECIMAL,
	STRING,
	LIST,
	LED,
	NONE
};


enum class MenuWidgetLedColor : quint8
{
	RED,
	YELLOW,
	GREEN
};


struct MenuWidgetEntry
{
	quint32 id;
	QString displayName;
	MenuWidgetValueType type1;
	QVariant value1;
	MenuWidgetValueType type2;
	QVariant value2;
	QVariant defaultValue1;
	QVariant defaultValue2;
};


class MenuWidget : public QLabel
{
	Q_OBJECT

public:
	MenuWidget(QWidget* parent = nullptr);

	void addEntry(MenuWidgetEntry e);
	void setName(QString name);
	void setMenuItemValue1(quint32 id, QVariant newValue);
	void setMenuItemValue2(quint32 id, QVariant newValue);
	void setMenuItemDefaultValue1(quint32 id, QVariant newDefault);
	void setMenuItemDefaultValue2(quint32 id, QVariant newDefault);
	void setHighlighted(bool highlighted);
	void menuUp();
	void menuDown();
	void entryUp();
	void entryDown();
	void analogScrollEntryValue(qint16 val);
	void navigateMenu();
	void navigateEntry();
	void confirmEntryValue();
	void entryExit();
	void menuExit();
	
	QVariant currentValue();
	QVariant valueOf(quint32 id);

signals:
	void entryConfirmed(quint32 id, QVariant value1);

protected:
	void paintEvent(QPaintEvent* evt) override;

private:
	QPixmap* _bg;
	QList<MenuWidgetEntry> _entries;
	QString _name;

	bool _highlighted;
	int _currentIndex;
	int _currentEntryIndex;

	void updatePixmap(QRect rectangle, bool repaint);
	void drawValue(QPainter& p, MenuWidgetValueType type, QVariant value, QVariant defaultValue, int x, int y);
	void scrollEntryValue(bool isUp, bool isAnalog=false, float analogRate=0);
};


#endif //MENUWIDGET_H