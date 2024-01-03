#include "menuwidget.h"
#include <qpainter.h>


MenuWidget::MenuWidget(QWidget* parent) : QLabel(parent)
{
	_name = "";
	_bg = nullptr;
	_highlighted = false;
	_currentIndex = -1;
	_currentEntryIndex = -1;
}


void MenuWidget::addEntry(MenuWidgetEntry e)
{
	_entries.append(e);
}


void MenuWidget::setName(QString name)
{
	_name = name;
}


void MenuWidget::updatePixmap(QRect rectangle, bool repaint)
{
	int height = rectangle.height();
	int width = rectangle.width();

	if (!_bg)
	{
		_bg = new QPixmap(width, height);
	}
	_bg->fill(QColor(0x20, 0x20, 0x20));

	QPainter p(_bg);
	p.setRenderHint(QPainter::Antialiasing);
	QFont f = (QFont("Verdana", 12.0));
	p.setFont(f);

	QPainterPath pathTop;
	pathTop.setFillRule(Qt::WindingFill);
	pathTop.addRoundedRect(QRectF(5, 5, width - 10, height / 2), 10, 10);
	pathTop.addRect(QRectF(5, height / 2 - 20, width - 10, 25));
	p.setPen(QPen(_highlighted ? Qt::yellow : Qt::black, 2));
	p.drawPath(pathTop);
	p.fillPath(pathTop, Qt::gray);
	p.setPen(QPen(Qt::black, 1));
	p.drawText(width / 2 - 25, 30, _name);
	

	QPainterPath pathBottom;
	pathBottom.setFillRule(Qt::WindingFill);
	pathBottom.addRect(QRectF(5, height / 5, width - 10, 20));
	pathBottom.addRoundedRect(QRectF(5, height / 5, width - 10, 4 * height / 5), 10, 10);
	p.setPen(QPen(_highlighted ? Qt::yellow : Qt::black, 2));
	p.drawPath(pathBottom);
	p.fillPath(pathBottom, QColor(0x00, 0x00, 0xAA));

	f = (QFont("Verdana", 10.0));
	p.setFont(f);
	int y = 80;
	p.setPen(QPen(Qt::white, 1));

	int i = 0;
	for (auto& entry : _entries)
	{
		if (i == _currentIndex)
		{
			p.setPen(QPen(Qt::yellow, 2));
			if (_currentEntryIndex != _currentIndex)
			{
				p.drawRect(10, y - 20, 75, 30);
			}
			else
			{
				p.drawRect(75, y - 20, 75, 30);
			}
			p.setPen(QPen(Qt::white, 1));
		}

		p.drawText(QPointF(10, y), entry.displayName);
		drawValue(p, entry.type1, entry.value1, entry.defaultValue1, 90, y);
		drawValue(p, entry.type2, entry.value2, entry.defaultValue2, 180, y);

		y += 40;
		i += 1;
	}

	if (repaint)
	{
		this->repaint();
	}
}


void MenuWidget::drawValue(QPainter& p, MenuWidgetValueType type, QVariant value, QVariant defaultValue, int x, int y)
{
	QVariant valueToDisplay = value == QVariant() ? defaultValue : value;

	switch (type)
	{
		case MenuWidgetValueType::STRING:
		{
			p.drawText(QPointF(x, y), valueToDisplay.toString());
			break;
		}
		case MenuWidgetValueType::DECIMAL:
		{
			float valueMult100 = valueToDisplay.toFloat() * 100.0;
			int iValueMult100 = (int)valueMult100;
			float round = iValueMult100 / 100.0f;
			p.drawText(QPointF(x, y), QString::number(round));
			break;
		}
		case MenuWidgetValueType::INTEGER:
		{
			p.drawText(QPointF(x, y), QString::number(valueToDisplay.toInt()));
			break;
		}
		case MenuWidgetValueType::LIST:
		{
			int index = defaultValue.toInt();
			QStringList list = value.toStringList();
			if (index < list.size())
			{
				p.drawText(QPointF(x, y), list.at(index));
			}
			break;
		}
		case MenuWidgetValueType::LED:
		{
			QPainterPath ledPath;
			ledPath.addEllipse(QRectF(x - 10, y - 15, 20, 20));
			MenuWidgetLedColor valueColor = MenuWidgetLedColor(valueToDisplay.toInt());
			p.fillPath(ledPath, valueColor == MenuWidgetLedColor::RED ? Qt::red :
				valueColor == MenuWidgetLedColor::YELLOW ? Qt::yellow : Qt::green);
			break;
		}
		case MenuWidgetValueType::NONE:
		default:
			break;
	}
}


void MenuWidget::setMenuItemValue1(quint32 id, QVariant newValue)
{
	for (auto& entry : _entries)
	{
		if (entry.id == id)
		{
			entry.value1 = newValue;
			break;
		}
	}

	updatePixmap(rect(), true);
}


void MenuWidget::setMenuItemValue2(quint32 id, QVariant newValue)
{
	for (auto& entry : _entries)
	{
		if (entry.id == id)
		{
			entry.value2 = newValue;
			break;
		}
	}

	updatePixmap(rect(), true);
}


void MenuWidget::setMenuItemDefaultValue1(quint32 id, QVariant newDefault)
{
	for (auto& entry : _entries)
	{
		if (entry.id == id)
		{
			entry.defaultValue1 = newDefault;
			break;
		}
	}

	updatePixmap(rect(), true);
}


void MenuWidget::setMenuItemDefaultValue2(quint32 id, QVariant newDefault)
{
	for (auto& entry : _entries)
	{
		if (entry.id == id)
		{
			entry.defaultValue2 = newDefault;
			break;
		}
	}

	updatePixmap(rect(), true);
}


void MenuWidget::setHighlighted(bool highlighted)
{
	_highlighted = highlighted;
	updatePixmap(rect(), true);
}


void MenuWidget::scrollEntryValue(bool isUp, bool isAnalog, float analogRate)
{
	int entryIdx = _currentIndex;
	if (_entries[entryIdx].type1 == MenuWidgetValueType::INTEGER)
	{
		_entries[entryIdx].value1 = _entries[entryIdx].value1.toInt() + (isUp ? 1 : -1);
	}
	else if (_entries[entryIdx].type1 == MenuWidgetValueType::DECIMAL)
	{
		float delta = (isUp ? 0.01 : -0.01);
		if (isAnalog)
		{
			delta = -0.01 * (analogRate * 15);
		}
		_entries[entryIdx].value1 = _entries[entryIdx].value1.toFloat() + delta;
	}
	else if (_entries[entryIdx].type1 == MenuWidgetValueType::LIST)
	{
		int curListIdx = _entries[entryIdx].defaultValue1.toInt();
		QStringList valueList = _entries[entryIdx].value1.toStringList();
		bool notBeyondLimits = isUp ? (curListIdx < valueList.size() - 1) : (curListIdx > 0);
		int valueOnBeyondLimit = isUp ? 0 : valueList.size() - 1;
		if (notBeyondLimits)
		{
			curListIdx += isUp ? 1 : -1;
		}
		else
		{
			curListIdx = valueOnBeyondLimit;
		}

		_entries[entryIdx].defaultValue1 = curListIdx;
	}

	updatePixmap(rect(), true);
}


void MenuWidget::entryUp()
{
	scrollEntryValue(true);
}


void MenuWidget::entryDown()
{
	scrollEntryValue(false);
}


void MenuWidget::analogScrollEntryValue(qint16 val)
{
	float rate = val / 32768.0f;
	scrollEntryValue(rate < 0, true, rate);
}

void MenuWidget::menuUp()
{
	if (_currentIndex > 0)
	{
		_currentIndex -= 1;
	}
	else
	{
		_currentIndex = _entries.size() - 1;
	}

	updatePixmap(rect(), true);
}


void MenuWidget::menuDown()
{
	if (_currentIndex < _entries.size() - 1)
	{
		_currentIndex += 1;
	}
	else
	{
		_currentIndex = 0;
	}

	updatePixmap(rect(), true);
}


void MenuWidget::navigateMenu()
{
	_currentIndex = 0;

	updatePixmap(rect(), true);
}


void MenuWidget::navigateEntry()
{
	_currentEntryIndex = _currentIndex;

	updatePixmap(rect(), true);
}


void MenuWidget::confirmEntryValue()
{
	emit entryConfirmed(_entries[_currentIndex].id, currentValue());
}


void MenuWidget::entryExit()
{
	_currentEntryIndex = -1;

	updatePixmap(rect(), true);
}


void MenuWidget::menuExit()
{
	_currentIndex = -1;
	_currentEntryIndex = -1;

	updatePixmap(rect(), true);
}


void MenuWidget::paintEvent(QPaintEvent* evt)
{
	QLabel::paintEvent(evt);

	QRect rectangle = rect();

	if (_bg)
	{
		QPainter painter(this);
		painter.drawPixmap(rectangle, *_bg);
	}
	else
	{
		updatePixmap(rectangle, false);
	}
}


QVariant MenuWidget::currentValue()
{
	return valueOf(_currentIndex);
}


QVariant MenuWidget::valueOf(quint32 id)
{
	for (auto& entry : _entries)
	{
		if (entry.id == id)
		{
			if (entry.type1 == MenuWidgetValueType::LIST)
			{
				QStringList list = entry.value1.toStringList();
				return QVariant(list.at(entry.defaultValue1.toInt()));
			}
			else
			{
				return entry.value1;
			}
		}
	}
}