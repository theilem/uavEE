////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavEE.
//
// uavEE is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavEE is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
#ifndef WIDGETRADIOCONTROLSTICK_H
#define WIDGETRADIOCONTROLSTICK_H

#include <QWidget>

class WidgetRadioControlStick: public QWidget
{
Q_OBJECT
public:
	explicit
	WidgetRadioControlStick(QWidget *parent = 0);
	void
	configure(const QJsonObject & json);
	int xCurrent;
	int yCurrent;
	int xMid;
	int yMid;
	int xMin;
	int xMax;
	int yMin;
	int yMax;

	bool reverseX;
	bool reverseY;

	void
	setValue(float x, float y);

protected:
	void
	paintEvent(QPaintEvent * event);

signals:

public slots:

};

#endif // WIDGETRADIOCONTROLSTICK_H
