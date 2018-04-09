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
#ifndef WIDGETRADIOCONTROLSWITCH_H
#define WIDGETRADIOCONTROLSWITCH_H

#include <QWidget>

class WidgetRadioControlSwitch: public QWidget
{
Q_OBJECT
public:
	explicit
	WidgetRadioControlSwitch(QWidget *parent = 0);
	void
	configure(const QJsonObject & json);
	unsigned int numPositions;
	unsigned int currPosition;  // 0 to numPositions - 1
	bool orientation;
	const static bool VERTICAL_ORIENTATION = false;
	const static bool HORIZONTAL_ORIENTATION = true;

signals:

public slots:

protected:
	void
	paintEvent(QPaintEvent *);

};

#endif // WIDGETRADIOCONTROLSWITCH_H
