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
#ifndef WIDGETSLIDER_H
#define WIDGETSLIDER_H

#include <QWidget>

class WidgetSlider: public QWidget
{
Q_OBJECT
public:
	explicit
	WidgetSlider(QWidget *parent = 0);
	void
	configure(const QJsonObject & json);
	enum orientation_t
	{
		UP = 0, RIGHT, DOWN, LEFT
	};
	int lowerBound;
	int upperBound;
	int current;
	void
	setOrientation(orientation_t o);
	void
	setValue(float value);

signals:

public slots:

protected:
	void
	paintEvent(QPaintEvent *);
private:
	orientation_t orientation;

};

#endif // WIDGETSLIDER_H
