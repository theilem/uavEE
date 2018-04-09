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
#ifndef WIDGETLEVELMETER_H
#define WIDGETLEVELMETER_H

#include <QWidget>

class WidgetLevelMeter: public QWidget
{
Q_OBJECT
public:
	explicit
	WidgetLevelMeter(QWidget *parent = 0);
	double level;
	double yellowThreshold;
	double redThreshold;

signals:

public slots:

protected:
	void
	paintEvent(QPaintEvent *);

};

#endif // WIDGETLEVELMETER_H
