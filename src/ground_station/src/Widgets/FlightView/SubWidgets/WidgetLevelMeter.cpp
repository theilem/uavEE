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
#include <ground_station/Widgets/FlightView/SubWidgets/WidgetLevelMeter.h>
#include <QtGui>

WidgetLevelMeter::WidgetLevelMeter(QWidget *parent) :
		QWidget(parent)
{
	level = 1;
	redThreshold = 0.33;
	yellowThreshold = 0.66;
}

void
WidgetLevelMeter::paintEvent(QPaintEvent *)
{
	const QColor RED(255, 0, 0);
	const QColor GREEN(0, 255, 0);
	const QColor YELLOW(255, 255, 0);
	const QColor DARK_GRAY(60, 60, 60);

	QPainter painter(this);
	painter.fillRect(0, 0, this->size().width(), this->size().height(), DARK_GRAY);
	QColor levelColor =
			(level <= redThreshold) ? RED : ((level <= yellowThreshold) ? YELLOW : GREEN);
	int h = this->size().height() * level;
	painter.fillRect(0, this->size().height() - h, this->size().width(), h, levelColor);
}
