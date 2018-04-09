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
#include <ground_station/Widgets/FlightView/SubWidgets/WidgetRadioControlSwitch.h>
#include <QtGui>

WidgetRadioControlSwitch::WidgetRadioControlSwitch(QWidget *parent) :
		QWidget(parent)
{
	numPositions = 2;
	currPosition = 0;
	orientation = VERTICAL_ORIENTATION;
}

void
WidgetRadioControlSwitch::configure(const QJsonObject &json)
{
	if (json.contains("positions"))
	{
		numPositions = json["positions"].toInt();
	}
	if (json.contains("current"))
	{
		currPosition = json["current"].toInt();
	}
	if (json.contains("orient"))
	{
		orientation = json["orient"].toBool();
	}
}

void
WidgetRadioControlSwitch::paintEvent(QPaintEvent *)
{
	const QColor BLACK(0, 0, 0);
	const QColor WHITE(255, 255, 255);
	const QColor BLUE(0, 0, 255);
	QPainter painter(this);
	unsigned int segmentSize =
			((orientation == VERTICAL_ORIENTATION) ?
					(this->size().height()) : (this->size().width())) / numPositions;
	unsigned int padding = segmentSize / 32;
	if (padding == 0)
		padding = 1;
	painter.fillRect(0, 0, this->size().width(), this->size().height(), WHITE);
	for (unsigned int i = 0; i < numPositions; i++)
	{
		QColor fillColor = (i == currPosition) ? BLUE : BLACK;
		if (orientation == VERTICAL_ORIENTATION)
		{
			painter.fillRect(padding, padding + i * segmentSize, this->size().width() - 2 * padding,
					segmentSize - 2 * padding, fillColor);
		}
		else
		{
			painter.fillRect(padding + i * segmentSize, padding, segmentSize - 2 * padding,
					this->size().height() - 2 * padding, fillColor);
		}
	}
}
