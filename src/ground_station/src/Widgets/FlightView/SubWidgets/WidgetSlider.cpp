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
#include <ground_station/Widgets/FlightView/SubWidgets/WidgetSlider.h>
#include <QtGui>

WidgetSlider::WidgetSlider(QWidget *parent) :
		QWidget(parent)
{
	lowerBound = -100;
	upperBound = 100;
	current = 0;
	orientation = UP;
}

void
WidgetSlider::configure(const QJsonObject &json)
{
	if (json.contains("current"))
	{
		current = json["current"].toInt();
	}
	if (json.contains("max"))
	{
		upperBound = json["max"].toInt();
	}
	if (json.contains("min"))
	{
		lowerBound = json["min"].toInt();
	}
	if (json.contains("orient"))
	{
		orientation = (orientation_t) json["orient"].toInt();
	}
}

void
WidgetSlider::setOrientation(orientation_t o)
{
	orientation = o;
}

void
WidgetSlider::paintEvent(QPaintEvent *)
{
	const QColor BLACK(0, 0, 0);
	const QColor RED_ORANGE(255, 50, 0);
	const QColor DARK_GRAY(60, 60, 60);
	const QColor WHITE(255, 255, 255);
	int padding = 0;
	QPainter painter(this);
	painter.fillRect(0, 0, this->size().width(), this->size().height(), BLACK);
	if (orientation == UP || orientation == DOWN)
	{
		//int padding = this->size().height()/8;
		int xPadding = padding / 6;
		painter.setBrush(QBrush(DARK_GRAY));
		painter.drawRect(xPadding, padding, this->size().width() - 2 * xPadding,
				this->size().height() - 2 * padding);
		painter.setBrush(QBrush(RED_ORANGE));
		painter.setPen(RED_ORANGE);
		float pxCoeff = (float) (this->size().height() - 2 * padding) / (upperBound - lowerBound);
		int d = pxCoeff * (current - lowerBound);
		int y = (orientation == UP) ? (this->size().height() - padding - d) : (padding + d);
		painter.drawRect(xPadding, y - 2, this->size().width() - 2 * xPadding, 5);
	}
	else
	{
		//int padding = this->size().width()/8;
		int yPadding = padding / 6;
		painter.setBrush(QBrush(DARK_GRAY));
		painter.drawRect(padding, yPadding, this->size().width() - 2 * padding,
				this->size().height() - 2 * yPadding);
		painter.setBrush(WHITE);
		painter.setPen(WHITE);
		int w = this->size().width();
		int h = this->size().height();
		QPoint c = this->rect().center();
		QFont font = painter.font();
		font.setPixelSize(this->height() / 2);
		font.setStyleHint(QFont::Monospace);
		painter.setFont(font);
		QString currentText;
		currentText.sprintf("%5d", current);
		//painter.drawText(QRect(c-QPoint(w,h), QSize(2*w,2*h)),Qt::AlignHCenter | Qt::AlignVCenter , QString("Yaw = ").append(currentText));
		painter.setBrush(QBrush(RED_ORANGE));
		painter.setPen(RED_ORANGE);
		float pxCoeff = (float) (this->size().width() - 2 * padding) / (upperBound - lowerBound);
		int d = pxCoeff * (current - lowerBound);
		int x = (orientation == RIGHT) ? (padding + d) : (this->size().width() - padding - d);
		painter.drawRect(x - 2, yPadding, 5, this->size().height() - 2 * yPadding);
	}
}

void
WidgetSlider::setValue(float value)
{
	//map float value proportionally to int value (range(0, 100) or range(-100, 100))
	int val = int(value * this->upperBound);

	if (val > this->upperBound)
		this->current = this->upperBound;
	else if (val < this->lowerBound)
		this->current = this->lowerBound;
	else
		this->current = val;
}
