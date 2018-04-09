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
#include <ground_station/Widgets/FlightView/SubWidgets/WidgetRadioControlStick.h>
#include <QtGui>

WidgetRadioControlStick::WidgetRadioControlStick(QWidget *parent) :
		QWidget(parent)
{
	xMin = -100;
	xMid = 0;
	xMax = 100;
	xCurrent = 0;
	yMin = -100;
	yMid = 0;
	yMax = 100;
	yCurrent = 0;

	reverseX = false;
	reverseY = false;
}

void
WidgetRadioControlStick::configure(const QJsonObject &json)
{
	if (json.contains("reverseX"))
	{
		reverseX = json["reverseX"].toBool();
	}
	if (json.contains("reverseY"))
	{
		reverseY = json["reverseY"].toBool();
	}

	if (json.contains("centerX"))
	{
		xMid = json["centerX"].toInt();
	}
	if (json.contains("maxX"))
	{
		xMax = json["maxX"].toInt();
	}
	if (json.contains("minX"))
	{
		xMin = json["minX"].toInt();
	}
	if (json.contains("centerY"))
	{
		yMid = json["centerY"].toInt();
	}
	if (json.contains("maxY"))
	{
		yMax = json["maxY"].toInt();
	}
	if (json.contains("minY"))
	{
		yMin = json["minY"].toInt();
	}
}

void
WidgetRadioControlStick::paintEvent(QPaintEvent *)
{
	const QColor BLACK(0, 0, 0);
	const QColor WHITE(255, 255, 255);
	const QColor RED_ORANGE(255, 50, 0);
	const QColor DARK_GRAY(60, 60, 60);

	QPainter painter(this);
	QPoint origin(0, 0);
	int squareSize = this->size().width();
	if (this->size().width() != this->size().height())
	{
		if (this->size().width() > this->size().height())
		{
			squareSize = this->size().height();
			origin.setX((this->size().width() - this->size().height()) / 2);
		}
		else
		{
			origin.setY((this->size().height() - this->size().width()) / 2);
		}
	}
	QPoint center = origin + QPoint(squareSize / 2, squareSize / 2);
	int PADDING = squareSize / 16;
	int circleRadius = (squareSize - 2 * PADDING) / 2;
	squareSize = circleRadius * 2 / sqrt(2);
	origin += QPoint(PADDING, PADDING);
	float pxPerUSX = (float) squareSize / (xMax - xMin);
	float pxPerUSY = (float) squareSize / (yMax - yMin);
	painter.fillRect(origin.x() - PADDING, origin.y() - PADDING, 2 * circleRadius + 2 * PADDING,
			2 * circleRadius + 2 * PADDING, BLACK);
	painter.setBrush(DARK_GRAY);
	painter.drawEllipse(center, circleRadius, circleRadius);
	painter.setPen(QPen(QBrush(WHITE), 1));
	int cx = ((xMax - xMin) / 2 + xMin - xMid) * pxPerUSX;
	int cy = ((yMax - yMin) / 2 + yMin - yMid) * pxPerUSY;
	if (reverseX)
	{
		cx *= -1;
	}
	if (reverseY)
	{
		cy *= -1;
	}
	painter.drawLine(center.x() - squareSize / 2, center.y() + cy, center.x() + squareSize / 2,
			center.y() + cy);
	painter.drawLine(center.x() - cx, center.y() - squareSize / 2, center.x() - cx,
			center.y() + squareSize / 2);
	int x = pxPerUSX * (xCurrent - xMid);
	int y = pxPerUSY * (yCurrent - yMid);
	if (reverseX)
	{
		x *= -1;
	}
	if (reverseY)
	{
		y *= -1;
	}
	QPoint p(center.x() - cx + x, center.y() + cy - y);
	painter.setPen(RED_ORANGE);
	painter.setBrush(RED_ORANGE);
	float rf = circleRadius * 0.08;
	int r = ((int) rf > 0) ? (int) rf : 1;
	painter.drawEllipse(p, r, r);
}

void
WidgetRadioControlStick::setValue(float x, float y)
{
	//map input x, y values proportionally to range(-100, 100)
	//int xVal = int(x * this->xMax);
	//int yVal = int(y * this->yMax);

	int xVal = int(x * 600);   //negligible response unless multiplied by large constant (??)
	int yVal = int(y * -600);

	if (xVal > this->xMax)
		this->xCurrent = this->xMax;
	else if (xVal < this->xMin)
		this->xCurrent = this->xMin;
	else
		this->xCurrent = xVal;

	if (yVal > this->yMax)
		this->yCurrent = this->yMax;
	else if (xVal < this->yMin)
		this->yCurrent = this->yMin;
	else
		this->yCurrent = yVal;
}
