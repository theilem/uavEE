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
#ifndef PIDCUSTOMPLOT_H
#define PIDCUSTOMPLOT_H
#include "ground_station/Widgets/QCustomPlot.h"

#define DEFAULTRESOLUTION 10

class PIDCustomPlot: public QCustomPlot
{
public:
	PIDCustomPlot(QWidget* = 0, std::string name = "UNDEFINED");
	void
	setTitle(std::string title);
	void
	addData(double current, double target);
	void
	resetGraph();
	void
	setPixelsPerDataPoint(double resolution);
protected:
	void
	resizeEvent(QResizeEvent *event) override;
	void
	mouseReleaseEvent(QMouseEvent *) override;
private:
	double pixelsPerDataPoint_;
	QCPTextElement plotTitle_;
	QCPGraph * currentGraph_;
	QCPGraph * targetGraph_;
	double maxValue;
	double minValue;
};

#endif // PIDCUSTOMPLOT_H
