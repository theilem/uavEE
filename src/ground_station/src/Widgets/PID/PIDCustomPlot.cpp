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
#include "ground_station/Widgets/PID/PIDCustomPlot.h"
#include "uavAP/Core/Logging/APLogger.h"

PIDCustomPlot::PIDCustomPlot(QWidget*, std::string name) :
		pixelsPerDataPoint_(DEFAULTRESOLUTION), plotTitle_(this), maxValue(0), minValue(0)
{
	plotTitle_.setTextColor(Qt::white);
	// Add graphs for CURRENT and TARGET to QCustomPLot
	currentGraph_ = addGraph();
	targetGraph_ = addGraph();
	//targetGraph_->setName("Target");
	setTitle(name);
	plotLayout()->insertRow(0);
	plotLayout()->addElement(0, 0, &plotTitle_);
	// Make it look cool
	currentGraph_->setPen(QPen(Qt::green, 2));
	targetGraph_->setPen(QPen(Qt::red, 2));
	setBackground(QColor(10, 10, 10));
	axisRect()->setBackground(QColor(10, 10, 10));
	xAxis->setBasePen(QPen(Qt::white));
	yAxis->setBasePen(QPen(Qt::white));
	xAxis->setTickPen(QPen(Qt::white));
	yAxis->setTickPen(QPen(Qt::white));
	xAxis->setSubTickPen(QPen(Qt::white));
	yAxis->setSubTickPen(QPen(Qt::white));
	//xAxis->setTickLabelColor(Qt::white);
	xAxis->setLabelPadding(0);
	yAxis->setTickLabelColor(Qt::white);
}

void
PIDCustomPlot::setTitle(std::string title)
{
	plotTitle_.setText(QString::fromStdString(title));
	//currentGraph_->setName(QString::fromStdString(title));
}

void
PIDCustomPlot::addData(double current, double target)
{
	//Implement rolling window of data points
	int toRemove = currentGraph_->dataCount() - width() / pixelsPerDataPoint_ - 1;
	double time = (double) QDateTime::currentMSecsSinceEpoch();
	if (toRemove > 0)
	{
		double key = currentGraph_->data()->at(toRemove)->key;
		currentGraph_->data()->removeBefore(key);
		targetGraph_->data()->removeBefore(key);
		xAxis->setRange(key, time);
	}
	else
	{
		xAxis->setRange(currentGraph_->data()->at(0)->key, time);
	}
	// Plot current and target vs. time
	currentGraph_->addData(time, current);
	targetGraph_->addData(time, target);
	double max = std::max(current, target);
	maxValue = max > maxValue ? max : maxValue;
	double min = std::min(current, target);
	minValue = min < minValue ? min : minValue;
	yAxis->setRange(minValue, maxValue);
	replot();
}

void
PIDCustomPlot::resetGraph()
{
	currentGraph_->setData(QVector<double>(0), QVector<double>(0));
	targetGraph_->setData(QVector<double>(0), QVector<double>(0));
}

void
PIDCustomPlot::setPixelsPerDataPoint(double resolution)
{
	if (resolution > 0)
		pixelsPerDataPoint_ = resolution;
}

void
PIDCustomPlot::resizeEvent(QResizeEvent *event)
{
	int toRemove = currentGraph_->dataCount() - width() / pixelsPerDataPoint_ - 1;
	if (toRemove > 0)
	{
		double key = currentGraph_->data()->at(toRemove)->key;
		currentGraph_->data()->removeBefore(key);
		targetGraph_->data()->removeBefore(key);
	}\

	//below copied from QCustomPlot::resizeEvent
	Q_UNUSED(event)
	// resize and repaint the buffer:
	setViewport(rect());
	replot(rpQueuedRefresh); // queued refresh is important here, to prevent painting issues in some contexts (e.g. MDI subwindow)
}

void
PIDCustomPlot::mouseReleaseEvent(QMouseEvent *)
{
	if (!currentGraph_->data()->size())
		return;
	maxValue = std::max(currentGraph_->data()->at(0)->value, 0.0);
	minValue = std::min(currentGraph_->data()->at(0)->value, 0.0);
	for (int x = 1; x < currentGraph_->data()->size(); x++)
	{
		double val = currentGraph_->data()->at(x)->value;
		maxValue = val > maxValue ? val : maxValue;
		minValue = val < minValue ? val : minValue;
	}
	for (int x = 0; x < targetGraph_->data()->size(); x++)
	{
		double val = targetGraph_->data()->at(x)->value;
		maxValue = val > maxValue ? val : maxValue;
		minValue = val < minValue ? val : minValue;
	}
	yAxis->setRange(minValue, maxValue);
	replot();
}
