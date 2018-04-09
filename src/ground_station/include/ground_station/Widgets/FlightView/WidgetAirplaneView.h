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
#ifndef WIDGETAIRPLANEVIEW_H
#define WIDGETAIRPLANEVIEW_H

#include "ground_station/Widgets/FlightView/SubWidgets/WidgetLevelMeter.h"
#include "ground_station/Widgets/FlightView/SubWidgets/WidgetSlider.h"
#include "ground_station/Widgets/FlightView/WidgetControlSurface.h"
#include <QWidget>
#include <QLabel>
#include "ground_station/IDataSignals.h"
#include "ground_station/IGSWidget.h"

namespace Ui
{
class WidgetAirplaneView;
}

class WidgetAirplaneView: public QWidget, public IGSWidget
{
Q_OBJECT

public:
	explicit
	WidgetAirplaneView(QWidget *parent = 0);
	~WidgetAirplaneView();
	bool
	connectAggregator(const std::shared_ptr<Aggregator> agg) override;
	WidgetControlSurface * controlSurfaceView;
	WidgetLevelMeter * meterLLL;
	WidgetLevelMeter * meterLL;
	WidgetLevelMeter * meterL;
	WidgetLevelMeter * meterR;
	WidgetLevelMeter * meterRR;
	WidgetLevelMeter * meterRRR;
	WidgetSlider * rpmT;
	WidgetSlider * rpmB;
	QLabel currLLL;
	QLabel maxLLL;
	QLabel nameLLL;
	QLabel currLL;
	QLabel maxLL;
	QLabel nameLL;
	QLabel currL;
	QLabel maxL;
	QLabel nameL;
	QLabel currRRR;
	QLabel maxRRR;
	QLabel nameRRR;
	QLabel currRR;
	QLabel maxRR;
	QLabel nameRR;
	QLabel currR;
	QLabel maxR;
	QLabel nameR;

protected:
	void
	resizeEvent(QResizeEvent *);
	//void paintEvent(QPaintEvent *);

private:
	void
	on_hasNewSample(const simulation_interface::actuation& ad);
	void
	configure(const boost::property_tree::ptree &json);
	Ui::WidgetAirplaneView *ui;
	QLabel battLLL;
	QLabel battLL;
	QLabel battL;
	QLabel battRRR;
	QLabel battRR;
	QLabel battR;
};

#endif // WIDGETAIRPLANEVIEW_H
