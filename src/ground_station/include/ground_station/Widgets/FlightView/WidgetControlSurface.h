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
#ifndef WIDGETCONTROLSURFACE_H
#define WIDGETCONTROLSURFACE_H

#include <QWidget>

#include "ground_station/IDataSignals.h"
#include "ground_station/Widgets/FlightView/SubWidgets/WidgetSlider.h"
#define NUM_CONTROL_SURFACES 7

namespace Ui
{
class WidgetControlSurface;
}

class WidgetControlSurface: public QWidget
{
	friend class WidgetAirplaneView;Q_OBJECT
public:
	explicit
	WidgetControlSurface(QWidget *parent = 0);
	enum control_surface_t
	{
		CS_ELEVATOR_LEFT = 0,
		CS_ELEVATOR_RIGHT,
		CS_RUDDER,
		CS_WING_LEFT_OUTBOARD,
		CS_WING_RIGHT_OUTBOARD,
		CS_WING_LEFT_INBOARD,
		CS_WING_RIGHT_INBOARD
	};
	void
	connect(std::shared_ptr<IDataSignals> dataSignals, const QJsonObject &json);
	void
	setMiddle(control_surface_t surface, quint16 middle);
	void
	setValueAct(control_surface_t surface, quint16 value);
	void
	setValueADC(control_surface_t surface, quint16 value);
	//void setValuePWM(control_surface_t surface, quint16 value);
	void
	setMinimum(control_surface_t surface, quint16 minimum);
	void
	setMaximum(control_surface_t surface, quint16 maximum);
	void
	setReverse(control_surface_t surface, bool r);

protected:
	void
	paintEvent(QPaintEvent * event);

private slots:
	void
	on_hasNewSample(const simulation_interface::actuation& ad);

private:
	void
	configure(const QJsonObject &json);

	quint16 valuesact[NUM_CONTROL_SURFACES];
	quint16 valuesadc[NUM_CONTROL_SURFACES];
	//quint16 valuespwm[NUM_CONTROL_SURFACES];
	quint16 mids[NUM_CONTROL_SURFACES];
	quint16 mins[NUM_CONTROL_SURFACES];
	quint16 maxs[NUM_CONTROL_SURFACES];
	bool reverses[NUM_CONTROL_SURFACES];
	bool adcPrimary, actuationPrimary, adcSecondary, actuationSecondary;
	Ui::WidgetControlSurface *ui;
	int controlMode;
};

#endif // WIDGETCONTROLSURFACE_H
