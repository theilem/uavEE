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
#ifndef WIDGETRADIOVIEW_H
#define WIDGETRADIOVIEW_H

#include <QWidget>

#include "SubWidgets/WidgetSlider.h"
#include "SubWidgets/WidgetRadioControlStick.h"
#include "SubWidgets/WidgetRadioControlSwitch.h"
#include "ground_station/IDataSignals.h"

class WidgetRadioView: public QWidget
{
Q_OBJECT
public:
	explicit
	WidgetRadioView(QWidget *parent = 0);
	void
	connect(std::shared_ptr<IDataSignals> dataSignals, QJsonObject config);

	WidgetRadioControlStick stickL;
	WidgetRadioControlStick stickR;

	WidgetSlider sliderBLL;
	WidgetSlider sliderBL;
	WidgetSlider sliderBR;
	WidgetSlider sliderBRR;
	WidgetSlider sliderTL;
	WidgetSlider sliderTC;
	WidgetSlider sliderTR;

	WidgetRadioControlSwitch switchLLLLL;
	WidgetRadioControlSwitch switchLLLL;
	WidgetRadioControlSwitch switchLLL;
	WidgetRadioControlSwitch switchLL;
	WidgetRadioControlSwitch switchL;
	WidgetRadioControlSwitch switchR;
	WidgetRadioControlSwitch switchRR;
	WidgetRadioControlSwitch switchRRR;
	WidgetRadioControlSwitch switchRRRR;
	WidgetRadioControlSwitch switchRRRRR;

private:
	void
	on_hasNewSample(const simulation_interface::actuation &a);
	void
	configure(const QJsonObject & json);

protected:
	void
	resizeEvent(QResizeEvent * event);

};

#endif // WIDGETRADIOVIEW_H
