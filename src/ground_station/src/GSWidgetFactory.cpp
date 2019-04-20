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
/*
 * GSWidgetFactory.cpp
 *
 *  Created on: Jan 9, 2018
 *      Author: mircot
 */

#include "uavAP/Core/Logging/APLogger.h"
#include "ground_station/Widgets/WidgetSteadyStateAnalysis.h"
#include "ground_station/Widgets/QFlightInstruments/WidgetSix.h"
#include "ground_station/Widgets/WidgetSensorData.h"
#include "ground_station/Widgets/PID/WidgetPIDPlots.h"
#include "ground_station/Widgets/WidgetOverheadMap.h"
#include "ground_station/Widgets/PID/WidgetCPGrid.h"
#include "ground_station/Widgets/WidgetManeuverPlanner.h"
#include "ground_station/Widgets/WidgetAdvancedControl.h"
#include "ground_station/Widgets/QFlightInstruments/WidgetPFD.h"
#include "ground_station/Widgets/WidgetLocalFrame.h"
#include "ground_station/Widgets/WidgetLocalPlanner.h"
#include "ground_station/Widgets/WidgetTrimAnalysis.h"
#include "ground_station/GSWidgetFactory.h"
#include "ground_station/IWidgetInterface.h"

GSWidgetFactory::GSWidgetFactory()
{
	//Widgets creatable by the factory need to have a:
	//static const char widgetName[] = "enter_widget_name_here"
	addWidget<WidgetCPGrid>();
	addWidget<WidgetManeuverPlanner>();
	addWidget<WidgetSteadyStateAnalysis>();
	addWidget<WidgetOverheadMap>();
	addWidget<WidgetPFD>();
	addWidget<WidgetPIDPlots>();
	addWidget<WidgetSensorData>();
	addWidget<WidgetSix>();
	addWidget<WidgetAdvancedControl>();
	addWidget<WidgetLocalFrame>();
	addWidget<WidgetLocalPlanner>();
	addWidget<WidgetTrimAnalysis>();
}

QWidget*
GSWidgetFactory::createWidget(const std::string& type, std::shared_ptr<IWidgetInterface> interface,
		QWidget* parent)
{
	auto it = creators_.find(type);
	if (it == creators_.end())
	{
		APLOG_ERROR << "Widget type not found.";
		return nullptr;
	}
	return it->second(interface, parent);
}

std::vector<std::string>
GSWidgetFactory::getWidgetTypes()
{
	std::vector<std::string> vec;
	for (auto& it : creators_)
		vec.push_back(it.first);
	return vec;
}
