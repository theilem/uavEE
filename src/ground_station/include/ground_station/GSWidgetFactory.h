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
/**
 * @author Mirco Theile, mircot@illinois.edu
 * @file GSWidgetFactory.h
 * @date Jan 9, 2018
 * @brief
 */

#ifndef GROUND_STATION_INCLUDE_GROUND_STATION_GSWIDGETFACTORY_H_
#define GROUND_STATION_INCLUDE_GROUND_STATION_GSWIDGETFACTORY_H_
#include <uavAP/Core/Logging/APLogger.h>
#include <map>
#include <memory>
#include <QWidget>
#include <type_traits>

class IWidgetInterface;

class GSWidgetFactory
{
public:

	GSWidgetFactory();

	QWidget*
	createWidget(const std::string& type, std::shared_ptr<IWidgetInterface> interface = nullptr, QWidget* parent = nullptr);

	std::vector<std::string>
	getWidgetTypes();

private:

	using WidgetCreator = std::function<QWidget*(std::shared_ptr<IWidgetInterface>, QWidget*)>;

	template <typename TYPE>
	typename std::enable_if<std::is_base_of<QWidget, TYPE>::value, void>::type
	addWidget();

	std::map<std::string, WidgetCreator> creators_;

};

template<typename TYPE>
inline typename std::enable_if<std::is_base_of<QWidget, TYPE>::value, void>::type
GSWidgetFactory::addWidget()
{
	APLOG_DEBUG << "Added widget " << std::string(TYPE::widgetName);
	if (creators_.find(std::string(TYPE::widgetName)) != creators_.end())
	{
		APLOG_ERROR << "Widget of that type already exists. Ignore new insertion.";
		return;
	}
	creators_.insert(std::make_pair(std::string(TYPE::widgetName), &TYPE::createGSWidget));
}

#endif /* GROUND_STATION_INCLUDE_GROUND_STATION_GSWIDGETFACTORY_H_ */
