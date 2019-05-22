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
#ifndef WIDGETMANEUVERPLANNER_H
#define WIDGETMANEUVERPLANNER_H

#include <uavAP/MissionControl/ManeuverPlanner/Override.h>

#include "ground_station/Widgets/GenericProto/NamedLineEdit.h"
#include "ground_station/ConfigManager.h"
#include <QWidget>

namespace Ui
{
class WidgetManeuverPlanner;
}

class IWidgetInterface;

class WidgetManeuverPlanner: public QWidget
{
Q_OBJECT

public:

	static constexpr auto widgetName = "maneuver_planner";

	explicit
	WidgetManeuverPlanner(QWidget* parent = 0);
	~WidgetManeuverPlanner();

	static QWidget*
	createGSWidget(std::shared_ptr<IWidgetInterface> interface, QWidget* parent);

	bool
	configure(const boost::property_tree::ptree& config);

private slots:
	void
	on_apply_clicked();

	void
	on_abort_clicked();

	void
	on_sendManeuver_clicked();

	void
	on_sendMission_clicked();

private:
	void
	connectInterface(std::shared_ptr<IWidgetInterface> interface);
	ObjectHandle<IConfigManager> configManager_;
	Ui::WidgetManeuverPlanner* ui;

	std::map<LocalPlannerTargets, NamedLineEdit*> localPlannerTargets_;
	std::map<ControllerTargets, NamedLineEdit*> controllerTargets_;
	std::map<PIDs, NamedLineEdit*> pids_;
	std::map<ControllerOutputs, NamedLineEdit*> controllerOutputs_;
	std::map<ControllerConstraints, NamedLineEdit*> controllerConstraints_;
	std::map<CustomOverrideIDs, NamedLineEdit*> custom_;
};

#endif // WIDGETMANEUVERPLANNER_H
