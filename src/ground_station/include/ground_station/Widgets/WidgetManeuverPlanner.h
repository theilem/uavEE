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

#include <QWidget>
#include <QGroupBox>
#include <QLineEdit>
#include <QHBoxLayout>
#include <uavAP/MissionControl/ManeuverPlanner/Override.h>

#include "ground_station/ConfigManager.h"

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

	static inline QWidget*
	createGSWidget(std::shared_ptr<IWidgetInterface> interface, QWidget* parent)
	{
		auto widget(new WidgetManeuverPlanner(parent));
		widget->connectInterface(interface);
		return widget;
	}

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

	template<typename TYPE>
	void
	createOverrideWidget(const std::vector<TYPE>& override, QGroupBox*& topGroupBox,
			std::map<TYPE, QLineEdit*>& overrideMap);

	ObjectHandle<IConfigManager> configManager_;
	Ui::WidgetManeuverPlanner* ui;

	std::map<LocalPlannerTargets, QLineEdit*> localPlannerTargets_;
	std::map<ControllerTargets, QLineEdit*> controllerTargets_;
	std::map<PIDs, QLineEdit*> pids_;
	std::map<ControllerOutputs, QLineEdit*> controllerOutputs_;
	std::map<ControllerConstraints, QLineEdit*> controllerConstraints_;
	std::map<CustomOverrideIDs, QLineEdit*> custom_;
};

template<typename TYPE>
inline void
WidgetManeuverPlanner::createOverrideWidget(const std::vector<TYPE>& override,
		QGroupBox*& topGroupBox, std::map<TYPE, QLineEdit*>& overrideMap)
{
	unsigned overrideCounter = 0;
	QHBoxLayout* topLayout = new QHBoxLayout;

	topLayout->setMargin(2);
	topLayout->setSpacing(0);

	for (const auto& it : override)
	{
		if (it == TYPE::INVALID)
		{
			continue;
		}

		QGroupBox* overrideGroupBox = new QGroupBox(QString(EnumMap<TYPE>::convert(it).c_str()));
		QGridLayout* overrideLayout = new QGridLayout;
		QLineEdit* overrideLineEdit = new QLineEdit;

		overrideLayout->setMargin(10);
		overrideLayout->setSpacing(0);
		overrideLayout->addWidget(overrideLineEdit);

		overrideGroupBox->setAlignment(Qt::AlignHCenter);
		overrideGroupBox->setLayout(overrideLayout);

		overrideMap.insert(std::make_pair(it, overrideLineEdit));
		topLayout->addWidget(overrideGroupBox);

		overrideCounter++;
	}

	if (overrideCounter == 0)
	{
		delete topLayout;
		topGroupBox->setHidden(true);
	}
	else
	{
		topGroupBox->setLayout(topLayout);
	}
}

#endif // WIDGETMANEUVERPLANNER_H
