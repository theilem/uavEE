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

#include <vector>
#include <string>
#include <QJsonArray>

#include "ground_station/IWidgetInterface.h"
#include "ground_station/Widgets/WidgetManeuverPlanner.h"
#include "ui_WidgetManeuverPlanner.h"

WidgetManeuverPlanner::WidgetManeuverPlanner(QWidget* parent) :
		QWidget(parent), ui(new Ui::WidgetManeuverPlanner)
{
	ui->setupUi(this);
}

WidgetManeuverPlanner::~WidgetManeuverPlanner()
{
	delete ui;
}

void
WidgetManeuverPlanner::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	if (!interface)
	{
		APLOG_ERROR << "WidgetManeuverPlanner cannot connect to interface";
		return;
	}
	if (!interface->getConfigManager().isSet())
	{
		APLOG_ERROR << "WidgetManeuverPlanner cannot get ConfigManager";
		return;
	}
	configManager_.set(interface->getConfigManager().get());
	ui->missionOptions->addItem("default");
	if (configManager_.isSet())
	{
		auto cm = configManager_.get();
		PropertyMapper<Configuration> pm(cm->getMissionConfig());
		Configuration planner;
		Configuration maneuver;
		if (pm.add("mission_planner", planner, true))
		{
			PropertyMapper<Configuration> plannerPm(planner);
			Configuration missions;
			if (plannerPm.add("missions", missions, true))
			{
				for (const auto& it : missions)
				{
					ui->missionOptions->addItem(QString::fromStdString(it.first));
				}
			}
		}
		if (pm.add("maneuver_planner", maneuver, true))
		{
			PropertyMapper<Configuration> plannerPm(maneuver);
			Configuration maneuvers;
			if (plannerPm.add("maneuvers", maneuvers, true))
			{
				for (const auto& it : maneuvers)
				{
					ui->maneuverOptions->addItem(QString::fromStdString(it.first));
				}
			}
		}

		auto config = cm->getWidgetConfigByName(widgetName);
		configure(config);
	}
}

void
WidgetManeuverPlanner::on_apply_clicked()
{
	Override override;

	for (const auto& it : localPlannerTargets_)
	{
		if (!it.second->text().isEmpty())
			override.localPlanner.insert(std::make_pair(it.first, it.second->text().toDouble()));
	}

	for (const auto& it : controllerTargets_)
	{
		if (!it.second->text().isEmpty())
			override.controllerTarget.insert(std::make_pair(it.first, it.second->text().toDouble()));
	}

	for (const auto& it : pids_)
	{
		if (!it.second->text().isEmpty())
			override.pid.insert(std::make_pair(it.first, it.second->text().toDouble()));
	}

	for (const auto& it : controllerOutputs_)
	{
		if (!it.second->text().isEmpty())
			override.output.insert(std::make_pair(it.first, it.second->text().toDouble()));
	}

	for (const auto& it : controllerConstraints_)
	{
		if (!it.second->text().isEmpty())
			override.constraint.insert(std::make_pair(it.first, it.second->text().toDouble()));
	}

	for (const auto& it : custom_)
	{
		if (!it.second->text().isEmpty())
			override.custom.insert(std::make_pair(it.first, it.second->text().toDouble()));
	}

	degreeToRadian(override);

	configManager_.get()->sendOverride(override);
}

void
WidgetManeuverPlanner::on_abort_clicked()
{
	Override override; //Send an empty override

	configManager_.get()->sendOverride(override);
}

void
WidgetManeuverPlanner::on_sendManeuver_clicked()
{
	if (!ui->maneuverOptions->currentText().isEmpty())
	{
		configManager_.get()->sendManeuverSet(ui->maneuverOptions->currentText().toStdString());
	}
}

bool
WidgetManeuverPlanner::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);

	std::vector<LocalPlannerTargets> lp;
	std::vector<ControllerTargets> ct;
	std::vector<PIDs> pids;
	std::vector<ControllerOutputs> out;
	std::vector<ControllerConstraints> constraints;
	std::vector<CustomOverrideIDs> custom;

	std::string overrideGroup;

	unsigned counter = 0;

	for (const auto& it : config)
	{
		overrideGroup = it.first;
		auto overrideGroupEnum = EnumMap<OverrideGroup>::convert(overrideGroup);

		switch (overrideGroupEnum)
		{
		case OverrideGroup::LOCAL_PLANNER:
		{
			pm.addEnumVector(overrideGroup, lp, false);
			break;
		}
		case OverrideGroup::CONTROLLER_TARGETS:
		{
			pm.addEnumVector(overrideGroup, ct, false);
			break;
		}
		case OverrideGroup::PIDS:
		{
			pm.addEnumVector(overrideGroup, pids, false);
			break;
		}
		case OverrideGroup::CONTROLLER_OUTPUTS:
		{
			pm.addEnumVector(overrideGroup, out, false);
			break;
		}
		case OverrideGroup::CONTROLLER_CONSTRAINTS:
		{
			pm.addEnumVector(overrideGroup, constraints, false);
			break;
		}
		case OverrideGroup::CUSTOM:
		{
			pm.addEnumVector(overrideGroup, custom, false);
			break;
		}
		case OverrideGroup::INVALID:
		{
			APLOG_WARN << "WidgetManeuverPlanner: Invalid Override Group: " << overrideGroup;
			break;
		}
		default:
		{
			APLOG_WARN << "WidgetManeuverPlanner: Unknown Override Group: " << overrideGroup;
		}
		}
	}

	ui->verticalLayout_2->setMargin(0);
	ui->verticalLayout_2->setSpacing(0);

	createOverrideWidget(lp, ui->localPlannerGroup, localPlannerTargets_);
	createOverrideWidget(ct, ui->controllerTargetsGroup, controllerTargets_);
	createOverrideWidget(pids, ui->pidsGroup, pids_);
	createOverrideWidget(out, ui->outputsGroup, controllerOutputs_);
	createOverrideWidget(constraints, ui->constraintsGroup, controllerConstraints_);
	createOverrideWidget(custom, ui->customGroup, custom_);

	return pm.map();
}

void
WidgetManeuverPlanner::on_sendMission_clicked()
{
	configManager_.get()->sendMission(ui->missionOptions->currentText().toStdString());
}
