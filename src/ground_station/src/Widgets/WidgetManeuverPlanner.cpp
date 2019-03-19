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
		PropertyMapper pm(cm->getMissionConfig());
		boost::property_tree::ptree planner;
		boost::property_tree::ptree maneuver;
		if (pm.add("mission_planner", planner, true))
		{
			PropertyMapper plannerPm(planner);
			boost::property_tree::ptree missions;
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
			PropertyMapper plannerPm(maneuver);
			boost::property_tree::ptree maneuvers;
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
		if (!it.second->isEmpty())
			override.localPlanner.insert(std::make_pair(it.first, it.second->getDouble()));
	}

	for (const auto& it : controllerTargets_)
	{
		if (!it.second->isEmpty())
			override.controllerTarget.insert(std::make_pair(it.first, it.second->getDouble()));
	}

	for (const auto& it : pids_)
	{
		if (!it.second->isEmpty())
			override.pid.insert(std::make_pair(it.first, it.second->getDouble()));
	}

	for (const auto& it : controllerOutputs_)
	{
		if (!it.second->isEmpty())
			override.output.insert(std::make_pair(it.first, it.second->getDouble()));
	}

	for (const auto& it : controllerConstraints_)
	{
		if (!it.second->isEmpty())
			override.constraint.insert(std::make_pair(it.first, it.second->getDouble()));
	}

	for (const auto& it : custom_)
	{
		if (!it.second->isEmpty())
			override.custom.insert(std::make_pair(it.first, it.second->getDouble()));
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
WidgetManeuverPlanner::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);

	std::vector<LocalPlannerTargets> lp;
	std::vector<ControllerTargets> ct;
	std::vector<PIDs> pids;
	std::vector<ControllerOutputs> out;
	std::vector<ControllerConstraints> constraints;
	std::vector<CustomOverrideIDs> custom;

	std::string overrideGroup;

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

	QVBoxLayout* layoutLP = new QVBoxLayout;
	layoutLP->setMargin(2);
	layoutLP->setSpacing(0);
	for (const auto& it : lp)
	{
		if (it == LocalPlannerTargets::INVALID)
			continue;
		auto edit = new NamedLineEdit(EnumMap<LocalPlannerTargets>::convert(it),
				ui->localPlannerGroup);
		localPlannerTargets_.insert(std::make_pair(it, edit));
		layoutLP->addWidget(edit);
	}
	ui->localPlannerGroup->setLayout(layoutLP);

	QVBoxLayout* layoutCT = new QVBoxLayout;
	layoutCT->setMargin(2);
	layoutCT->setSpacing(0);
	for (const auto& it : ct)
	{
		if (it == ControllerTargets::INVALID)
			continue;
		auto edit = new NamedLineEdit(EnumMap<ControllerTargets>::convert(it),
				ui->controllerTargetsGroup);
		controllerTargets_.insert(std::make_pair(it, edit));
		layoutCT->addWidget(edit);
	}
	ui->controllerTargetsGroup->setLayout(layoutCT);

	QVBoxLayout* layoutPid = new QVBoxLayout;
	layoutPid->setMargin(2);
	layoutPid->setSpacing(0);
	for (const auto& it : pids)
	{
		if (it == PIDs::INVALID)
			continue;
		auto edit = new NamedLineEdit(EnumMap<PIDs>::convert(it), ui->pidsGroup);
		pids_.insert(std::make_pair(it, edit));
		layoutPid->addWidget(edit);
	}
	ui->pidsGroup->setLayout(layoutPid);

	QVBoxLayout* layoutOut = new QVBoxLayout;
	layoutOut->setMargin(2);
	layoutOut->setSpacing(0);
	for (const auto& it : out)
	{
		if (it == ControllerOutputs::INVALID)
			continue;
		auto edit = new NamedLineEdit(EnumMap<ControllerOutputs>::convert(it), ui->outputsGroup);
		controllerOutputs_.insert(std::make_pair(it, edit));
		layoutOut->addWidget(edit);
	}
	ui->outputsGroup->setLayout(layoutOut);

	QVBoxLayout* layoutConstraints = new QVBoxLayout;
	layoutConstraints->setMargin(2);
	layoutConstraints->setSpacing(0);
	for (const auto& it : constraints)
	{
		if (it == ControllerConstraints::INVALID)
			continue;
		auto edit = new NamedLineEdit(EnumMap<ControllerConstraints>::convert(it), ui->constraintsGroup);
		controllerConstraints_.insert(std::make_pair(it, edit));
		layoutConstraints->addWidget(edit);
	}
	ui->constraintsGroup->setLayout(layoutConstraints);

	QVBoxLayout* layoutCustom = new QVBoxLayout;
	layoutCustom->setMargin(2);
	layoutCustom->setSpacing(0);
	for (const auto& it : custom)
	{
		if (it == CustomOverrideIDs::INVALID)
			continue;
		auto edit = new NamedLineEdit(EnumMap<CustomOverrideIDs>::convert(it), ui->customGroup);
		custom_.insert(std::make_pair(it, edit));
		layoutCustom->addWidget(edit);
	}
	ui->customGroup->setLayout(layoutCustom);

	return pm.map();
}

void
WidgetManeuverPlanner::on_sendMission_clicked()
{
	configManager_.get()->sendMission(ui->missionOptions->currentText().toStdString());
}
