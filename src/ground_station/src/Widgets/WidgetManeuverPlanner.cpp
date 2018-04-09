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
#include <radio_comm/send_control_override.h>
#include "ground_station/IWidgetInterface.h"
#include "ground_station/Widgets/WidgetManeuverPlanner.h"
#include "ui_WidgetManeuverPlanner.h"
#include <QJsonArray>

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
    if(!interface)
    {
        APLOG_ERROR << "WidgetManeuverPlanner cannot connect to interface";
        return;
    }
    if(!interface->getConfigManager().isSet())
    {
        APLOG_ERROR << "WidgetManeuverPlanner cannot get ConfigManager";
        return;
    }
    configManager_.set(interface->getConfigManager().get());
    ui->missionOptions->addItem("default");
    if(configManager_.isSet())
    {
        auto cm = configManager_.get();
        PropertyMapper pm(cm->getMissionConfig());
        boost::property_tree::ptree planner;
        if (pm.add("planner", planner, true))
        {
            PropertyMapper plannerPm(planner);
            boost::property_tree::ptree maneuvers;
            if (plannerPm.add("maneuvers", maneuvers, true))
            {
                for (const auto& it : maneuvers)
                {
                    ui->maneuverOptions->addItem(QString::fromStdString(it.first));
                }
            }
            boost::property_tree::ptree missions;
            if (plannerPm.add("missions", missions, true))
            {
                for (const auto& it : missions)
                {
                    ui->missionOptions->addItem(QString::fromStdString(it.first));
                }
            }
        }
    }
}

void
WidgetManeuverPlanner::on_apply_clicked()
{
    radio_comm::send_control_override::Request co;
    co.overridemaneuverplanner = ui->overridePlanner->isChecked();

    co.rolltargetvalue = ui->rollTargetValue->text().toDouble() * M_PI / 180;
    co.pitchtargetvalue = ui->pitchTargetValue->text().toDouble() * M_PI / 180;
    co.velocitytargetvalue = ui->velocityTargetValue->text().toDouble();
    co.climbratetargetvalue = ui->climbRateTargetValue->text().toDouble();
    co.yawratetargetvalue = ui->yawRateTargetValue->text().toDouble() * M_PI / 180;

    co.rolloutputvalue = ui->rollOutputValue->text().toDouble();
    co.pitchoutputvalue = ui->pitchOutputValue->text().toDouble();
    co.yawoutputvalue = ui->yawOutputValue->text().toDouble();
    co.throttleoutputvalue = ui->throttleOutputValue->text().toDouble();
    co.flapoutputvalue = ui->flapOutputValue->text().toDouble();

    co.rolltargetoverride = ui->rollTargetOverride->isChecked();
    co.pitchtargetoverride = ui->pitchTargetOverride->isChecked();
    co.velocitytargetoverride = ui->velocityTargetOverride->isChecked();
    co.climbratetargetoverride = ui->climbRateTargetOverride->isChecked();
    co.yawratetargetoverride = ui->yawRateTargetOverride->isChecked();

    co.rolloutputoverride = ui->rollOutputOverride->isChecked();
    co.pitchoutputoverride = ui->pitchOutputOverride->isChecked();
    co.yawoutputoverride = ui->yawOutputOverride->isChecked();
    co.throttleoutputoverride = ui->throttleOutputOverride->isChecked();
    co.flapoutputoverride = ui->flapOutputOverride->isChecked();

    co.activate = ui->activate->isChecked();

    configManager_.get()->sendManeuverOverride(co);
}

void
WidgetManeuverPlanner::on_sendManeuver_clicked()
{
    if (!ui->maneuverOptions->currentText().isEmpty())
    {
        configManager_.get()->sendManeuverSequence(ui->maneuverOptions->currentText().toStdString());
    }
}

void
WidgetManeuverPlanner::on_sendMission_clicked()
{
    configManager_.get()->sendMission(ui->missionOptions->currentText().toStdString());
}
