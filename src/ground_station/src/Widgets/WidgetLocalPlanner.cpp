/*
 * WidgetLocalPlanner.cpp
 *
 *  Created on: Jul 31, 2018
 *      Author: mircot
 */
#include <ground_station/Widgets/WidgetLocalPlanner.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/protobuf/messages/LocalPlanner.pb.h>
#include <uavAP/Core/protobuf/messages/ManeuverPlanner.pb.h>
#include "ground_station/ConfigManager.h"

#include <ui_WidgetLocalPlanner.h>

WidgetLocalPlanner::WidgetLocalPlanner(QWidget* parent) :
		QWidget(parent), ui(new Ui::WidgetLocalPlanner), params_(nullptr)
{
	ui->setupUi(this);

	ui->comboBox->addItem(QString("local_planner"));
	ui->comboBox->addItem(QString("global_planner"));

	LocalPlannerParams params;
	ManeuverPlannerParams man;

}

void
WidgetLocalPlanner::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	configManager_ = interface->getConfigManager();
}

void
WidgetLocalPlanner::on_pushButton_clicked()
{
	auto cm = configManager_.get();
	if (!cm)
	{
		APLOG_ERROR << "Config manager missing. Cannot continue.";
		return;
	}

	params_->Clear();
	ui->widget->populateMessage(*params_);

	APLOG_DEBUG << "Sending params \n" << params_->DebugString();
	switch (currentTuning_)
	{
	case Tuning::LOCAL_PLANNER:
	{
		cm->tuneLocalPlanner(*dynamic_cast<LocalPlannerParams*>(params_));
		break;
	}
	case Tuning::GLOBAL_PLANNER:
	{
		break;
	}
	default:
		return;
	}

}

void
WidgetLocalPlanner::on_comboBox_currentIndexChanged(const QString& name)
{
	if (params_)
		delete params_;

	currentTuning_ = EnumMap < Tuning > ::convert(name.toStdString());
	switch (currentTuning_)
	{
	case Tuning::LOCAL_PLANNER:
	{
		params_ = new LocalPlannerParams();
		break;
	}
	case Tuning::GLOBAL_PLANNER:
	{
		params_ = new ManeuverPlannerParams();
		break;
	}
	default:
		return;
	}
	ui->widget->setProtoLayout(params_->GetDescriptor());
}
