/*
 * WidgetAdvancedControl.cpp
 *
 *  Created on: Jul 24, 2018
 *      Author: sim
 */
#include <ground_station/IWidgetInterface.h>
#include <ground_station/Widgets/WidgetAdvancedControl.h>
#include <ui_WidgetAdvancedControl.h>

WidgetAdvancedControl::WidgetAdvancedControl(QWidget* parent) :
		QWidget(parent), ui(new Ui::WidgetAdvancedControl)
{
	ui->setupUi(this);
}

WidgetAdvancedControl::~WidgetAdvancedControl()
{
}

void
WidgetAdvancedControl::on_apply_clicked()
{
	radio_comm::send_advanced_control::Request co;
	co.special_sel = ui->specialSelVal->text().toStdString();
	co.camber_sel = ui->camberSelVal->text().toStdString();
	co.throws_sel = ui->throwsVal->text().toStdString();

	co.special_val = ui->specialVal->text().toDouble();
	co.camber_val = ui->camberVal->text().toDouble();

	configManager_.get()->sendAdvancedControl(co);
}

void
WidgetAdvancedControl::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	if (!interface)
	{
		APLOG_ERROR << "WidgetAdvancedControl cannot connect to interface";
		return;
	}
	if (!interface->getConfigManager().isSet())
	{
		APLOG_ERROR << "WidgetAdvancedControl cannot get ConfigManager";
		return;
	}
	configManager_.set(interface->getConfigManager().get());
}
