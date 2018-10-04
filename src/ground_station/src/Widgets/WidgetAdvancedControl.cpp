/*
 * WidgetAdvancedControl.cpp
 *
 *  Created on: Jul 24, 2018
 *      Author: sim
 */

#include <ui_WidgetAdvancedControl.h>
#include <uavAP/Core/EnumMap.hpp>
#include <uavAP/FlightControl/Controller/AdvancedControl.h>

#include "ground_station/IWidgetInterface.h"
#include "ground_station/Widgets/WidgetAdvancedControl.h"

WidgetAdvancedControl::WidgetAdvancedControl(QWidget* parent) :
		QWidget(parent), ui(new Ui::WidgetAdvancedControl)
{
	ui->setupUi(this);
}

WidgetAdvancedControl::~WidgetAdvancedControl()
{
}

void
WidgetAdvancedControl::on_send_clicked()
{
	radio_comm::send_advanced_control::Request co;

	co.special_sel = ui->specialControl->currentText().toStdString();
	co.camber_sel = ui->camberControl->currentText().toStdString();
	co.throws_sel = ui->throwsControl->currentText().toStdString();

	co.special_val = ui->specialValue->text().toDouble();
	co.camber_val = ui->camberValue->text().toDouble();

	configManager_.get()->sendAdvancedControl(co);
}

void
WidgetAdvancedControl::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	if (!interface)
	{
		APLOG_ERROR << "WidgetAdvancedControl: Cannot Connect to Interface.";
		return;
	}

	if (!interface->getConfigManager().isSet())
	{
		APLOG_ERROR << "WidgetAdvancedControl: ConfigManager Missing.";
		return;
	}

	configManager_.set(interface->getConfigManager().get());

	if (configManager_.isSet())
	{
		auto cm = configManager_.get();

		PropertyMapper pm(cm->getAlvoloConfig());
		boost::property_tree::ptree interface;
		pm.add("interface", interface, true);

		PropertyMapper interfacePm(interface);
		boost::property_tree::ptree specialControl;
		boost::property_tree::ptree camberControl;
		boost::property_tree::ptree throwsControl;

		if (interfacePm.add("special_control", specialControl, true))
		{
			for (const auto& it : specialControl)
			{
				auto specialEnum = EnumMap<SpecialControl>::convert(it.first);

				if (specialEnum == SpecialControl::INVALID)
				{
					APLOG_ERROR << "Invalid Special Control " << it.first;
					continue;
				}

				ui->specialControl->addItem(QString::fromStdString(it.first));
			}
		}

		if (interfacePm.add("camber_offset", camberControl, true))
		{
			for (const auto& it : camberControl)
			{
				auto camberEnum = EnumMap<CamberControl>::convert(it.first);

				if (camberEnum == CamberControl::INVALID)
				{
					APLOG_ERROR << "Invalid Camber Control " << it.first;
					continue;
				}

				ui->camberControl->addItem(QString::fromStdString(it.first));
			}
		}

		if (interfacePm.add("channel_mapping", throwsControl, true))
		{
			for (const auto& it : throwsControl)
			{
				auto throwsEnum = EnumMap<ThrowsControl>::convert(it.first);

				if (throwsEnum == ThrowsControl::INVALID)
				{
					APLOG_ERROR << "Invalid Throws Control " << it.first;
					continue;
				}

				ui->throwsControl->addItem(QString::fromStdString(it.first));
			}
		}
	}
}
