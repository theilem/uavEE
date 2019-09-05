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
 * WidgetTrimAnalysis.cpp
 *
 *  Created on: Apr 19, 2019
 *      Author: sim
 */

#include "ground_station/Widgets/WidgetTrimAnalysis.h"
#include "ground_station/IDataSignals.h"
#include "ground_station/IConfigManager.h"

WidgetTrimAnalysis::WidgetTrimAnalysis(QWidget *parent) :
		QWidget(parent), ui(new Ui::WidgetTrimAnalysis)
{
	ui->setupUi(this);
}

WidgetTrimAnalysis::~WidgetTrimAnalysis()
{
	delete ui;
}

void
WidgetTrimAnalysis::onControllerOutputTrim(const ControllerOutput& trim)
{
	QString string;

	string.sprintf("%10.5f", trim.rollOutput);
	string = string.simplified();
	string.replace(" ", "");
	ui->rollTrim->setText(string);

	string.sprintf("%10.5f", trim.pitchOutput);
	string = string.simplified();
	string.replace(" ", "");
	ui->pitchTrim->setText(string);

	string.sprintf("%10.5f", trim.yawOutput);
	string = string.simplified();
	string.replace(" ", "");
	ui->yawTrim->setText(string);

	string.sprintf("%10.5f", trim.throttleOutput);
	string = string.simplified();
	string.replace(" ", "");
	ui->throttleTrim->setText(string);
}

void
WidgetTrimAnalysis::on_sendOffset_clicked()
{
	ControllerOutput offset;

	if (ui->rollOffset->text().isEmpty())
	{
		offset.rollOutput = 0;
	}
	else
	{
		offset.rollOutput = ui->rollOffset->text().toDouble();
	}

	if (ui->pitchOffset->text().isEmpty())
	{
		offset.pitchOutput = 0;
	}
	else
	{
		offset.pitchOutput = ui->pitchOffset->text().toDouble();
	}

	if (ui->yawOffset->text().isEmpty())
	{
		offset.yawOutput = 0;
	}
	else
	{
		offset.yawOutput = ui->yawOffset->text().toDouble();
	}

	if (ui->throttleOffset->text().isEmpty())
	{
		offset.throttleOutput = 0;
	}
	else
	{
		offset.throttleOutput = ui->throttleOffset->text().toDouble();
	}

	configManager_.get()->sendControllerOutputOffset(offset);
}

void
WidgetTrimAnalysis::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	if (!interface)
	{
		APLOG_WARN << "WidgetTrimAnalysis: Cannot Connect to Interface.";
		return;
	}

	if (!interface->getConfigManager().isSet())
	{
		APLOG_ERROR << "WidgetTrimAnalysis: Cannot Get Config Manager from Interface.";
		return;
	}

	configManager_.set(interface->getConfigManager().get());

	if (!configManager_.isSet())
	{
		APLOG_ERROR << "WidgetTrimAnalysis: Cannot Get Config Manager.";
		return;
	}

	if (auto ds = interface->getIDataSignals().get())
	{
		QObject::connect(ds.get(), SIGNAL(onControllerOutputTrim(const ControllerOutput&)), this,
				SLOT(onControllerOutputTrim(const ControllerOutput&)));
	}
	else
	{
		APLOG_ERROR << "WidgetTrimAnalysis: IDataSignals Missing.";
	}
}
