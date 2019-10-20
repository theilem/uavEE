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
 * WidgetWindAnalysis.cpp
 *
 *  Created on: Oct 19, 2019
 *      Author: simonyu
 */

#include <uavAP/MissionControl/WindAnalysis/WindAnalysisStatus.h>

#include "ground_station/Widgets/WidgetWindAnalysis.h"
#include "ground_station/IDataSignals.h"
#include "ground_station/IConfigManager.h"

WidgetWindAnalysis::WidgetWindAnalysis(QWidget *parent) :
		QWidget(parent), ui(new Ui::WidgetWindAnalysis)
{
	ui->setupUi(this);
}

WidgetWindAnalysis::~WidgetWindAnalysis()
{
	delete ui;
}

void
WidgetWindAnalysis::onWindAnalysisStatus(const WindAnalysisStatus& windAnalysisStatus)
{
	QString string;

	string.sprintf("%10.5f", windAnalysisStatus.velocity.x());
	string = string.simplified();
	string.replace(" ", "");
	ui->eastDisplay->setText(string);

	string.sprintf("%10.5f", windAnalysisStatus.velocity.y());
	string = string.simplified();
	string.replace(" ", "");
	ui->northDisplay->setText(string);

	string.sprintf("%10.5f", windAnalysisStatus.velocity.z());
	string = string.simplified();
	string.replace(" ", "");
	ui->upDisplay->setText(string);

	string.sprintf("%10.5f", windAnalysisStatus.speed);
	string = string.simplified();
	string.replace(" ", "");
	ui->speedDisplay->setText(string);

	string.sprintf("%10.5f", radToDeg(boundAngleRad(windAnalysisStatus.direction - M_PI)));
	string = string.simplified();
	string.replace(" ", "");
	ui->directionDisplay->setText(string);
}

void
WidgetWindAnalysis::on_autoButton_clicked()
{
	WindAnalysisStatus windAnalysisStatus;

	windAnalysisStatus.reset();

	configManager_.get()->sendWindAnalysisStatus(windAnalysisStatus);
}

void
WidgetWindAnalysis::on_clearButton_clicked()
{
	ui->eastValue->clear();
	ui->northValue->clear();
	ui->upValue->clear();
	ui->speedValue->clear();
	ui->directionValue->clear();
}

void
WidgetWindAnalysis::on_manualButton_clicked()
{
	WindAnalysisStatus windAnalysisStatus;
	bool velocityValid = true;

	if (ui->eastValue->text().isEmpty())
	{
		windAnalysisStatus.velocity.x() = std::numeric_limits<double>::quiet_NaN();
		velocityValid = false;
	}
	else
	{
		windAnalysisStatus.velocity.x() = ui->eastValue->text().toDouble();
	}

	if (ui->northValue->text().isEmpty())
	{
		windAnalysisStatus.velocity.y() = std::numeric_limits<double>::quiet_NaN();
		velocityValid = false;
	}
	else
	{
		windAnalysisStatus.velocity.y() = ui->northValue->text().toDouble();
	}

	if (ui->upValue->text().isEmpty())
	{
		windAnalysisStatus.velocity.z() = std::numeric_limits<double>::quiet_NaN();
		velocityValid = false;
	}
	else
	{
		windAnalysisStatus.velocity.z() = ui->upValue->text().toDouble();
	}

	if (!velocityValid)
	{
		if (ui->speedValue->text().isEmpty())
		{
			windAnalysisStatus.speed = std::numeric_limits<double>::quiet_NaN();
		}
		else
		{
			windAnalysisStatus.speed = ui->speedValue->text().toDouble();
		}

		if (ui->directionValue->text().isEmpty())
		{
			windAnalysisStatus.direction = std::numeric_limits<double>::quiet_NaN();
		}
		else
		{
			windAnalysisStatus.direction = boundAngleRad(
					degToRad(ui->directionValue->text().toDouble()) - M_PI);
		}
	}
	else
	{
		windAnalysisStatus.speed = std::numeric_limits<double>::quiet_NaN();
		windAnalysisStatus.direction = std::numeric_limits<double>::quiet_NaN();
	}

	windAnalysisStatus.manual = true;

	configManager_.get()->sendWindAnalysisStatus(windAnalysisStatus);
}

void
WidgetWindAnalysis::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	if (!interface)
	{
		APLOG_WARN << "WidgetWindAnalysis: Cannot Connect to Interface.";
		return;
	}

	if (!interface->getConfigManager().isSet())
	{
		APLOG_ERROR << "WidgetWindAnalysis: Cannot Get Config Manager from Interface.";
		return;
	}

	configManager_.set(interface->getConfigManager().get());

	if (!configManager_.isSet())
	{
		APLOG_ERROR << "WidgetWindAnalysis: Cannot Get Config Manager.";
		return;
	}

	if (auto ds = interface->getIDataSignals().get())
	{
		QObject::connect(ds.get(), SIGNAL(onWindAnalysisStatus(const WindAnalysisStatus&)), this,
				SLOT(onWindAnalysisStatus(const WindAnalysisStatus&)));
	}
	else
	{
		APLOG_ERROR << "WidgetWindAnalysis: IDataSignals Missing.";
	}
}
