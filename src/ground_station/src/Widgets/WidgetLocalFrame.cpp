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
 * WidgetLocalFrame.cpp
 *
 *  Created on: Jul 24, 2018
 *      Author: sim
 */

#include "ground_station/Widgets/WidgetLocalFrame.h"
#include "ground_station/IDataSignals.h"
#include "ground_station/IConfigManager.h"

WidgetLocalFrame::WidgetLocalFrame(QWidget* parent) :
		QWidget(parent), ui(new Ui::WidgetLocalFrame)
{
	ui->setupUi(this);
	init_ = false;
}

WidgetLocalFrame::~WidgetLocalFrame()
{
}

void
WidgetLocalFrame::on_reset_clicked()
{
	std::unique_lock<std::mutex> lock(frameMutex_);
	frame_ = frameOriginal_;
	copyLocalFrame(frame_);
	lock.unlock();
}

void
WidgetLocalFrame::on_copy_clicked()
{
	std::unique_lock<std::mutex> lock(frameMutex_);
	copyLocalFrame(frame_);
	lock.unlock();
}

void
WidgetLocalFrame::on_send_clicked()
{
	double originE;
	double originN;
	double originU;
	double orientation;

	std::unique_lock<std::mutex> lock(frameMutex_);
	if (ui->originEValue->text().isEmpty())
	{
		originE = frame_.getOrigin().x();
	}
	else
	{
		originE = ui->originEValue->text().toDouble();
	}

	if (ui->originNValue->text().isEmpty())
	{
		originN = frame_.getOrigin().y();
	}
	else
	{
		originN = ui->originNValue->text().toDouble();
	}

	if (ui->originUValue->text().isEmpty())
	{
		originU = frame_.getOrigin().z();
	}
	else
	{
		originU = ui->originUValue->text().toDouble();
	}

	if (ui->orientationValue->text().isEmpty())
	{
		orientation = frame_.getYaw();
	}
	else
	{
		orientation = ui->orientationValue->text().toDouble() * M_PI / 180;
	}

	VehicleOneFrame frame(orientation, Vector3(originE, originN, originU));

	configManager_.get()->sendLocalFrame(frame);
	frame_ = frame;
	lock.unlock();

	mapLogic_.get()->askForLocalFrame();
}

void
WidgetLocalFrame::onLocalFrame(const VehicleOneFrame& frame)
{
	QString string;

	std::unique_lock<std::mutex> lock(frameMutex_);
	string.sprintf("%10.5f", frame.getOrigin().x());
	string = string.simplified();
	string.replace(" ", "");
	ui->originEDisplay->setText(string);

	string.sprintf("%10.5f", frame.getOrigin().y());
	string = string.simplified();
	string.replace(" ", "");
	ui->originNDisplay->setText(string);

	string.sprintf("%10.5f", frame.getOrigin().z());
	string = string.simplified();
	string.replace(" ", "");
	ui->originUDisplay->setText(string);

	string.sprintf("%10.5f", frame.getYaw() * 180 / M_PI);
	string = string.simplified();
	string.replace(" ", "");
	ui->orientationDisplay->setText(string);

	frame_ = frame;

	if (!init_)
	{
		frameOriginal_ = frame;
		init_ = true;
	}
	lock.unlock();
}

void
WidgetLocalFrame::copyLocalFrame(const VehicleOneFrame& frame)
{
	QString string;

	string.sprintf("%10.5f", frame.getOrigin().x());
	string = string.simplified();
	string.replace(" ", "");
	ui->originEValue->setText(string);

	string.sprintf("%10.5f", frame.getOrigin().y());
	string = string.simplified();
	string.replace(" ", "");
	ui->originNValue->setText(string);

	string.sprintf("%10.5f", frame.getOrigin().z());
	string = string.simplified();
	string.replace(" ", "");
	ui->originUValue->setText(string);

	string.sprintf("%10.5f", frame.getYaw() * 180 / M_PI);
	string = string.simplified();
	string.replace(" ", "");
	ui->orientationValue->setText(string);
}

void
WidgetLocalFrame::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	if (!interface)
	{
		APLOG_ERROR << "WidgetLocalFrame: Cannot Connect to Interface.";
		return;
	}

	if (!interface->getConfigManager().isSet())
	{
		APLOG_ERROR << "WidgetLocalFrame: ConfigManager Missing.";
		return;
	}

	configManager_.set(interface->getConfigManager().get());

	if (!configManager_.isSet())
	{
		APLOG_ERROR << "WidgetLocalFrame: Cannot Get Config Manager.";
		return;
	}

	mapLogic_.set(interface->getMapLogic().get());

	if (!mapLogic_.isSet())
	{
		APLOG_ERROR << "WidgetLocalFrame: Cannot Get Map Logic.";
		return;
	}

	if (auto ds = interface->getIDataSignals().get())
	{
		QObject::connect(ds.get(), SIGNAL(onLocalFrame(const VehicleOneFrame&)), this,
				SLOT(onLocalFrame(const VehicleOneFrame&)));
	}
	else
	{
		APLOG_ERROR << "WidgetLocalFrame: IDataSignals Missing.";
	}
}
