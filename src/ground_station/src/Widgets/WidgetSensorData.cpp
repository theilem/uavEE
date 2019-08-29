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
#include <autopilot_interface/detail/uavAPConversions.h>
#include "ground_station/Widgets/WidgetSensorData.h"
#include "ui_WidgetSensorData.h"
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/SensorData.h>
#include "ground_station/IDataSignals.h"
#include "ground_station/IWidgetInterface.h"
#include <uavAP/Core/Time.h>
#include <uavAP/Core/Frames/InertialFrame.h>
#include <ctime>

WidgetSensorData::WidgetSensorData(QWidget* parent) :
		QWidget(parent), ui(new Ui::WidgetSensorData)
{
	ui->setupUi(this);
}

WidgetSensorData::~WidgetSensorData()
{
	APLOG_DEBUG << "Wdget SensorData deleted";
	delete ui;
}

void
WidgetSensorData::onLocalFrame(const VehicleOneFrame& localFrame)
{
	localFrame_ = localFrame;
}

void
WidgetSensorData::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	if (!interface)
	{
		APLOG_WARN << "WidgetSensorData received null interface";
		return;
	}
	if (auto ds = interface->getIDataSignals().get())
	{
		QObject::connect(ds.get(), SIGNAL(onSensorData(const simulation_interface::sensor_data&)),
				this, SLOT(onSensorData(const simulation_interface::sensor_data&)));
		QObject::connect(ds.get(), SIGNAL(onLocalFrame(const VehicleOneFrame&)),
				this, SLOT(onLocalFrame(const VehicleOneFrame&)));
	}
	else
		APLOG_ERROR << "Cannot connect WidgetSensorData to SensorData. IDataSignals missing.";
}

void
WidgetSensorData::onSensorData(const simulation_interface::sensor_data& data)
{
	QString t;

	SensorData sd = rosToAp(data);

	if (ui->earthFrameCheckBox->isChecked())
	{
		changeFrame(localFrame_, InertialFrame(), sd);
	}

	t = QString::fromStdString("N/A");
	ui->timeValue->setText(t);

	t.sprintf("%10.5f", sd.position.x());
	ui->peValue->setText(t);

	t.sprintf("%10.5f", sd.position.y());
	ui->pnValue->setText(t);

	t.sprintf("%10.5f", sd.position.z());
	ui->puValue->setText(t);

	t.sprintf("%10.5f", sd.velocity.x());
	ui->veValue->setText(t);

	t.sprintf("%10.5f", sd.velocity.y());
	ui->vnValue->setText(t);

	t.sprintf("%10.5f", sd.velocity.z());
	ui->vuValue->setText(t);

	t.sprintf("%10.5f", sd.airSpeed);
	ui->vaValue->setText(t);

	t.sprintf("%10.5f", sd.groundSpeed);
	ui->vgValue->setText(t);

	t.sprintf("%10.5f", sd.acceleration.x());
	ui->auValue->setText(t);

	t.sprintf("%10.5f", sd.acceleration.y());
	ui->avValue->setText(t);

	t.sprintf("%10.5f", sd.acceleration.z());
	ui->awValue->setText(t);

	t.sprintf("%10.5f", sd.attitude.x() * 180 / M_PI);
	ui->rollValue->setText(t);

	t.sprintf("%10.5f", sd.attitude.y() * 180 / M_PI);
	ui->pitchValue->setText(t);

	t.sprintf("%10.5f", sd.attitude.z() * 180 / M_PI);
	ui->yawValue->setText(t);

	t.sprintf("%10.5f", sd.angularRate.x() * 180 / M_PI);
	ui->rollrValue->setText(t);

	t.sprintf("%10.5f", sd.angularRate.y() * 180 / M_PI);
	ui->pitchrValue->setText(t);

	t.sprintf("%10.5f", sd.angularRate.z() * 180 / M_PI);
	ui->yawrValue->setText(t);

	t.sprintf("%10.5f", sd.batteryVoltage);
	ui->voltValue->setText(t);

	t.sprintf("%10.5f", sd.batteryCurrent);
	ui->currValue->setText(t);

	t.sprintf("%10.5f", sd.aileron);
	ui->aileronValue->setText(t);

	t.sprintf("%10.5f", sd.elevator);
	ui->elevatorValue->setText(t);

	t.sprintf("%10.5f", sd.rudder);
	ui->rudderValue->setText(t);

	t.sprintf("%10.5f", sd.throttle * 100);
	ui->throttleValue->setText(t);

	t.sprintf("%10.5f", sd.rpm);
	ui->rpmValue->setText(t);

	update();
}
