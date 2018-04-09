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
#include "ground_station/Widgets/WidgetSensorData.h"
#include "ui_WidgetSensorData.h"
#include <uavAP/Core/Logging/APLogger.h>
#include "ground_station/IDataSignals.h"
#include "ground_station/IWidgetInterface.h"
#include <uavAP/Core/Time.h>
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
WidgetSensorData::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
    if(!interface)
    {
        APLOG_WARN << "WidgetSensorData received null interface";
        return;
    }
    if (auto ds = interface->getIDataSignals().get())
        QObject::connect(ds.get(), SIGNAL(onSensorData(const simulation_interface::sensor_data&)), this,
                         SLOT(onSensorData(const simulation_interface::sensor_data&)));
    else
        APLOG_ERROR << "Cannot connect WidgetSensorData to SensorData. IDataSignals missing.";
}

void
WidgetSensorData::onSensorData(const simulation_interface::sensor_data& sd)
{
    QString t;

    t.sprintf("%10.5f m", sd.position.x);
    ui->peValue->setText(t);

    t.sprintf("%10.5f m", sd.position.y);
    ui->pnValue->setText(t);

    t.sprintf("%10.5f m", sd.position.z);
    ui->puValue->setText(t);

    t.sprintf("%10.5f m/s", sd.velocity.linear.x);
    ui->veValue->setText(t);

    t.sprintf("%10.5f m/s", sd.velocity.linear.y);
    ui->vnValue->setText(t);

    t.sprintf("%10.5f m/s", sd.velocity.linear.z);
    ui->vuValue->setText(t);

    t.sprintf("%10.5f m/s/s", sd.acceleration.linear.x);
    ui->auValue->setText(t);

    t.sprintf("%10.5f m/s/s", sd.acceleration.linear.y);
    ui->avValue->setText(t);

    t.sprintf("%10.5f m/s/s", sd.acceleration.linear.z);
    ui->awValue->setText(t);

    t.sprintf("%10.5f degrees", sd.attitude.x * 180 / M_PI);
    ui->rollValue->setText(t);

    t.sprintf("%10.5f degrees", sd.attitude.y * 180 / M_PI);
    ui->pitchValue->setText(t);

    t.sprintf("%10.5f degrees", sd.attitude.z * 180 / M_PI);
    ui->yawValue->setText(t);

    t.sprintf("%10.5f degrees/s", sd.velocity.angular.x * 180 / M_PI);
    ui->rollrValue->setText(t);

    t.sprintf("%10.5f degrees/s", sd.velocity.angular.y * 180 / M_PI);
    ui->pitchrValue->setText(t);

    t.sprintf("%10.5f degrees/s", sd.velocity.angular.z * 180 / M_PI);
    ui->yawrValue->setText(t);

    t.sprintf("%10.5f degrees/s/s", sd.acceleration.angular.x * 180 / M_PI);
    ui->rollaValue->setText(t);

    t.sprintf("%10.5f degrees/s/s", sd.acceleration.angular.y * 180 / M_PI);
    ui->pitchaValue->setText(t);

    t.sprintf("%10.5f degrees/s/s", sd.acceleration.angular.z * 180 / M_PI);
    ui->yawaValue->setText(t);

    t.sprintf("not yet implemented");
    ui->vaValue->setText(t);

    t.sprintf("not yet implemented");
    ui->vgValue->setText(t);

    t = QString::fromStdString(boost::posix_time::to_simple_string(sd.header.stamp.toBoost()));
    ui->timeValue->setText(t);

    update();
}
