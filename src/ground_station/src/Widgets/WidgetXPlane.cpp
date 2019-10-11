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

#include <ctime>
#include <boost/date_time/posix_time/time_formatters.hpp>
#include <uavAP/Core/Frames/InertialFrame.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/SensorData.h>
#include <uavAP/Core/Time.h>
#include <autopilot_interface/detail/uavAPConversions.h>
#include <simulation_interface/wind_layer.h>

#include "ui_WidgetXPlane.h"
#include "ground_station/Widgets/WidgetXPlane.h"
#include "ground_station/IConfigManager.h"
#include "ground_station/IDataSignals.h"
#include "ground_station/IWidgetInterface.h"

WidgetXPlane::WidgetXPlane(QWidget* parent) :
		QWidget(parent), ui(new Ui::WidgetXPlane), gpsFix_(0), autopilotActive_(0), lastGPSFix_(
				false), lastAutopilotActive_(false), sensorDataActive_(false), edit_(true), frameIndex_(
				0), windLayerIndex_(0)
{
	ui->setupUi(this);
	ui->gpsFixValue->setFixedSize(ui->yawRateValue->size());
	ui->autopilotValue->setFixedSize(ui->batteryVoltageValue->size());

	QObject::connect(ui->frameValue, SIGNAL(currentIndexChanged(int)), this,
			SLOT(onFrameIndexChanged(int)));

	QObject::connect(ui->windLayerValue, SIGNAL(currentIndexChanged(int)), this,
			SLOT(onWindLayerIndexChanged(int)));

	disabledStyle_ =
			":disabled { color: white; border-color: rgb(49, 54, 59); padding-left: 0; padding-right: 0 }";
	enabledStyle_ = "padding-left: 0; padding-right: 0";
	enabledStyleButton_ = "background-color: rgb(35, 38, 41); padding-left: 0; padding-right: 0";

	std::unique_lock<std::mutex> lock(editMutex_);
	setEdit(false);
	lock.unlock();
}

WidgetXPlane::~WidgetXPlane()
{
	APLOG_DEBUG << "WidgetXPlane: Widget Deleted.";
	delete ui;
}

void
WidgetXPlane::on_engineStartValue_clicked()
{
	auto configManager = configManager_.get();

	if (!configManager)
	{
		APLOG_ERROR << "WidgetXPlane: configuration manager missing.";
		return;
	}

	configManager->engine(true);
}

void
WidgetXPlane::on_engineStopValue_clicked()
{
	auto configManager = configManager_.get();

	if (!configManager)
	{
		APLOG_ERROR << "WidgetXPlane: configuration manager missing.";
		return;
	}

	configManager->engine(false);
}

void
WidgetXPlane::on_gpsFixValue_clicked()
{
	if (gpsFix_ == 0 || gpsFix_ == 1)
	{
		gpsFix_ = 2;
		ui->gpsFixValue->setText(QString("Acquired"));
	}
	else if (gpsFix_ == 2)
	{
		gpsFix_ = 1;
		ui->gpsFixValue->setText(QString("Lost"));
	}
}

void
WidgetXPlane::on_autopilotValue_clicked()
{
	if (autopilotActive_ == 0 || autopilotActive_ == 1)
	{
		autopilotActive_ = 2;
		ui->autopilotValue->setText(QString("On"));
	}
	else if (autopilotActive_ == 2)
	{
		autopilotActive_ = 1;
		ui->autopilotValue->setText(QString("Off"));
	}
}

void
WidgetXPlane::on_edit_clicked()
{
	std::unique_lock<std::mutex> lock(editMutex_);
	if (edit_)
	{
		lock.unlock();
		return;
	}

	setEdit(true);
	lock.unlock();
}

void
WidgetXPlane::on_cancel_clicked()
{
	std::unique_lock<std::mutex> lock(editMutex_);
	if (!edit_)
	{
		lock.unlock();
		return;
	}

	setEdit(false);
	lock.unlock();
}

void
WidgetXPlane::on_apply_clicked()
{
	std::unique_lock<std::mutex> editlock(editMutex_);
	if (!edit_)
	{
		editlock.unlock();
		return;
	}

	std::unique_lock<std::mutex> frameIndexLock(frameIndexMutex_);
	int frameIndex = frameIndex_;
	frameIndexLock.unlock();

	std::unique_lock<std::mutex> windLayerIndexLock(windLayerIndexMutex_);
	int windLayerIndex = windLayerIndex_;
	windLayerIndexLock.unlock();

	simulation_interface::sensor_data sensorData;

	readText(sensorData.position.x, ui->positionEValue);
	readText(sensorData.position.y, ui->positionNValue);
	readText(sensorData.position.z, ui->positionUValue);

	readText(sensorData.velocity.linear.x, ui->velocityEValue);
	readText(sensorData.velocity.linear.y, ui->velocityNValue);
	readText(sensorData.velocity.linear.z, ui->velocityUValue);

	sensorData.air_speed = std::numeric_limits<double>::quiet_NaN();
	sensorData.ground_speed = std::numeric_limits<double>::quiet_NaN();

	readText(sensorData.acceleration.linear.x, ui->accelerationUValue);
	readText(sensorData.acceleration.linear.y, ui->accelerationVValue);
	readText(sensorData.acceleration.linear.z, ui->accelerationWValue);

	readText(sensorData.attitude.x, ui->rollAngleValue);
	readText(sensorData.attitude.y, ui->pitchAngleValue);
	readText(sensorData.attitude.z, ui->yawAngleValue);

	sensorData.attitude.x = degToRad(sensorData.attitude.x);
	sensorData.attitude.y = degToRad(sensorData.attitude.y);
	sensorData.attitude.z = degToRad(sensorData.attitude.z);

	sensorData.angle_of_attack = std::numeric_limits<double>::quiet_NaN();
	sensorData.angle_of_sideslip = std::numeric_limits<double>::quiet_NaN();

	readText(sensorData.velocity.angular.x, ui->rollRateValue);
	readText(sensorData.velocity.angular.y, ui->pitchRateValue);
	readText(sensorData.velocity.angular.z, ui->yawRateValue);

	sensorData.velocity.angular.x = degToRad(sensorData.velocity.angular.x);
	sensorData.velocity.angular.y = degToRad(sensorData.velocity.angular.y);
	sensorData.velocity.angular.z = degToRad(sensorData.velocity.angular.z);

	std::unique_lock<std::mutex> lastLock(lastMutex_);
	sensorData.gps_fix = lastGPSFix_;
	sensorData.autopilot_active = lastAutopilotActive_;
	lastLock.unlock();

	if (gpsFix_)
	{
		if (gpsFix_ == 1)
		{
			sensorData.gps_fix = false;
		}
		else if (gpsFix_ == 2)
		{
			sensorData.gps_fix = true;
		}
	}

	if (autopilotActive_)
	{
		if (autopilotActive_ == 1)
		{
			sensorData.autopilot_active = false;
		}
		else if (autopilotActive_ == 2)
		{
			sensorData.autopilot_active = true;
		}
	}

	readText(sensorData.battery_voltage, ui->batteryVoltageValue);
	readText(sensorData.battery_current, ui->batteryCurrentValue);

	readText(sensorData.motor_speed, ui->motorSpeedValue);

	readText(sensorData.aileron_level, ui->aileronLevelValue);
	readText(sensorData.elevator_level, ui->elevatorLevelValue);
	readText(sensorData.rudder_level, ui->rudderLevelValue);
	readText(sensorData.throttle_level, ui->throttleLevelValue);

	sensorData.throttle_level /= 100;

	readText(sensorData.precipitation_level, ui->precipitationLevelValue);
	readText(sensorData.storminess_level, ui->storminessLevelValue);

	sensorData.precipitation_level /= 100;
	sensorData.storminess_level /= 100;

	simulation_interface::wind_layer windLayer;

	windLayer.wind_layer_index = windLayerIndex;
	readText(windLayer.wind_altitude, ui->windAltitudeValue);
	readText(windLayer.wind_direction, ui->windDirectionValue);
	readText(windLayer.wind_speed, ui->windSpeedValue);
	readText(windLayer.wind_turbulence, ui->windTurbulenceValue);
	readText(windLayer.wind_shear_direction, ui->windShearDirectionValue);
	readText(windLayer.wind_shear_speed, ui->windShearSpeedValue);

	windLayer.wind_direction = degToRad(windLayer.wind_direction);
	windLayer.wind_turbulence /= 100;
	windLayer.wind_shear_direction = degToRad(windLayer.wind_shear_direction);

	sensorData.wind_layers.push_back(windLayer);

	setEdit(false);
	editlock.unlock();

	if (frameIndex == 0)
	{
		toInertialFrame(sensorData);
	}

	auto configManager = configManager_.get();

	if (!configManager)
	{
		APLOG_ERROR << "WidgetXPlane: configuration manager missing.";
		return;
	}

	configManager->publishGroundStationSensorData(sensorData);
}

void
WidgetXPlane::onFrameIndexChanged(int index)
{
	std::unique_lock<std::mutex> lock(frameIndexMutex_);
	frameIndex_ = index;
	lock.unlock();
}

void
WidgetXPlane::onWindLayerIndexChanged(int index)
{
	std::unique_lock<std::mutex> lock(windLayerIndexMutex_);
	windLayerIndex_ = index;
	lock.unlock();
}

void
WidgetXPlane::onXPlaneSensorData(const simulation_interface::sensor_data& sensorData)
{
	std::unique_lock<std::mutex> lastLock(lastMutex_);
	lastGPSFix_ = sensorData.gps_fix;
	lastAutopilotActive_ = sensorData.autopilot_active;
	lastLock.unlock();

	std::unique_lock<std::mutex> editLock(editMutex_);
	if (edit_)
	{
		return;
	}
	editLock.unlock();

	if (!sensorDataActive_)
	{
		sensorDataActive_ = true;
		setText("-");
	}

	std::string printFormat = "%10.5f";
	QString string;
	SensorData sensorDataAP = rosToAp(sensorData);
	int frameIndex = 0;
	int windLayerIndex = 0;

	std::unique_lock<std::mutex> frameIndexLock(frameIndexMutex_);
	frameIndex = frameIndex_;
	frameIndexLock.unlock();

	if (frameIndex == 0)
	{
		changeFrame(InertialFrame(), localFrame_, sensorDataAP);
	}

	string = QString::fromStdString(
			boost::posix_time::to_simple_string(sensorData.header.stamp.toBoost()));
	ui->timeValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorDataAP.position.x());
	ui->positionEValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorDataAP.position.y());
	ui->positionNValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorDataAP.position.z());
	ui->positionUValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorDataAP.velocity.x());
	ui->velocityEValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorDataAP.velocity.y());
	ui->velocityNValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorDataAP.velocity.z());
	ui->velocityUValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.air_speed);
	ui->airSpeedValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.ground_speed);
	ui->groundSpeedValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.acceleration.linear.x);
	ui->accelerationUValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.acceleration.linear.y);
	ui->accelerationVValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.acceleration.linear.z);
	ui->accelerationWValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorDataAP.attitude.x()));
	ui->rollAngleValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorDataAP.attitude.y()));
	ui->pitchAngleValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorDataAP.attitude.z()));
	ui->yawAngleValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorData.angle_of_attack));
	ui->attackAngleValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorData.angle_of_sideslip));
	ui->SideslipAngleValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorData.velocity.angular.x));
	ui->rollRateValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorData.velocity.angular.y));
	ui->pitchRateValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorData.velocity.angular.z));
	ui->yawRateValue->setText(string);

	if (sensorDataAP.hasGPSFix)
	{
		ui->gpsFixValue->setText("Acquired");
		gpsFix_ = 2;
	}
	else
	{
		ui->gpsFixValue->setText("Lost");
		gpsFix_ = 1;
	}

	if (sensorDataAP.autopilotActive)
	{
		ui->autopilotValue->setText("On");
		autopilotActive_ = 2;
	}
	else
	{
		ui->autopilotValue->setText("Off");
		autopilotActive_ = 1;
	}

	string.sprintf(printFormat.c_str(), sensorData.battery_voltage);
	ui->batteryVoltageValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.battery_current);
	ui->batteryCurrentValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.motor_speed);
	ui->motorSpeedValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.aileron_level);
	ui->aileronLevelValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.elevator_level);
	ui->elevatorLevelValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.rudder_level);
	ui->rudderLevelValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.throttle_level * 100);
	ui->throttleLevelValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.precipitation_level * 100);
	ui->precipitationLevelValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.storminess_level * 100);
	ui->storminessLevelValue->setText(string);

	std::unique_lock<std::mutex> windLayerIndexLock(windLayerIndexMutex_);
	windLayerIndex = windLayerIndex_;
	windLayerIndexLock.unlock();

	if (sensorData.wind_layers.size() >= 3)
	{
		for (const auto& it : sensorData.wind_layers)
		{
			if (it.wind_layer_index == windLayerIndex)
			{
				string.sprintf(printFormat.c_str(), it.wind_altitude);
				ui->windAltitudeValue->setText(string);

				string.sprintf(printFormat.c_str(), radToDeg(it.wind_direction));
				ui->windDirectionValue->setText(string);

				string.sprintf(printFormat.c_str(), it.wind_speed);
				ui->windSpeedValue->setText(string);

				string.sprintf(printFormat.c_str(), it.wind_turbulence * 100);
				ui->windTurbulenceValue->setText(string);

				string.sprintf(printFormat.c_str(), radToDeg(it.wind_shear_direction));
				ui->windShearDirectionValue->setText(string);

				string.sprintf(printFormat.c_str(), it.wind_shear_speed);
				ui->windShearSpeedValue->setText(string);

				break;
			}
		}
	}

	update();
}

void
WidgetXPlane::onLocalFrame(const VehicleOneFrame& localFrame)
{
	localFrame_ = localFrame;
}

void
WidgetXPlane::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	if (!interface)
	{
		APLOG_WARN << "WidgetXPlane: Interface Missing.";
		return;
	}

	if (!interface->getConfigManager().isSet())
	{
		APLOG_ERROR << "WidgetXPlane: configuration manager missing.";
		return;
	}

	configManager_.set(interface->getConfigManager().get());

	if (!configManager_.isSet())
	{
		APLOG_ERROR << "WidgetXPlane: configuration manager missing.";
		return;
	}

	if (auto ds = interface->getIDataSignals().get())
	{
		QObject::connect(ds.get(),
				SIGNAL(onXPlaneSensorData(const simulation_interface::sensor_data&)), this,
				SLOT(onXPlaneSensorData(const simulation_interface::sensor_data&)));
		QObject::connect(ds.get(), SIGNAL(onLocalFrame(const VehicleOneFrame&)), this,
				SLOT(onLocalFrame(const VehicleOneFrame&)));
	}
	else
	{
		APLOG_ERROR << "WidgetXPlane: IDataSignals Missing.";
	}
}

void
WidgetXPlane::toInertialFrame(simulation_interface::sensor_data& sensorData)
{
	SensorData sensorDataAP;
	bool positionInvalid = true;
	bool velocityInvalid = true;
	bool attitudeInvalid = true;

	positionInvalid = (std::isnan(sensorData.position.x) || std::isnan(sensorData.position.y)
			|| std::isnan(sensorData.position.z));
	velocityInvalid =
			(std::isnan(sensorData.velocity.linear.x) || std::isnan(sensorData.velocity.linear.y)
					|| std::isnan(sensorData.velocity.linear.z));
	attitudeInvalid = (std::isnan(sensorData.attitude.x) || std::isnan(sensorData.attitude.y)
			|| std::isnan(sensorData.attitude.z));

	if (positionInvalid)
	{
		sensorData.position.x = 0;
		sensorData.position.y = 0;
		sensorData.position.z = 0;
	}

	if (velocityInvalid)
	{
		sensorData.velocity.linear.x = 0;
		sensorData.velocity.linear.y = 0;
		sensorData.velocity.linear.z = 0;
	}

	if (attitudeInvalid)
	{
		sensorData.attitude.x = 0;
		sensorData.attitude.y = 0;
		sensorData.attitude.z = 0;
	}

	sensorDataAP = rosToAp(sensorData);

	changeFrame(localFrame_, InertialFrame(), sensorDataAP);

	sensorData.position = Vector3ToXYZType<geometry_msgs::Point>(sensorDataAP.position);
	sensorData.velocity.linear = Vector3ToXYZType<geometry_msgs::Vector3>(sensorDataAP.velocity);
	sensorData.attitude = Vector3ToXYZType<geometry_msgs::Vector3>(sensorDataAP.attitude);

	if (positionInvalid)
	{
		sensorData.position.x = std::numeric_limits<double>::quiet_NaN();
		sensorData.position.y = std::numeric_limits<double>::quiet_NaN();
		sensorData.position.z = std::numeric_limits<double>::quiet_NaN();
	}

	if (velocityInvalid)
	{
		sensorData.velocity.linear.x = std::numeric_limits<double>::quiet_NaN();
		sensorData.velocity.linear.y = std::numeric_limits<double>::quiet_NaN();
		sensorData.velocity.linear.z = std::numeric_limits<double>::quiet_NaN();
	}

	if (attitudeInvalid)
	{
		sensorData.attitude.x = std::numeric_limits<double>::quiet_NaN();
		sensorData.attitude.y = std::numeric_limits<double>::quiet_NaN();
		sensorData.attitude.z = std::numeric_limits<double>::quiet_NaN();
	}
}

void
WidgetXPlane::readText(double& sensorData, const QLineEdit* lineEdit)
{
	if (!lineEdit)
	{
		sensorData = std::numeric_limits<double>::quiet_NaN();
		return;
	}

	bool valid = false;
	double result = lineEdit->text().toDouble(&valid);

	if (valid)
	{
		sensorData = result;
	}
	else
	{
		sensorData = std::numeric_limits<double>::quiet_NaN();
	}
}

void
WidgetXPlane::setText(const QString& text)
{
	ui->positionEValue->setText(text);
	ui->positionNValue->setText(text);
	ui->positionUValue->setText(text);

	ui->velocityEValue->setText(text);
	ui->velocityNValue->setText(text);
	ui->velocityUValue->setText(text);

	ui->airSpeedValue->setText(text);
	ui->groundSpeedValue->setText(text);

	ui->accelerationUValue->setText(text);
	ui->accelerationVValue->setText(text);
	ui->accelerationWValue->setText(text);

	ui->rollAngleValue->setText(text);
	ui->pitchAngleValue->setText(text);
	ui->yawAngleValue->setText(text);

	ui->attackAngleValue->setText(text);
	ui->SideslipAngleValue->setText(text);

	ui->rollRateValue->setText(text);
	ui->pitchRateValue->setText(text);
	ui->yawRateValue->setText(text);

	ui->gpsFixValue->setText(text);
	ui->autopilotValue->setText(text);

	ui->batteryVoltageValue->setText(text);
	ui->batteryCurrentValue->setText(text);

	ui->motorSpeedValue->setText(text);

	ui->aileronLevelValue->setText(text);
	ui->elevatorLevelValue->setText(text);
	ui->rudderLevelValue->setText(text);
	ui->throttleLevelValue->setText(text);

	ui->precipitationLevelValue->setText(text);
	ui->storminessLevelValue->setText(text);

	ui->windAltitudeValue->setText(text);
	ui->windDirectionValue->setText(text);
	ui->windSpeedValue->setText(text);
	ui->windTurbulenceValue->setText(text);

	ui->windShearDirectionValue->setText(text);
	ui->windShearSpeedValue->setText(text);
}

void
WidgetXPlane::setStyle(const bool& edit)
{
	QString style = "";

	if (edit)
	{
		style = enabledStyle_;
		clear();
	}
	else
	{
		style = disabledStyle_;

		if (sensorDataActive_)
		{
			setText("-");
		}
		else
		{
			setText("N/A");
		}
	}

	ui->positionEValue->setStyleSheet(style);
	ui->positionNValue->setStyleSheet(style);
	ui->positionUValue->setStyleSheet(style);

	ui->velocityEValue->setStyleSheet(style);
	ui->velocityNValue->setStyleSheet(style);
	ui->velocityUValue->setStyleSheet(style);

	ui->airSpeedValue->setStyleSheet(style);
	ui->groundSpeedValue->setStyleSheet(style);

	ui->accelerationUValue->setStyleSheet(style);
	ui->accelerationVValue->setStyleSheet(style);
	ui->accelerationWValue->setStyleSheet(style);

	ui->rollAngleValue->setStyleSheet(style);
	ui->pitchAngleValue->setStyleSheet(style);
	ui->yawAngleValue->setStyleSheet(style);

	ui->attackAngleValue->setStyleSheet(style);
	ui->SideslipAngleValue->setStyleSheet(style);

	ui->rollRateValue->setStyleSheet(style);
	ui->pitchRateValue->setStyleSheet(style);
	ui->yawRateValue->setStyleSheet(style);

	if (edit)
	{
		ui->gpsFixValue->setStyleSheet(enabledStyleButton_);
		ui->autopilotValue->setStyleSheet(enabledStyleButton_);
	}
	else
	{
		ui->gpsFixValue->setStyleSheet(style);
		ui->autopilotValue->setStyleSheet(style);
	}

	ui->batteryVoltageValue->setStyleSheet(style);
	ui->batteryCurrentValue->setStyleSheet(style);

	ui->motorSpeedValue->setStyleSheet(style);

	ui->aileronLevelValue->setStyleSheet(style);
	ui->elevatorLevelValue->setStyleSheet(style);
	ui->rudderLevelValue->setStyleSheet(style);
	ui->throttleLevelValue->setStyleSheet(style);

	ui->precipitationLevelValue->setStyleSheet(style);
	ui->storminessLevelValue->setStyleSheet(style);

	ui->windAltitudeValue->setStyleSheet(style);
	ui->windDirectionValue->setStyleSheet(style);
	ui->windSpeedValue->setStyleSheet(style);
	ui->windTurbulenceValue->setStyleSheet(style);

	ui->windShearDirectionValue->setStyleSheet(style);
	ui->windShearSpeedValue->setStyleSheet(style);
}

void
WidgetXPlane::setEdit(const bool& edit)
{
	if (edit == edit_)
	{
		return;
	}

	edit_ = edit;

	setStyle(edit);

	ui->positionEValue->setEnabled(edit);
	ui->positionNValue->setEnabled(edit);
	ui->positionUValue->setEnabled(edit);

	ui->velocityEValue->setEnabled(edit);
	ui->velocityNValue->setEnabled(edit);
	ui->velocityUValue->setEnabled(edit);

	ui->airSpeedValue->setEnabled(false);
	ui->groundSpeedValue->setEnabled(false);

	ui->accelerationUValue->setEnabled(edit);
	ui->accelerationVValue->setEnabled(edit);
	ui->accelerationWValue->setEnabled(edit);

	ui->rollAngleValue->setEnabled(edit);
	ui->pitchAngleValue->setEnabled(edit);
	ui->yawAngleValue->setEnabled(edit);

	ui->attackAngleValue->setEnabled(false);
	ui->SideslipAngleValue->setEnabled(false);

	ui->rollRateValue->setEnabled(edit);
	ui->pitchRateValue->setEnabled(edit);
	ui->yawRateValue->setEnabled(edit);

	ui->gpsFixValue->setEnabled(edit);
	ui->autopilotValue->setEnabled(edit);

	ui->batteryVoltageValue->setEnabled(edit);
	ui->batteryCurrentValue->setEnabled(edit);

	ui->motorSpeedValue->setEnabled(edit);

	ui->aileronLevelValue->setEnabled(edit);
	ui->elevatorLevelValue->setEnabled(edit);
	ui->rudderLevelValue->setEnabled(edit);
	ui->throttleLevelValue->setEnabled(edit);

	ui->precipitationLevelValue->setEnabled(edit);
	ui->storminessLevelValue->setEnabled(edit);

	ui->windAltitudeValue->setEnabled(edit);
	ui->windDirectionValue->setEnabled(edit);
	ui->windSpeedValue->setEnabled(edit);
	ui->windTurbulenceValue->setEnabled(edit);

	ui->windShearDirectionValue->setEnabled(edit);
	ui->windShearSpeedValue->setEnabled(edit);
}

void
WidgetXPlane::clear()
{
	setText("");

	gpsFix_ = 0;
	autopilotActive_ = 0;
}
