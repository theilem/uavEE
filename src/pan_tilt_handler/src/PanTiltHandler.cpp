////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
#include "pan_tilt_handler/PanTiltHandler.h"
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/DataPresentation/APDataPresentation/BinaryToArchive.h>
#include <uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h>
#include <autopilot_interface/detail/uavAPConversions.h>
#include "pan_tilt_handler/PanTiltHandlerWidgetInterface.h"

PanTiltHandler::PanTiltHandler() :
		overrideGPS_(false), overrideTarget_(false), idlemode_(true), sendOutput_(false), target(0,
				0, M_PI / 2, M_PI)
{
	/*auto sender = std::dynamic_pointer_cast<QObject>(AircraftSD);
	 if (sender)
	 QObject::connect(sender.get(), SIGNAL(onSensorData(const SensorData&)), this, SLOT(processAircraftSD(const SensorData&)));
	 else
	 APLOG_ERROR << "AntennaHandler: Couldn't dynamic cast signal sender!";*/
}

std::shared_ptr<PanTiltHandler>
PanTiltHandler::create(const boost::property_tree::ptree& config)
{
	auto pth = std::make_shared<PanTiltHandler>();
	pth->configure(config);
	return pth;
}

bool
PanTiltHandler::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper propertyMapper(config);
	if (!propertyMapper.add("pan_tilt_handler_arduino_path", arduinoPath_, true))
		APLOG_ERROR << "Cannot find pan tilt handler arduino path.";
	widgetInterface_ = std::make_shared<PanTiltHandlerWidgetInterface>();
	return propertyMapper.map();
}

void
PanTiltHandler::notifyAggregationOnUpdate(const Aggregator& agg)
{
	scheduler_.setFromAggregationIfNotSet(agg);
	idc_.setFromAggregationIfNotSet(agg);
	widgetInterface_->notifyAggregationOnUpdate(agg);
}

bool
PanTiltHandler::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		/*if(!configManager_.isSet()){
		 APLOG_ERROR<<"AntennaHandler: ConfigManager not set";
		 return true;
		 }*/
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "AntennaHandler: Scheduler not set";
			return true;
		}
		if (!idc_.isSet())
		{
			APLOG_ERROR << "AntennaHandler: InterDeviceCommunication not set";
			return true;
		}
		break;
	}
	case RunStage::NORMAL:
	{
		pidCascade_ = std::make_shared<PanTiltPIDCascade>(&panTiltIMUData_, &aircraftSensorData_,
				&target, &output);
//        pidCascade_->configure(configManager_.get()->getAntennaConfig());
//        if(configManager_.get()->getAntennaConfig().find("arduino_path")==configManager_.get()->getAntennaConfig().not_found()){
//             APLOG_WARN<<"arduino path not found, PanTiltHandler going into idle mode";
//             break;
//        }
//        std::string arduinopath = configManager_.get()->getAntennaConfig().get_child("arduino_path").get_value<std::string>();
		APLOG_WARN << "arduino path is " << arduinoPath_;
		SerialNetworkParams params(arduinoPath_, 115200, "\r");
		sender_ = idc_.get()->createSender(params);
		if (!sender_.isConnected())
		{
			APLOG_WARN << "arduino path could not be open, PanTiltHandler going into idle mode";
			break;
		}
		idlemode_ = false;
		scheduler_.get()->schedule(std::bind(&PanTiltHandler::calculateControl, this),
				Milliseconds(0), Milliseconds(10));
		break;
	}
	case RunStage::FINAL:
		break;
	default:
		break;
	}
	return false;
}

Vector2
PanTiltHandler::getAntennaLocation() const
{
	return Vector2(panTiltIMUData_.gpsData.position[0], panTiltIMUData_.gpsData.position[1]);
}

double
PanTiltHandler::getAntennaTargetHeading() const
{
	return target.heading - target.headingoffset;
}

double
PanTiltHandler::getAntennaCurrentHeading() const
{
	return panTiltIMUData_.imuData.attitude[2] - target.headingoffset;
}

const PIDParametersMap&
PanTiltHandler::getPIDMap() const
{
	return pidCascade_->getPIDParamMap();
}

bool
PanTiltHandler::tunePID(const PIDTuning& tunePID)
{
	return pidCascade_->tunePID(tunePID.pid, tunePID.params);
}

bool
PanTiltHandler::isIdle() const
{
	return idlemode_;
}

std::shared_ptr<IWidgetInterface>
PanTiltHandler::getInterface() const
{
	return widgetInterface_;
}

void
PanTiltHandler::processAircraftSD(const simulation_interface::sensor_data& sd)
{
	if (idlemode_)
		return;
	aircraftSensorData_ = sd;
	if (!overrideTarget_)
	{
		std::lock_guard<std::mutex> lock(panTiltIMUDataMutex_);
		double x = aircraftSensorData_.position.x - panTiltIMUData_.gpsData.position[0];
		double y = aircraftSensorData_.position.y - panTiltIMUData_.gpsData.position[1];
		double r = sqrt(x * x + y * y);
		//calculate targets here
		target.pitch = atan2(aircraftSensorData_.position.z - panTiltIMUData_.gpsData.position[2],
				r);
		//target.pitch += M_PI;
		if (target.pitch > M_PI)
			target.pitch -= 2 * M_PI;
		else if (target.pitch < -M_PI)
			target.pitch += 2 * M_PI;
		target.pitch *= -1; //NED
		target.heading = atan2(y, x);
		//std::cout<<"Target Heading: "<<target.heading <<" Target Pitch: "<<target.pitch<<std::endl;
		APLOG_TRACE << "x: " << x << " y: " << y << " Target Heading: "
				<< target.heading * 180 / M_PI << " Target Pitch: " << target.pitch * 180 / M_PI;
	}
}

//void
//PanTiltHandler::updateSensorData()
//{
//    emit onSensorData(panTiltIMUData_);
//}

void
PanTiltHandler::calculateControl()
{
	std::lock_guard<std::mutex> lock(panTiltIMUDataMutex_);
	pidCascade_->evaluate();
	emit onPIDStati(apToRos(pidCascade_->getPIDStatus()));
	std::stringstream ss;
	ss << (uint32_t) (1500 + output.panOut * 500) << " "
			<< (uint32_t) (1500 - output.tiltOut * 500);
//    std::string str;
//    BinaryToArchive archive(str);
//    archive << static_cast<uint32_t>(output.panOut*500 + 1500) << static_cast<uint32_t>(output.tiltOut*500 + 1500);
	std::string str = ss.str();
	//APLOG_WARN<<str;
	if (sendOutput_)
		sender_.sendPacket(Packet(str));
}
