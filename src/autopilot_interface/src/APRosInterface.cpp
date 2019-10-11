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
/*
 * AutopilotInterface.cpp
 *
 *  Created on: Nov 25, 2017
 *      Author: mircot
 */
#include <autopilot_interface/detail/uavAPConversions.h>
#include <ros/node_handle.h>
#include <simulation_interface/actuation.h>
#include <uavAP/Core/Logging/APLogger.h>
#include "autopilot_interface/APRosInterface.h"
#include "autopilot_interface/AutopilotInterface/IAutopilotInterface.h"

APRosInterface::APRosInterface()
{
}

APRosInterface::~APRosInterface()
{
}

std::shared_ptr<APRosInterface>
APRosInterface::create(const Configuration& config)
{
	auto interface = std::make_shared<APRosInterface>();
	interface->configure(config);
	return interface;
}

bool
APRosInterface::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);
	return pm.map();
}

void
APRosInterface::onSensorData(const simulation_interface::sensor_data& sensorData)
{

	auto data = rosToAp(sensorData);

	auto af = autopilotInterface_.get();
	if (!af)
	{
		APLOG_ERROR << "Autopilot interface missing. Cannot send sensor data.";
		return;
	}

	af->sendSensorData(data);
}

void
APRosInterface::onThrustPower(const power_modeling::power_info&)
{

}

void
APRosInterface::onControllerOut(const ControllerOutput& control)
{
	simulation_interface::actuation actuation;

	actuation.rollOutput = control.rollOutput;
	actuation.pitchOutput = control.pitchOutput;
	actuation.yawOutput = control.yawOutput;
	actuation.throttleOutput = control.throttleOutput;
	actuation.sequenceNr = control.sequenceNr;

	actuationPublisherRos_.publish(actuation);
}

void
APRosInterface::notifyAggregationOnUpdate(const Aggregator& agg)
{
	autopilotInterface_.setFromAggregationIfNotSet(agg);
}

bool
APRosInterface::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!autopilotInterface_.isSet())
		{
			APLOG_ERROR << "Autopilot interface missing.";
			return true;
		}
		break;
	}
	case RunStage::NORMAL:
	{
		auto af = autopilotInterface_.get();
		controlConnection_ = af->subscribeOnControllerOut(boost::bind(&APRosInterface::onControllerOut, this, _1));

		ros::NodeHandle nh;

		actuationPublisherRos_ = nh.advertise<simulation_interface::actuation>("/autopilot_interface/actuation", 20);
		sensorDataSubscriptionRos_ = nh.subscribe("/x_plane_interface/sensor_data", 20, &APRosInterface::onSensorData,
				this);
		powerModelSubscriptionRos_ = nh.subscribe("power_model/thrust_power", 20,
				&APRosInterface::onThrustPower, this);
		break;
	}
	case RunStage::FINAL:
		break;
	default:
		break;
	}
	return false;
}
