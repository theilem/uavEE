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
/**
 * @file XPlaneRosNode.h
 * @author Mirco Theile, mircot@illinois.edu
 * @date [DD.MM.YYYY] 27.3.2018
 * @brief
 */
#ifndef XPLANEROSNODE_H
#define XPLANEROSNODE_H

#include <ros/ros.h>

#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <boost/property_tree/ptree.hpp>

#include "xPlane/CHeaders/XPLM/XPLMDataAccess.h"

#include <simulation_interface/actuation.h>

class IScheduler;

class XPlaneRosNode: public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "ros_node";

	XPlaneRosNode();

	~XPlaneRosNode();

	static std::shared_ptr<XPlaneRosNode>
	create(const boost::property_tree::ptree& config);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

	void
	enableAutopilot();

	void
	disableAutopilot();

private:

	void
	getSensorData();

	void
	actuate(const simulation_interface::actuation& act);

	ObjectHandle<IScheduler> scheduler_;

	int sensorFrequency_;

	ros::Publisher sensorDataPublisher_;
	ros::Subscriber actuationSubscriber_;

	XPLMDataRef positionRefs_[3];
	XPLMDataRef velocityRefs_[3];
	XPLMDataRef trueAirSpeedRef_;
	XPLMDataRef accelerationRefs_[3];
	XPLMDataRef attitudeRefs_[3];
	XPLMDataRef angularRateRefs_[3];
	XPLMDataRef batteryVoltageRef_;
	XPLMDataRef batteryCurrentRef_;
	XPLMDataRef aileronRef_;
	XPLMDataRef elevatorRef_;
	XPLMDataRef rudderRef_;
	XPLMDataRef throttleRef_;
	XPLMDataRef rpmRef_;

	XPLMDataRef overridesRef_[2];
	XPLMDataRef joystickAttitudeRef_[3];

	unsigned int sequenceNr_;

	bool autopilotActive_;
};

#endif // XPLANEROSNODE_H
