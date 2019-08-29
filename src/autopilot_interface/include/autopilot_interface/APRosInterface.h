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
 * AutopilotInterface.h
 *
 *  Created on: Nov 25, 2017
 *      Author: mircot
 */

#ifndef AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_APROSINTERFACE_H_
#define AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_APROSINTERFACE_H_
#include <boost/signals2.hpp>
#include <power_modeling/power_info.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <simulation_interface/sensor_data.h>

#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>

#include <memory>

class IAutopilotInterface;

class APRosInterface : public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "ros_interface";

	APRosInterface();

	~APRosInterface();

	static std::shared_ptr<APRosInterface>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

private:

	void
	onSensorData(const simulation_interface::sensor_data& sensorData);

	void
	onThrustPower(const power_modeling::power_info&);

	void
	onControllerOut(const ControllerOutput& control);

	ros::Publisher actuationPublisherRos_;
	ros::Subscriber sensorDataSubscriptionRos_;
	ros::Subscriber powerModelSubscriptionRos_;

	boost::signals2::connection controlConnection_;

	ObjectHandle<IAutopilotInterface> autopilotInterface_;

};

#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_APROSINTERFACE_H_ */
