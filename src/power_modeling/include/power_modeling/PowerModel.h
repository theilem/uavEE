////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavEE.
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
 * PowerModel.h
 *
 *  Created on: Jan 5, 2018
 *      Author: mircot
 */

#ifndef POWER_MODELING_INCLUDE_POWER_MODELING_POWERMODEL_H_
#define POWER_MODELING_INCLUDE_POWER_MODELING_POWERMODEL_H_
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <simulation_interface/sensor_data.h>
#include <power_modeling/power_info.h>
#include <radio_comm/local_planner_status.h>
#include <radio_comm/serialized_object.h>
#include <ros/service_server.h>
#include <std_srvs/Trigger.h>
#include <uavAP/Core/DataPresentation/ContentMapping.h>
#include <uavAP/Core/DataPresentation/APDataPresentation/APDataPresentation.h>
#include <uavAP/MissionControl/GlobalPlanner/Trajectory.h>
#include <mutex>

class PowerModel
{
public:

	PowerModel();

	bool
	configure();

private:

	power_modeling::power_info
	calculateAerodynamics(const simulation_interface::sensor_data& sd);

	power_modeling::power_info
	predictAerodynamics();

	void
	onSensorData(const simulation_interface::sensor_data& data);

	void
	onTrajectory(const radio_comm::serialized_object& data);

	void
	onLocalPlannerStatus(const radio_comm::local_planner_status& data);

	bool
	resetPowerService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);

	double loverd_;
	double mass_;
	double gravityConstant_;

	double cumulativeTotalThrustPower_;
	double cumulativePredictedTotalThrustPower_;
	ros::Time lastSensorDataStamp_;
	ros::Duration timeDiff_;

	APDataPresentation<Content, Target> dataPresentation_;

	std::mutex trajectoryMutex_;
	Trajectory trajectory_;
	PathSectionIterator currentPathSection_;

	ros::Publisher thrustPowerPublisher_;
	ros::Publisher predictedPowerPublisher_;

	ros::Subscriber sensorDataSubscriber_;
	ros::Subscriber trajectorySubscriber_;
	ros::Subscriber localPlannerStatusSubscriber_;

	ros::ServiceServer resetPowerService_;

};

#endif /* POWER_MODELING_INCLUDE_POWER_MODELING_POWERMODEL_H_ */
