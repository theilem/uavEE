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
 * uavAPConversions.h
 *
 *  Created on: Dec 10, 2017
 *      Author: mircot
 */

#ifndef AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_UAVAPCONVERSIONS_H_
#define AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_UAVAPCONVERSIONS_H_
#include <simulation_interface/sensor_data.h>
#include <radio_comm/pidstati.h>
#include <uavAP/Core/SensorData.h>
#include <uavAP/FlightControl/Controller/PIDController/detail/PIDHandling.h>
#include <radio_comm/local_planner_status.h>
#include <uavAP/Core/protobuf/messages/LocalPlanner.pb.h>

template<typename XYZType>
XYZType
Vector3ToXYZType(const Vector3& vec)
{
	XYZType xyz;
	xyz.x = vec.x();
	xyz.y = vec.y();
	xyz.z = vec.z();
	return xyz;
}

template<typename XYZType>
Vector3
xyzTypeToVector3(const XYZType& vec)
{
	return Vector3(vec.x, vec.y, vec.z);
}

simulation_interface::sensor_data
apToRos(const SensorData& sd)
{
	simulation_interface::sensor_data data;

	data.position = Vector3ToXYZType<geometry_msgs::Point>(sd.position);
	data.attitude = Vector3ToXYZType<geometry_msgs::Vector3>(sd.attitude);
	data.velocity.linear = Vector3ToXYZType<geometry_msgs::Vector3>(sd.velocity);
	data.velocity.angular = Vector3ToXYZType<geometry_msgs::Vector3>(sd.angularRate);
	data.acceleration.linear = Vector3ToXYZType<geometry_msgs::Vector3>(sd.acceleration);
	data.acceleration.angular = Vector3ToXYZType<geometry_msgs::Vector3>(sd.angularAcc);

	data.header.stamp.fromBoost(sd.timestamp);

	return data;
}

SensorData
rosToAp(const simulation_interface::sensor_data& sd)
{
	SensorData data;

	data.position = xyzTypeToVector3(sd.position);
	data.attitude = xyzTypeToVector3(sd.attitude);
	data.velocity = xyzTypeToVector3(sd.velocity.linear);
	data.angularRate = xyzTypeToVector3(sd.velocity.angular);
	data.acceleration = xyzTypeToVector3(sd.acceleration.linear);
	data.angularAcc = xyzTypeToVector3(sd.acceleration.angular);

	data.timestamp = sd.header.stamp.toBoost();

	return data;
}

radio_comm::pidstati
apToRos(const PIDStati& stati)
{
	radio_comm::pidstati data;

	for (auto& it : stati)
	{
		radio_comm::pidstatus status;
		status.id = it.first;
		status.target = it.second.target;
		status.value = it.second.value;
		data.stati.push_back(status);
	}

	return data;
}

radio_comm::airplane_linear_local_planner_status
apToRos(const AirplaneLinearLocalPlannerStatus& status)
{
	radio_comm::airplane_linear_local_planner_status data;

	data.heading_target = status.heading_target();

	return data;
}

radio_comm::velocity_body
apToRos(const VelocityBody& vel)
{
	radio_comm::velocity_body data;

	data.velocity_x = vel.velocity_x();
	data.velocity_y = vel.velocity_y();
	data.velocity_z = vel.velocity_z();

	return data;
}

radio_comm::linear_local_planner_status
apToRos(const LinearLocalPlannerStatus& status)
{
	radio_comm::linear_local_planner_status data;

	if (status.has_airplane_status())
		data.airplane_status = apToRos(status.airplane_status());
	if (status.has_velocity_target())
		data.velocity_target = apToRos(status.velocity_target());

	data.current_path_section = status.current_path_section();
	data.yaw_rate_target = status.yaw_rate_target();

	return data;
}

radio_comm::local_planner_status
apToRos(const LocalPlannerStatus& status)
{
	radio_comm::local_planner_status data;

	if (status.has_linear_status())
		data.linear_status = apToRos(status.linear_status());

	return data;
}

#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_UAVAPCONVERSIONS_H_ */
