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

#include <ros/time.h>
#include <uavAP/Core/SensorData.h>
#include <uavAP/FlightControl/Controller/PIDController/PIDHandling.h>
#include <radio_comm/pidstati.h>
#include <radio_comm/velocity_body.h>
#include <simulation_interface/sensor_data.h>

template<typename XYZType>
inline XYZType
Vector3ToXYZType(const Vector3& vec)
{
	XYZType xyz;
	xyz.x = vec.x();
	xyz.y = vec.y();
	xyz.z = vec.z();
	return xyz;
}

template<typename XYZType>
inline Vector3
xyzTypeToVector3(const XYZType& vec)
{
	return Vector3(vec.x, vec.y, vec.z);
}

inline simulation_interface::sensor_data
apToRos(const SensorData& sd)
{
	simulation_interface::sensor_data data;

//	data.header.stamp = ros::Time::fromBoost(sd.timestamp);
	data.header.stamp = ros::Time::now(); // Quick hack
	data.position = Vector3ToXYZType<geometry_msgs::Point>(sd.position);
	data.velocity.linear = Vector3ToXYZType<geometry_msgs::Vector3>(sd.velocity);
	data.velocity.angular = Vector3ToXYZType<geometry_msgs::Vector3>(sd.angularRate);
	data.acceleration.linear = Vector3ToXYZType<geometry_msgs::Vector3>(sd.acceleration);
	data.attitude = Vector3ToXYZType<geometry_msgs::Vector3>(sd.attitude);
	data.air_speed = sd.airSpeed;
	data.ground_speed = sd.groundSpeed;
	data.gps_fix = sd.hasGPSFix;
	data.autopilot_active = sd.autopilotActive;
	data.angle_of_attack = sd.angleOfAttack;
	data.angle_of_sideslip = sd.angleOfSideslip;
	data.battery_voltage = sd.batteryVoltage;
	data.battery_current = sd.batteryCurrent;
	data.aileron_level = sd.aileron;
	data.elevator_level = sd.elevator;
	data.rudder_level = sd.rudder;
	data.throttle_level = sd.throttle;
	data.motor_speed = sd.rpm;

	return data;
}

inline SensorData
rosToAp(const simulation_interface::sensor_data& sd)
{
	SensorData data;

	data.position = xyzTypeToVector3(sd.position);
	data.velocity = xyzTypeToVector3(sd.velocity.linear);
	data.acceleration = xyzTypeToVector3(sd.acceleration.linear);
	data.attitude = xyzTypeToVector3(sd.attitude);
	data.angularRate = xyzTypeToVector3(sd.velocity.angular);
//	data.timestamp = sd.header.stamp.toBoost();
	data.timestamp = Clock::now(); // Quick hack
	data.airSpeed = sd.air_speed;
	data.groundSpeed = sd.ground_speed;
	data.hasGPSFix = sd.gps_fix;
	data.autopilotActive = sd.autopilot_active;
	data.angleOfAttack = sd.angle_of_attack;
	data.angleOfSideslip = sd.angle_of_sideslip;
	data.batteryVoltage = sd.battery_voltage;
	data.batteryCurrent = sd.battery_current;
	data.aileron = sd.aileron_level;
	data.elevator = sd.elevator_level;
	data.rudder = sd.rudder_level;
	data.throttle = sd.throttle_level;
	data.rpm = sd.motor_speed;

	return data;
}

inline radio_comm::pidstati
apToRos(const PIDStati& stati)
{
	radio_comm::pidstati data;

	for (auto& it : stati)
	{
		radio_comm::pidstatus status;
		status.id = static_cast<unsigned int>(it.first);
		status.target = it.second.target;
		status.value = it.second.value;
		data.stati.push_back(status);
	}

	return data;
}

//inline radio_comm::velocity_body
//apToRos(const VelocityBody& vel)
//{
//	radio_comm::velocity_body data;
//
//	data.velocity_x = vel.velocity_x();
//	data.velocity_y = vel.velocity_y();
//	data.velocity_z = vel.velocity_z();
//
//	return data;
//}

#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_UAVAPCONVERSIONS_H_ */
