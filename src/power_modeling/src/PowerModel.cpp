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
 * PowerModel.cpp
 *
 *  Created on: Jan 5, 2018
 *      Author: mircot
 */
#include "power_modeling/PowerModel.h"

#include <autopilot_interface/detail/uavAPConversions.h>
#include <ros/node_handle.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/MissionControl/GlobalPlanner/Trajectory.h>

PowerModel::PowerModel() :
		loverd_(0), mass_(0), gravityConstant_(9.80665), cumulativeTotalThrustPower_(0), cumulativePredictedTotalThrustPower_(
				0)
{
}

power_modeling::power_info
PowerModel::calculateAerodynamics(const simulation_interface::sensor_data& sd)
{
	power_modeling::power_info power;

	Vector3 vel = xyzTypeToVector3(sd.velocity.linear);
	Vector3 accel = xyzTypeToVector3(sd.acceleration.linear);

	const double& roll = sd.attitude.x;
	const double& pitch = sd.attitude.y;
	double climbangle = asin(vel[2] / vel.norm());

	power.steady_state_thrust = mass_ * gravityConstant_
			* (cos(climbangle) + loverd_ * sin(climbangle)) / (loverd_ * cos(roll));

	power.steady_state_thrust_power = power.steady_state_thrust * vel.norm();

	power.dynamics_thrust_power = vel.dot(accel) * mass_;

	power.dynamics_thrust = power.dynamics_thrust_power / vel.norm();

	power.total_thrust = power.dynamics_thrust + power.steady_state_thrust;

	power.total_thrust_power = std::max(
			power.steady_state_thrust_power + power.dynamics_thrust_power, 0.0);

	if (!lastSensorDataStamp_.isZero())
	{
		timeDiff_ = sd.header.stamp - lastSensorDataStamp_;
		cumulativeTotalThrustPower_ += power.total_thrust_power * timeDiff_.toNSec() / 1e9;
	}

	lastSensorDataStamp_ = sd.header.stamp;

	power.cumulative_total_thrust_power = cumulativeTotalThrustPower_;

	return power;
}

bool
PowerModel::configure()
{
	ros::NodeHandle nh;
	bool success = true;
	success &= nh.getParam("/power_modeling_node/l_over_d", loverd_);
	success &= nh.getParam("/power_modeling_node/mass", mass_);
	nh.getParam("/power_model/gravity_constant", gravityConstant_);

	if (!success)
		return false;

	thrustPowerPublisher_ = nh.advertise<power_modeling::power_info>("power_model/thrust_power",
			20);
	predictedPowerPublisher_ = nh.advertise<power_modeling::power_info>(
			"power_model/predicted_power", 20);
	sensorDataSubscriber_ = nh.subscribe("sensor_data", 20, &PowerModel::onSensorData, this);
	trajectorySubscriber_ = nh.subscribe("radio_comm/trajectory", 20, &PowerModel::onTrajectory,
			this);
	localPlannerStatusSubscriber_ = nh.subscribe("radio_comm/local_planner_status", 20,
			&PowerModel::onLocalPlannerStatus, this);

	resetPowerService_ = nh.advertiseService("power_model/reset_power",
			&PowerModel::resetPowerService, this);
	return true;
}

power_modeling::power_info
PowerModel::predictAerodynamics()
{
	auto section = *currentPathSection_;

	power_modeling::power_info power;

	if (auto spline = std::dynamic_pointer_cast<CubicSpline>(section))
	{
		double u = spline->closestU_;

		Vector3 p_prime = spline->c1_ + 2 * spline->c2_ * u + 3 * spline->c3_ * pow(u, 2);
		Vector3 p_p_prime = 2 * spline->c2_ + 6 * spline->c3_ * u;

		double psi_prime = (p_p_prime[1] * p_prime[0] - p_p_prime[0] * p_prime[1])
				/ (pow(p_prime[0], 2) + pow(p_prime[1], 2));

		double part1 = mass_ * gravityConstant_ / (loverd_ * p_prime.norm());
		double part2 = sqrt(
				1
						+ pow(pow(spline->getVelocity(), 2) * psi_prime / gravityConstant_, 2)
								/ p_prime.squaredNorm());
		double part3 = p_prime.topRows(2).norm() + loverd_ * p_prime[2];

		power.steady_state_thrust = part1 * part2 * part3;

		power.steady_state_thrust_power = power.steady_state_thrust * spline->getVelocity();

//		power.dynamics_thrust_power = p_prime.dot(p_p_prime)
//				* pow(spline->getVelocity() / p_prime.norm(), 3) * mass_;
//
//		power.dynamics_thrust = power.dynamics_thrust_power / spline->velocity_;

		power.total_thrust_power = std::max(power.steady_state_thrust_power, 0.0);

		if (!timeDiff_.isZero())
		{
			cumulativePredictedTotalThrustPower_ += power.total_thrust_power * timeDiff_.toNSec()
					/ 1e9;
		}

		power.cumulative_total_thrust_power = cumulativePredictedTotalThrustPower_;
	}

	return power;
}

void
PowerModel::onSensorData(const simulation_interface::sensor_data& data)
{
	auto power = calculateAerodynamics(data);
	thrustPowerPublisher_.publish(power);

	if (currentPathSection_ == trajectory_.pathSections.end() || !(*currentPathSection_))
	{
		//APLOG_DEBUG << "Trajectory needed for power prediction.";
		return;
	}

	(*currentPathSection_)->updatePosition(xyzTypeToVector3(data.position));
	auto predicted = predictAerodynamics();

	predictedPowerPublisher_.publish(predicted);
}

void
PowerModel::onTrajectory(const radio_comm::serialized_object& data)
{
	Content content = Content::INVALID;
	Packet packet(data.serialized);
	boost::any any = dataPresentation_.deserialize(packet, content);
	if (content != Content::TRAJECTORY)
	{
		APLOG_ERROR << "Trajectory received cannot be deserialized.";
		return;
	}

	std::lock_guard<std::mutex> lock(trajectoryMutex_);
	try
	{
		trajectory_ = boost::any_cast<Trajectory>(any);
	} catch (boost::bad_any_cast& err)
	{
		APLOG_ERROR << "Bad any cast for trajectory." << err.what();
		return;
	}
	currentPathSection_ = trajectory_.pathSections.end();
}

void
PowerModel::onLocalPlannerStatus(const radio_comm::local_planner_status& data)
{
	std::lock_guard<std::mutex> lock(trajectoryMutex_);
	if (data.linear_status.current_path_section >= trajectory_.pathSections.size())
	{
		//APLOG_ERROR << "Current path section not on trajectory. Maybe request new trajectory.";
		return;
	}
	currentPathSection_ = trajectory_.pathSections.begin()
			+ data.linear_status.current_path_section;
}

bool
PowerModel::resetPowerService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
	cumulativeTotalThrustPower_ = 0;
	cumulativePredictedTotalThrustPower_ = 0;
	resp.success = true;
	return true;
}
