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
 * SimulationConnector.cpp
 *
 *  Created on: Jul 28, 2017
 *      Author: mircot
 */
#include <simulation_interface/codec/Codec.h>
#include <simulation_interface/SimulationConnector.h>
#include <uavAP/Core/DataPresentation/DataPresentation.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include <uavAP/Core/TimeProvider/ITimeProvider.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/IDC/IDC.h>
#include <uavAP/Core/IDC/IDCSender.h>
#include <uavAP/Core/Scheduler/IScheduler.h>

#include <std_msgs/Int32.h>

#include "simulation_interface/sensor_data.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

SimulationConnector::SimulationConnector() :
		lastSequenceNr_(0), correctSimDelay_(false), correctionCounter_(0)
{
}

SimulationConnector::~SimulationConnector()
{
}

bool
SimulationConnector::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!idc_.isSet())
		{
			APLOG_ERROR << "SimulationConnector: SerialIDC missing.";
			return true;
		}
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "SimulationConnector: Scheduler missing.";
			return true;
		}
		ros::NodeHandle nh;
		sensorPublisher_ = nh.advertise<simulation_interface::sensor_data>("sensor_data", 20);
		delayPublisher_ = nh.advertise<std_msgs::Int32>("roundtrip_delay_micros", 20);
		actuationSubscriber_ = nh.subscribe("actuation", 20, &SimulationConnector::actuate, this);
		break;
	}
	case RunStage::NORMAL:
	{
		auto idc = idc_.get();
		sensorReceiver_ = idc->subscribeOnPacket("sim_sense",
				std::bind(&SimulationConnector::sense, this, std::placeholders::_1));
		actuationSender_ = idc->createSender("sim_act");
		break;
	}
	case RunStage::FINAL:
	{
		break;
	}
	default:
		break;
	}
	return false;
}

void
SimulationConnector::notifyAggregationOnUpdate(const Aggregator& agg)
{
	scheduler_.setFromAggregationIfNotSet(agg);
	timeProvider_.setFromAggregationIfNotSet(agg);
	idc_.setFromAggregationIfNotSet(agg);
	dataPresentation_.setFromAggregationIfNotSet(agg);
}

void
SimulationConnector::sense(const Packet& packet)
{
	std::string decoded = decode(packet.getBuffer());

	simulation_interface::sensor_data sd;
	Vector3 velocityLinear;

	bool success = false;
	int parsed = sscanf(decoded.c_str(),
			"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
			&sd.acceleration.linear.x, &sd.acceleration.linear.y, &sd.acceleration.linear.z,
			&sd.velocity.angular.x, &sd.velocity.angular.y, &sd.velocity.angular.z, &sd.attitude.x,
			&sd.attitude.y, &sd.attitude.z, &sd.position.y, &sd.position.x, &sd.position.z,
			&velocityLinear[0], &velocityLinear[1], &velocityLinear[2]);
	if (parsed != 15)
	{
		APLOG_WARN << "Invalid SensorData from Simulator.";
		return;
	}
	sd.position.z *= -1; //ENU

	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(sd.attitude.z, Vector3::UnitZ());

	velocityLinear = m * velocityLinear;

	//Convert to ENU
	sd.velocity.linear.x = velocityLinear.y();
	sd.velocity.linear.y = velocityLinear.x();
	sd.velocity.linear.z = -1 * velocityLinear.z();
	sd.attitude.z = boundAngleRad(-(sd.attitude.z - M_PI / 2));

	//Convert p-q-r to rates
	double sRoll = sin(sd.attitude.x);
	double cRoll = cos(sd.attitude.x);
	double sPitch = sin(sd.attitude.y);
	double cPitch = cos(sd.attitude.y);
	Eigen::Matrix3d rotL1B;
	rotL1B << 1, 0, -sPitch, 0, cRoll, sRoll * cPitch, 0, -sRoll, cRoll * cPitch;

	Vector3 rates(sd.velocity.angular.x, sd.velocity.angular.y, sd.velocity.angular.z);

	rates = rotL1B.inverse() * rates;

	sd.velocity.angular.x = rates.x();
	sd.velocity.angular.y = rates.y();
	sd.velocity.angular.z = rates.z();

	sd.ground_speed = velocityLinear.norm();
	sd.air_speed = sd.ground_speed;

	lastSequenceNr_++;
	sd.sequenceNr = lastSequenceNr_;

	if (correctSimDelay_)
	{
		handleSimDelay(sd);
	}
	else
	{
		sendSensorData(sd);
	}
}

void
SimulationConnector::actuate(const simulation_interface::actuation& out)
{
	std::unique_lock<std::mutex> lock(timePointsMutex_);
	auto it = timepoints_.find(out.sequenceNr);
	if (it == timepoints_.end())
	{
		if (!timepoints_.empty())
		{
			it--;
			APLOG_DEBUG << "Invalid sequence number: " << out.sequenceNr << ". Stored seqNr: "
					<< timepoints_.upper_bound(0)->first << " - "
					<< timepoints_.lower_bound(UINT32_MAX)->first;
		}
		else
			APLOG_DEBUG << "Invalid sequence number";
	}
	else
	{
		auto diff = Clock::now() - it->second;
		std_msgs::Int32 delay;
		delay.data = std::chrono::duration_cast<Microseconds>(diff).count();
		delayPublisher_.publish(delay);
		timepoints_.erase(timepoints_.cbegin(), --it);
	}
	lock.unlock();

	ControllerOutput c;
	c.pitchOutput = out.pitchOutput;
	c.rollOutput = out.rollOutput;
	c.throttleOutput = out.throttleOutput;
	c.yawOutput = out.yawOutput;

	std::stringstream ss;

	ss << c.throttleOutput << "," << c.yawOutput << "," << c.pitchOutput << "," << c.rollOutput
			<< "," << -c.rollOutput << "," << -1.0 << ",";

	ss << "1";

	actuationSender_.sendPacket(Packet(ss.str()));
}

bool
SimulationConnector::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);

	pm.add<bool>("correct_sim_delay", correctSimDelay_, false);

	return pm.map();
}

std::shared_ptr<SimulationConnector>
SimulationConnector::create(const Configuration& config)
{
	auto sim = std::make_shared<SimulationConnector>();
	sim->configure(config);
	return sim;
}

void
SimulationConnector::sendSensorData(simulation_interface::sensor_data data)
{
	TimePoint timepoint = Clock::now();

	data.header.stamp = ros::Time::now();
	std::unique_lock<std::mutex> lock(timePointsMutex_);

	timepoints_.insert(std::make_pair(data.sequenceNr, timepoint));
	while (timepoints_.size() > 100)
		timepoints_.erase(timepoints_.begin());

	lock.unlock();

	sensorPublisher_.publish(data);
}

void
SimulationConnector::handleSimDelay(const simulation_interface::sensor_data& data)
{
	auto sched = scheduler_.get();
	if (!sched)
	{
		APLOG_ERROR << "scheduler missing.";
		sendSensorData(data);
		return;
	}

	if (correctionCounter_ == 0)
	{
		++correctionCounter_;
		correctionInit_ = Clock::now();
		sendSensorData(data);
		return;
	}

	TimePoint now = Clock::now();
	Duration timediff = (correctionInit_ + Milliseconds(10) * correctionCounter_) - now;

	if (timediff.count() < 0)
	{
		sendSensorData(data);
		correctionCounter_ = 0;
		correctionInit_ = now;
	}
	else
	{
		sched->schedule(std::bind(&SimulationConnector::sendSensorData, this, data), timediff);
	}

	++correctionCounter_;
}
