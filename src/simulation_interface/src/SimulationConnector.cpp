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
 * SimulationConnector.cpp
 *
 *  Created on: Jul 28, 2017
 *      Author: mircot
 */
#include <simulation_interface/codec/Codec.h>
#include <simulation_interface/SimulationConnector.h>
#include <uavAP/Core/DataPresentation/IDataPresentation.h>
#include <uavAP/Core/IDC/Serial/SerialIDC.h>
#include <uavAP/Core/IDC/Serial/SerialIDCParams.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include <uavAP/Core/TimeProvider/ITimeProvider.h>
#include <uavAP/API/ChannelMixing.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>
#include <uavAP/Core/LinearAlgebra.h>

#include <std_msgs/Int32.h>

#include "simulation_interface/sensor_data.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

SimulationConnector::SimulationConnector() :
		serialPort_("/dev/ttyUSB0"), lastSequenceNr_(0), correctSimDelay_(true), correctionCounter_(
				0)
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
		if (!nodeHandle_)
		{
			APLOG_ERROR << "Node Handle not set. ";
			return true;
		}
		sensorPublisher_ = nodeHandle_->advertise<simulation_interface::sensor_data>("sensor_data",
				20);
		delayPublisher_ = nodeHandle_->advertise<std_msgs::Int32>("roundtrip_delay_micros", 20);
		actuationSubscriber_ = nodeHandle_->subscribe("actuation", 20,
				&SimulationConnector::actuate, this);
		break;
	}
	case RunStage::NORMAL:
	{
		auto idc = idc_.get();
		SerialIDCParams params(serialPort_, 115200, "\0");
		idc->subscribeOnPacket(params,
				std::bind(&SimulationConnector::sense, this, std::placeholders::_1));
		SerialIDCParams actuationParams(serialPort_, 115200, "\n");
		actuationSender_ = idc->createSender(actuationParams);
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
SimulationConnector::notifyAggregationOnUpdate(Aggregator& agg)
{
	scheduler_.setFromAggregationIfNotSet(agg);
	timeProvider_.setFromAggregationIfNotSet(agg);
	idc_.setFromAggregationIfNotSet(agg);
	dataPresentation_.setFromAggregationIfNotSet(agg);
	channelMixing_.setFromAggregationIfNotSet(agg);
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
	auto channelMix = channelMixing_.get();
	if (!channelMix)
	{
		APLOG_ERROR << "Channel Mixing is missing. Cannot send actuation command.";
		return;
	}

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
		auto diff = boost::get_system_time() - it->second;
		std_msgs::Int32 delay;
		delay.data = diff.total_microseconds();
		delayPublisher_.publish(delay);
		timepoints_.erase(timepoints_.cbegin(), --it);
	}
	lock.unlock();

	ControllerOutput control;
	control.collectiveOutput = out.collectiveOutput;
	control.flapOutput = out.flapOutput;
	control.pitchOutput = out.pitchOutput;
	control.rollOutput = out.rollOutput;
	control.throttleOutput = out.throttleOutput;
	control.yawOutput = out.yawOutput;

	auto channels = channelMix->mixChannels(control);

	std::stringstream ss;
	for (auto it : channels)
	{
		ss << it << ",";
	}

	ss << "1\r";

	actuationSender_.sendPacket(Packet(ss.str()));
}

bool
SimulationConnector::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);

	pm.add("serial_port", serialPort_, true);
    pm.add("correct_sim_delay", correctSimDelay_, false);

	return pm.map();
}

std::shared_ptr<SimulationConnector>
SimulationConnector::create(const boost::property_tree::ptree& config)
{
	auto sim = std::make_shared<SimulationConnector>();
	sim->configure(config);
	return sim;
}

void
SimulationConnector::setNodeHandle(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
	nodeHandle_ = nodeHandle;
}

void
SimulationConnector::sendSensorData(simulation_interface::sensor_data data)
{
	TimePoint timepoint = boost::get_system_time();

	data.header.stamp = ros::Time::fromBoost(timepoint);
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
		correctionInit_ = boost::get_system_time();
		sendSensorData(data);
		return;
	}

	TimePoint now = boost::get_system_time();
	Duration timediff = (correctionInit_ + Milliseconds(10) * correctionCounter_)
			- now;

	if (timediff.is_negative())
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

