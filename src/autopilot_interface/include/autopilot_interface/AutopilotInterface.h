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
 * AutopilotInterface.h
 *
 *  Created on: Nov 25, 2017
 *      Author: mircot
 */

#ifndef AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_H_
#define AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_H_
#include <boost/property_tree/ptree.hpp>
#include <uavAP/Core/IPC/Publisher.h>
#include <uavAP/Core/IPC/Subscription.h>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Runner/IRunnableObject.h>

#include <ros/ros.h>
#include <simulation_interface/actuation.h>
#include <simulation_interface/sensor_data.h>

#include <uavAP/API/ap_ext/ap_ext.h>
#include <uavAP/API/ap_ext/ServoMapping.h>
#include <uavAP/API/ChannelMixing.h>
#include <uavAP/Core/IDC/Sender.h>

#include <power_modeling/power_info.h>

class AutopilotInterface
{
public:

	AutopilotInterface();

	~AutopilotInterface();

	bool
	startApExt();

	void
	setDeviceBridge(const std::string& serialPort);

	bool
	configure(const boost::property_tree::ptree& config);

private:

	void
	onSensorData(const simulation_interface::sensor_data& sensorData);

	void
	onThrustPower(const power_modeling::power_info&);

	void
	sendActuation();

	simulation_interface::actuation
	reverseChannelMixing(unsigned long* pwm);

	void
	onPacket(const Packet& packet);

	bool setup_;

	bool deviceBridge_;
	std::string serialPort_;
	Aggregator aggregator_;
	Sender sensorDataSender_;

	data_sample_t dataSample_;

	ros::Publisher actuationPublisherRos_;
	ros::Subscriber sensorDataSubscriptionRos_;
	ros::Subscriber powerModelSubscriptionRos_;

	ChannelMixing channelMixing_;
	ServoMapping servoMapping_;

	const int numChannels_;

	uint32_t lastSequenceNr_;

};

#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_H_ */
