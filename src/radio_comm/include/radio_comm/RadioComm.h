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
 * RadioComm.h
 *
 *  Created on: Dec 9, 2017
 *      Author: mircot
 */

#ifndef RADIO_COMM_INCLUDE_RADIO_COMM_RADIOCOMM_H_
#define RADIO_COMM_INCLUDE_RADIO_COMM_RADIOCOMM_H_
#include <boost/property_tree/ptree.hpp>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Runner/IRunnableObject.h>

#include <ros/ros.h>
#include <uavAP/Core/IDC/Sender.h>

#include "radio_comm/select_mission.h"
#include "radio_comm/request_data.h"
#include "radio_comm/tune_pid.h"
#include "radio_comm/send_control_override.h"

class IInterDeviceComm;
class IScheduler;
class Packet;
class ITimeProvider;

enum class Content
;
enum class Target
;
template<typename C, typename T>
class IDataPresentation;

class RadioComm: public IAggregatableObject, public IRunnableObject
{
public:

	RadioComm() = default;

	static std::shared_ptr<RadioComm>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	void
	notifyAggregationOnUpdate(Aggregator& agg) override;

	bool
	run(RunStage stage) override;

private:

	void
	onAutopilotPacket(const Packet& packet);

	/*
	 * Services
	 */
	bool
	selectMission(radio_comm::select_mission::Request& req,
			radio_comm::select_mission::Response& resp);

	bool
	requestData(radio_comm::request_data::Request& req, radio_comm::request_data::Response& resp);

	bool
	tunePID(radio_comm::tune_pid::Request& req, radio_comm::tune_pid::Response& resp);

	bool
	sendControlOverride(radio_comm::send_control_override::Request& req, radio_comm::send_control_override::Response& resp);

	bool
	sendPacket(const Packet& packet);

	ObjectHandle<IInterDeviceComm> idc_;
//	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<IDataPresentation<Content, Target>> dataPresentation_;
//	ObjectHandle<ITimeProvider> timeProvider_;

	std::string radioSerialPort_;

	Sender radioSender_;

	ros::Publisher sensorDataPublisher_;
	ros::Publisher pidStatiPublisher_;
	ros::Publisher trajectoryPublisher_;
	ros::Publisher missionPublisher_;
	ros::Publisher localPlannerStatusPublisher_;

	ros::ServiceServer selectMissionService_;
	ros::ServiceServer requestDataService_;
	ros::ServiceServer tunePIDService_;
	ros::ServiceServer sendControlOverrideService_;
};

#endif /* RADIO_COMM_INCLUDE_RADIO_COMM_RADIOCOMM_H_ */
