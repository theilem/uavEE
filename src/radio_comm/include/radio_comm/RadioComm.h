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
#include <uavAP/Core/IDC/IDCSender.h>
#include <ros/ros.h>
#include <uavAP/Core/IPC/Subscription.h>

#include "radio_comm/select_mission.h"
#include "radio_comm/select_maneuver.h"
#include "radio_comm/request_data.h"
#include "radio_comm/tune_generic.h"
#include "radio_comm/tune_pid.h"
#include "radio_comm/serialized_service.h"
#include "radio_comm/send_advanced_control.h"

class IDC;
class IPC;
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

	static constexpr TypeId typeId = "radio_comm";

	RadioComm() = default;

	static std::shared_ptr<RadioComm>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

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
	selectManeuver(radio_comm::select_maneuver::Request& req,
			radio_comm::select_maneuver::Response& resp);

	bool
	selectInspectingMetrics(radio_comm::serialized_service::Request& req,
			radio_comm::serialized_service::Response& resp);

	bool
	requestData(radio_comm::request_data::Request& req, radio_comm::request_data::Response& resp);

	bool
	tunePID(radio_comm::tune_pid::Request& req, radio_comm::tune_pid::Response& resp);

	bool
	tuneGeneric(radio_comm::tune_generic::Request& req, radio_comm::tune_generic::Response& resp);

	bool
	sendOverride(radio_comm::serialized_service::Request& req,
			radio_comm::serialized_service::Response& resp);

	bool
	sendAdvancedControl(radio_comm::send_advanced_control::Request& req,
			radio_comm::send_advanced_control::Response& resp);

	bool
	sendLocalFrame(radio_comm::serialized_service::Request& req,
			radio_comm::serialized_service::Response& resp);

	bool
	sendPacket(const Packet& packet);

	ObjectHandle<IDC> idc_;
	ObjectHandle<IDataPresentation<Content, Target>> dataPresentation_;
	ObjectHandle<IPC> ipc_;

	Subscription groundStationSubscription_;
	IDCSender radioSender_;
	boost::signals2::connection radioReceiver_;

	ros::Publisher sensorDataPublisher_;
	ros::Publisher pidStatiPublisher_;
	ros::Publisher inspectingMetricsPublisher_;
	ros::Publisher trajectoryPublisher_;
	ros::Publisher missionPublisher_;
	ros::Publisher overridePublisher_;
	ros::Publisher localFramePublisher_;
	ros::Publisher safetyBoundsPublisher_;
	ros::Publisher localPlannerStatusPublisher_;

	ros::ServiceServer selectMissionService_;
	ros::ServiceServer selectManeuverService_;
	ros::ServiceServer selectInspectingMetricsService_;
	ros::ServiceServer requestDataService_;
	ros::ServiceServer genericTuningService_;
	ros::ServiceServer tunePIDService_;
	ros::ServiceServer sendOverrideService_;
	ros::ServiceServer sendAdvancedControlService_;
	ros::ServiceServer sendLocalFrameService_;
};

#endif /* RADIO_COMM_INCLUDE_RADIO_COMM_RADIOCOMM_H_ */
