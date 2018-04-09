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
 * RadioComm.cpp
 *
 *  Created on: Dec 10, 2017
 *      Author: mircot
 */
#include "radio_comm/RadioComm.h"
#include "radio_comm/pidstati.h"
#include "radio_comm/serialized_object.h"
#include "radio_comm/local_planner_status.h"

#include <autopilot_interface/uavAPConversions.h>
#include <simulation_interface/sensor_data.h>
#include <uavAP/Core/IDC/IInterDeviceComm.h>
#include <uavAP/Core/IDC/Serial/SerialIDCParams.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include <uavAP/Core/DataPresentation/Content.h>
#include <uavAP/FlightControl/Controller/PIDController/detail/PIDHandling.h>
#include <uavAP/MissionControl/MissionPlanner/ControlOverride.h>

std::shared_ptr<RadioComm>
RadioComm::create(const boost::property_tree::ptree& config)
{
	auto comm = std::make_shared<RadioComm>();
	if (!comm->configure(config))
		APLOG_ERROR << "RadioComm configuration failed";
	return comm;
}

bool
RadioComm::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	pm.add("serial_port", radioSerialPort_, true);

	return pm.map();
}

void
RadioComm::notifyAggregationOnUpdate(Aggregator& agg)
{
	idc_.setFromAggregationIfNotSet(agg);
//	scheduler_.setFromAggregationIfNotSet(agg);
	dataPresentation_.setFromAggregationIfNotSet(agg);
}

bool
RadioComm::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!idc_.isSet())
		{
			APLOG_ERROR << "RadioComm: InterDeviceCommunication missing";
			return true;
		}
//		if (!scheduler_.isSet())
//		{
//			APLOG_ERROR << "RadioComm: Scheduler missing";
//			return true;
//		}
		if (!dataPresentation_.isSet())
		{
			APLOG_ERROR << "RadioComm: DataPresentation missing";
			return true;
		}
//		if (!timeProvider_.isSet())
//		{
//			APLOG_ERROR << "RadioComm: TimeProvider missing";
//			return true;
//		}

		ros::NodeHandle nh;
		sensorDataPublisher_ = nh.advertise<simulation_interface::sensor_data>(
				"/radio_comm/sensor_data", 20);
		pidStatiPublisher_ = nh.advertise<radio_comm::pidstati>("/radio_comm/pid_stati", 20);
		trajectoryPublisher_ = nh.advertise<radio_comm::serialized_object>("/radio_comm/trajectory", 20);
		missionPublisher_ = nh.advertise<radio_comm::serialized_object>("/radio_comm/mission", 20);
		localPlannerStatusPublisher_ = nh.advertise<radio_comm::local_planner_status>(
				"/radio_comm/local_planner_status", 20);

		selectMissionService_ = nh.advertiseService("/radio_comm/select_mission",
				&RadioComm::selectMission, this);
		requestDataService_ = nh.advertiseService("/radio_comm/request_data",
				&RadioComm::requestData, this);
		tunePIDService_ = nh.advertiseService("/radio_comm/tune_pid",
				&RadioComm::tunePID, this);
		sendControlOverrideService_ = nh.advertiseService("/radio_comm/send_control_override", &RadioComm::sendControlOverride, this);
		break;
	}
	case RunStage::NORMAL:
	{
		auto idc = idc_.get();
		if (!idc)
		{
			APLOG_ERROR << "Radio Comm IDC missing.";
			return true;
		}
		SerialIDCParams params(radioSerialPort_, 115200, "*-*\n");
		idc->subscribeOnPacket(params,
				std::bind(&RadioComm::onAutopilotPacket, this, std::placeholders::_1));

		radioSender_ = idc->createSender(params);
		break;
	}
	case RunStage::FINAL:
		break;
	default:
		break;
	}
	return false;
}

void
RadioComm::onAutopilotPacket(const Packet& packet)
{
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "RadioComm Data Presentation missing. Cannot deserialize packet.";
		return;
	}

	std::string packetString = packet.getBuffer();

	Content content(Content::INVALID);
	boost::any any = dp->deserialize(packet, content);

	try
	{
		switch (content)
		{
		case Content::SENSOR_DATA:
			sensorDataPublisher_.publish(apToRos(boost::any_cast<SensorData>(any)));
			break;
		case Content::PID_STATUS:
			pidStatiPublisher_.publish(apToRos(boost::any_cast<PIDStati>(any)));
			break;
		case Content::LOCAL_PLANNER_STATUS:
			localPlannerStatusPublisher_.publish(apToRos(boost::any_cast<LocalPlannerStatus>(any)));
			break;
		case Content::TRAJECTORY:
		{
			radio_comm::serialized_object traj;
			traj.serialized = packetString;
			trajectoryPublisher_.publish(traj);
			break;
		}
		case Content::MISSION:
		{
			radio_comm::serialized_object mission;
			mission.serialized = packetString;
			missionPublisher_.publish(mission);
			break;
		}
		default:
			APLOG_ERROR << "Unknown Content " << (int) content;
			break;
		}
	} catch (boost::bad_any_cast& err)
	{
		APLOG_ERROR << "Bad any cast for Content " << (int) content << ": " << any.empty();
	}

}

bool
RadioComm::selectMission(radio_comm::select_mission::Request& req,
		radio_comm::select_mission::Response& resp)
{
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "DataPresentation missing.";
		return false;
	}
	auto packet = dp->serialize(req.mission, Content::SELECT_MISSION);
	dp->setTarget(packet, Target::MISSION_CONTROL);
	return sendPacket(packet);
}

bool
RadioComm::requestData(radio_comm::request_data::Request& req,
		radio_comm::request_data::Response& resp)
{
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "DataPresentation missing.";
		return false;
	}

	Target target = Target::INVALID;
	switch (static_cast<DataRequest>(req.data_request_id))
	{
	case DataRequest::MISSION:
		target = Target::MISSION_CONTROL;
		break;
	case DataRequest::SAFETY_BOUNDS:
		target = Target::MISSION_CONTROL;
		break;
	case DataRequest::TRAJECTORY:
		target = Target::FLIGHT_CONTROL;
		break;
	default:
		break;
	}

	if (target != Target::INVALID)
	{
		resp.valid_request = true;
		auto packet = dp->serialize(static_cast<DataRequest>(req.data_request_id),
				Content::REQUEST_DATA);
		dp->setTarget(packet, target);
		return sendPacket(packet);
	}

	resp.valid_request = false;
	return true;
}

bool
RadioComm::tunePID(radio_comm::tune_pid::Request& req, radio_comm::tune_pid::Response& resp)
{
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "DataPresentation missing.";
		return false;
	}
	PIDTuning tuning;
	tuning.pid = req.id;
	tuning.params.kp = req.kp;
	tuning.params.ki = req.ki;
	tuning.params.kd = req.kd;
	tuning.params.ff = req.ff;
	tuning.params.imax = req.imax;

	resp.valid_request = true;

	auto packet = dp->serialize(tuning, Content::TUNE_PID);
	dp->setTarget(packet, Target::FLIGHT_CONTROL);
	return sendPacket(packet);
}

bool
RadioComm::sendControlOverride(radio_comm::send_control_override::Request& req, radio_comm::send_control_override::Response& resp)
{
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "DataPresentation missing.";
		return false;
	}
	ControlOverride co;
	co.overrideManeuverPlanner = req.overridemaneuverplanner;
	co.activation.activate = req.activate;
	co.activation.overrideClimbRateTarget = req.climbratetargetoverride;
	co.activation.overrideFlapOutput = req.flapoutputoverride;
	co.activation.overridePitchOutput = req.pitchoutputoverride;
	co.activation.overridePitchTarget = req.pitchtargetoverride;
	co.activation.overrideRollOutput = req.rolloutputoverride;
	co.activation.overrideRollTarget = req.rolltargetoverride;
	co.activation.overrideThrottleOutput = req.throttleoutputoverride;
	co.activation.overrideVelocityTarget = req.velocitytargetoverride;
	co.activation.overrideYawOutput = req.yawoutputoverride;
	co.activation.overrideYawRateTarget = req.yawratetargetoverride;
	co.activation.overrideClimbRateTarget = req.climbratetargetoverride;
	co.activation.overrideFlapOutput = req.flapoutputoverride;
	co.activation.overridePitchOutput = req.pitchoutputoverride;
	co.activation.overridePitchTarget = req.pitchtargetoverride;
	co.activation.overrideRollOutput = req.rolloutputoverride;
	co.activation.overrideRollTarget = req.rolltargetoverride;
	co.activation.overrideThrottleOutput = req.throttleoutputoverride;
	co.activation.overrideVelocityTarget = req.velocitytargetoverride;
	co.activation.overrideYawOutput = req.yawoutputoverride;
	co.activation.overrideYawRateTarget = req.yawratetargetoverride;
	co.target.climbRateTarget = req.climbratetargetvalue;
	co.target.flapOutput = req.flapoutputvalue;
	co.target.pitchOutput = req.pitchoutputvalue;
	co.target.pitchTarget = req.pitchtargetvalue;
	co.target.rollOutput = req.rolloutputvalue;
	co.target.rollTarget = req.rolltargetvalue;
	co.target.throttleOutput = req.throttleoutputvalue;
	co.target.velocityTarget = req.velocitytargetvalue;
	co.target.yawOutput = req.yawoutputvalue;
	co.target.yawRateTarget = req.yawratetargetvalue;
	co.target.climbRateTarget = req.climbratetargetvalue;
	co.target.flapOutput = req.flapoutputvalue;
	co.target.pitchOutput = req.pitchoutputvalue;
	co.target.pitchTarget = req.pitchtargetvalue;
	co.target.rollOutput = req.rolloutputvalue;
	co.target.rollTarget = req.rolltargetvalue;
	co.target.throttleOutput = req.throttleoutputvalue;
	co.target.velocityTarget = req.velocitytargetvalue;
	co.target.yawOutput = req.yawoutputvalue;
	co.target.yawRateTarget = req.yawratetargetvalue;
	auto packet = dp->serialize(co, Content::OVERRIDE_CONTROL);
	dp->setTarget(packet, Target::MISSION_CONTROL);

	resp.valid_request = true;

	return sendPacket(packet);
}

bool
RadioComm::sendPacket(const Packet& packet)
{
	return radioSender_.sendPacket(packet);
}
