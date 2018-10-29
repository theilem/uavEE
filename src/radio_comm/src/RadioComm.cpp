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
 * RadioComm.cpp
 *
 *  Created on: Dec 10, 2017
 *      Author: mircot
 */

#include <std_msgs/String.h>
#include <uavAP/Core/IDC/IDC.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include <uavAP/Core/DataPresentation/ContentMapping.h>
#include <uavAP/Core/DataPresentation/IDataPresentation.h>
#include <uavAP/Core/Frames/VehicleOneFrame.h>
#include <uavAP/FlightAnalysis/StateAnalysis/Metrics.h>
#include <uavAP/FlightAnalysis/StateAnalysis/SteadyStateAnalysis.h>
#include <uavAP/FlightControl/Controller/AdvancedControl.h>
#include <uavAP/FlightControl/Controller/PIDController/PIDHandling.h>
#include <uavAP/MissionControl/ManeuverPlanner/Override.h>
#include <uavAP/Core/DataPresentation/BinarySerialization.hpp>
#include <autopilot_interface/detail/uavAPConversions.h>
#include <simulation_interface/sensor_data.h>

#include "radio_comm/serialized_proto.h"
#include "radio_comm/RadioComm.h"
#include "radio_comm/pidstati.h"
#include "radio_comm/serialized_object.h"

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
	return pm.map();
}

void
RadioComm::notifyAggregationOnUpdate(const Aggregator& agg)
{
	idc_.setFromAggregationIfNotSet(agg);
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

		if (!dataPresentation_.isSet())
		{
			APLOG_ERROR << "RadioComm: DataPresentation missing";
			return true;
		}

		ros::NodeHandle nh;
		sensorDataPublisher_ = nh.advertise<simulation_interface::sensor_data>(
				"/radio_comm/sensor_data", 20);
		pidStatiPublisher_ = nh.advertise<radio_comm::pidstati>("/radio_comm/pid_stati", 20);
		inspectingMetricsPublisher_ = nh.advertise<radio_comm::serialized_object>(
				"/radio_comm/inspecting_metrics", 20);
		trajectoryPublisher_ = nh.advertise<radio_comm::serialized_object>("/radio_comm/trajectory",
				20);
		missionPublisher_ = nh.advertise<radio_comm::serialized_object>("/radio_comm/mission", 20);
		overridePublisher_ = nh.advertise<radio_comm::serialized_object>("/radio_comm/override", 20);
		localFramePublisher_ = nh.advertise<radio_comm::serialized_object>(
				"/radio_comm/local_frame", 20);
		localPlannerStatusPublisher_ = nh.advertise<radio_comm::serialized_proto>(
				"/radio_comm/local_planner_status", 20);
		safetyBoundsPublisher_ = nh.advertise<std_msgs::String>("/radio_comm/safety_bounds", 20);

		selectMissionService_ = nh.advertiseService("/radio_comm/select_mission",
				&RadioComm::selectMission, this);
		selectManeuverService_ = nh.advertiseService("/radio_comm/select_maneuver",
				&RadioComm::selectManeuver, this);
		selectInspectingMetricsService_ = nh.advertiseService(
				"/radio_comm/select_inspecting_metrics", &RadioComm::selectInspectingMetrics, this);
		requestDataService_ = nh.advertiseService("/radio_comm/request_data",
				&RadioComm::requestData, this);
		genericTuningService_ = nh.advertiseService("/radio_comm/tune_generic",
				&RadioComm::tuneGeneric, this);
		tunePIDService_ = nh.advertiseService("/radio_comm/tune_pid", &RadioComm::tunePID, this);
		sendOverrideService_ = nh.advertiseService("/radio_comm/send_override",
				&RadioComm::sendOverride, this);
		sendAdvancedControlService_ = nh.advertiseService("/radio_comm/send_advanced_control",
				&RadioComm::sendAdvancedControl, this);
		sendLocalFrameService_ = nh.advertiseService("/radio_comm/send_local_frame",
						&RadioComm::sendLocalFrame, this);

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
		radioReceiver_ = idc->subscribeOnPacket("autopilot",
				std::bind(&RadioComm::onAutopilotPacket, this, std::placeholders::_1));

		radioSender_ = idc->createSender("autopilot");
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

	boost::any any;
	try
	{
		any = dp->deserialize(packet, content);
	} catch (std::runtime_error& err)
	{
		APLOG_ERROR << "runtime error: " << err.what();
		return;
	}

	try
	{
		switch (content)
		{
		case Content::SENSOR_DATA:
		{
			sensorDataPublisher_.publish(apToRos(boost::any_cast<SensorData>(any)));
			break;
		}
		case Content::PID_STATUS:
			pidStatiPublisher_.publish(apToRos(boost::any_cast<PIDStati>(any)));
			break;
		case Content::INSPECTING_METRICS:
		{
			radio_comm::serialized_object inspectingMetrics;
			inspectingMetrics.serialized =
					dp::serialize(boost::any_cast<SteadyStateMetrics>(any)).getBuffer();
			inspectingMetricsPublisher_.publish(inspectingMetrics);
			break;
		}
		case Content::LOCAL_PLANNER_STATUS:
		{
			radio_comm::serialized_proto msg;
			msg.proto_message = boost::any_cast<LocalPlannerStatus>(any).SerializeAsString();
			localPlannerStatusPublisher_.publish(msg);
			break;
		}
		case Content::SAFETY_BOUNDS:
		{
			std_msgs::String msg;
			msg.data = boost::any_cast<Rectanguloid>(any).SerializeAsString();
			safetyBoundsPublisher_.publish(msg);
			break;
		}
		case Content::TRAJECTORY:
		{
			radio_comm::serialized_object traj;
			traj.serialized = dp::serialize(boost::any_cast<Trajectory>(any)).getBuffer();
			trajectoryPublisher_.publish(traj);
			break;
		}
		case Content::MISSION:
		{
			radio_comm::serialized_object mission;
			mission.serialized = dp::serialize(boost::any_cast<Mission>(any)).getBuffer();
			missionPublisher_.publish(mission);
			break;
		}
		case Content::LOCAL_FRAME:
		{
			radio_comm::serialized_object local;
			local.serialized = dp::serialize(boost::any_cast<VehicleOneFrame>(any)).getBuffer();
			localFramePublisher_.publish(local);
			break;
		}
		case Content::OVERRIDE:
		{
			radio_comm::serialized_object override;
			override.serialized = dp::serialize(boost::any_cast<Override>(any)).getBuffer();
			overridePublisher_.publish(override);
			break;
		}
		default:
			APLOG_ERROR << "Unknown Content " << (int) content;
			break;
		}
	} catch (boost::bad_any_cast& err)
	{
		APLOG_ERROR << "Bad any cast for Content " << (int) content << ": " << any.empty();
	} catch (std::runtime_error& err)
	{
		APLOG_ERROR << "runtime error in conversion: " << err.what();
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
RadioComm::selectManeuver(radio_comm::select_maneuver::Request& req,
		radio_comm::select_maneuver::Response& resp)
{
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "DataPresentation missing.";
		return false;
	}
	auto packet = dp->serialize(req.maneuver, Content::SELECT_MANEUVER_SET);
	dp->setTarget(packet, Target::MISSION_CONTROL);
	return sendPacket(packet);
}

bool
RadioComm::selectInspectingMetrics(radio_comm::serialized_service::Request& req,
		radio_comm::serialized_service::Response& resp)
{
	auto dp = dataPresentation_.get();

	if (!dp)
	{
		APLOG_ERROR << "Data Presentation Missing.";
		return false;
	}

	InspectingMetricsPair pair = dp::deserialize<InspectingMetricsPair>(req.serialized);

	auto packet = dp->serialize(pair, Content::SELECT_INSPECTING_METRICS);
	dp->setTarget(packet, Target::FLIGHT_ANALYSIS);

	resp.valid = true;

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
	case DataRequest::LOCAL_FRAME:
		target = Target::MISSION_CONTROL;
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
RadioComm::tuneGeneric(radio_comm::tune_generic::Request& req,
		radio_comm::tune_generic::Response& resp)
{
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "DataPresentation missing.";
		return false;
	}

	Packet packet;
	switch (req.id)
	{
	case static_cast<int>(Tuning::LOCAL_PLANNER):
	{
		LocalPlannerParams params;
		params.ParseFromString(req.proto_message);
		resp.valid = true;
		packet = dp->serialize(params, Content::TUNE_LOCAL_PLANNER);
		dp->setTarget(packet, Target::FLIGHT_CONTROL);
		break;
	}
	default:
	{
		APLOG_ERROR << "Tuning not implemented for: "
				<< EnumMap<Tuning>::convert(static_cast<Tuning>(req.id));

		resp.valid = false;
		return false;
	}
	}

	return sendPacket(packet);
}

bool
RadioComm::sendOverride(radio_comm::serialized_service::Request& req,
		radio_comm::serialized_service::Response& resp)
{
	auto dp = dataPresentation_.get();

	if (!dp)
	{
		APLOG_ERROR << "DataPresentation missing.";
		return false;
	}

	Override override = dp::deserialize<Override>(req.serialized);

	auto packet = dp->serialize(override, Content::OVERRIDE);
	dp->setTarget(packet, Target::MISSION_CONTROL);

	resp.valid = true;

	return sendPacket(packet);
}

bool
RadioComm::sendAdvancedControl(radio_comm::send_advanced_control::Request& req,
		radio_comm::send_advanced_control::Response& resp)
{
	auto dp = dataPresentation_.get();

	if (!dp)
	{
		APLOG_ERROR << "DataPresentation Missing.";
		return false;
	}

	AdvancedControl advanced;

	auto specialEnum = EnumMap<SpecialControl>::convert(req.special_sel);

	if (specialEnum == SpecialControl::INVALID)
	{
		APLOG_ERROR << "Invalid Special Control " << req.special_sel;
	}
	else
	{
		advanced.specialSelection = specialEnum;
		advanced.specialValue = req.special_val;
	}

	auto throwsEnum = EnumMap<ThrowsControl>::convert(req.throws_sel);

	if (throwsEnum == ThrowsControl::INVALID)
	{
		APLOG_ERROR << "Invalid Throws Control " << req.throws_sel;
	}
	else
	{
		advanced.throwsSelection = throwsEnum;
	}

	auto camberEnum = EnumMap<CamberControl>::convert(req.camber_sel);

	if (camberEnum == CamberControl::INVALID)
	{
		APLOG_ERROR << "Invalid Camber Control " << req.camber_sel;
	}
	else
	{
		advanced.camberSelection = camberEnum;
		advanced.camberValue = req.camber_val;
	}

	auto packet = dp->serialize(advanced, Content::ADVANCED_CONTROL);
	dp->setTarget(packet, Target::FLIGHT_CONTROL);

	resp.valid_request = true;

	return sendPacket(packet);
}

bool
RadioComm::sendLocalFrame(radio_comm::serialized_service::Request& req,
		radio_comm::serialized_service::Response& resp)
{
	auto dp = dataPresentation_.get();

	if (!dp)
	{
		APLOG_ERROR << "DataPresentation missing.";
		return false;
	}

	VehicleOneFrame frame = dp::deserialize<VehicleOneFrame>(req.serialized);

	auto packet = dp->serialize(frame, Content::LOCAL_FRAME);
	dp->setTarget(packet, Target::MISSION_CONTROL);

	resp.valid = true;

	return sendPacket(packet);
}

bool
RadioComm::sendPacket(const Packet& packet)
{
	return radioSender_.sendPacket(packet);
}
