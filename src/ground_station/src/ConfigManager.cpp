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
/**
 *   @author Richard Nai, rnai2@illinois.edu
 *   @file ConfigManager.h
 *   @date [dd/mm/yyyy] 3/9/2018
 *   @brief UAV Ground Station Configuration Manager source file
 */

#include <QJsonDocument>
#include <QFile>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <uavAP/Core/DataPresentation/Content.h>
#include <uavAP/Core/protobuf/messages/LocalPlanner.pb.h>
#include <uavAP/Core/protobuf/messages/ManeuverPlanner.pb.h>
#include <uavAP/FlightAnalysis/StateAnalysis/SteadyStateAnalysis.h>
#include <uavAP/Core/Frames/VehicleOneFrame.h>
#include <uavAP/Core/DataPresentation/BinarySerialization.hpp>
#include <radio_comm/select_mission.h>
#include <radio_comm/select_maneuver.h>
#include <radio_comm/serialized_service.h>
#include <radio_comm/tune_pid.h>
#include <radio_comm/tune_generic.h>

#include "ground_station/LayoutGenerator.h"
#include "ground_station/ConfigManager.h"

ConfigManager::ConfigManager() :
		mode_(UNDEFINED)
{
}

std::shared_ptr<ConfigManager>
ConfigManager::create(const boost::property_tree::ptree& config)
{
	auto cm = std::make_shared<ConfigManager>();
	cm->configure(config);
	return cm;
}

bool
ConfigManager::configure(const boost::property_tree::ptree& config)
{
	std::string path;
	PropertyMapper propertyMapper(config);
	if (propertyMapper.add("ground_station_config_path", path, true))
		boost::property_tree::read_json(path, gsConfig_);
	else
		APLOG_ERROR << "Cannot find groundstation config path.";
	if (!propertyMapper.add("ground_station_resource_path", resourcePath_, true))
		APLOG_ERROR << "Cannot find groundstation resource path.";

	return propertyMapper.map();
}

boost::property_tree::ptree
ConfigManager::getMainConfig() const
{
	PropertyMapper propertyMapper(gsConfig_);
	boost::property_tree::ptree mainconf;
	propertyMapper.add("main_config", mainconf, true);
	return mainconf;
}

boost::property_tree::ptree
ConfigManager::getWidgetConfigs() const
{
	boost::property_tree::ptree widgetconf;
	auto c = getMainConfig();
	PropertyMapper mc(c);
	mc.add("widget_configs", widgetconf, true);
	return widgetconf;
}

boost::property_tree::ptree
ConfigManager::getWidgetConfigByName(const std::string& key) const
{
	auto mc = getWidgetConfigs();
	PropertyMapper wc(mc);
	boost::property_tree::ptree widgetConf;
	if (!wc.add(key, widgetConf, true))
		APLOG_ERROR << "Could not find widget configs for " << key;
	return widgetConf;
}

const boost::property_tree::ptree&
ConfigManager::getMissionConfig() const
{
	return missionConfig_;
}

const boost::property_tree::ptree&
ConfigManager::getFlightConfig() const
{
	return flightConfig_;
}

const boost::property_tree::ptree&
ConfigManager::getGSConfig() const
{
	return gsConfig_;
}

const boost::property_tree::ptree&
ConfigManager::getAlvoloConfig() const
{
	return alvoloConfig_;
}

void
ConfigManager::notifyAggregationOnUpdate(const Aggregator&)
{
}

const std::string&
ConfigManager::getResourcePath() const
{
	return resourcePath_;
}

bool
ConfigManager::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		auto mainConfig = getMainConfig();
		PropertyMapper pm(mainConfig);

		std::string flightConfPath;
		pm.add("flight_control_path", flightConfPath, true);
		boost::property_tree::read_json(flightConfPath, flightConfig_);		//TODO error checking
		APLOG_TRACE << "ConfigManager: Flight config set";

		std::string missionConfPath;
		pm.add("mission_control_path", missionConfPath, true);
		boost::property_tree::read_json(missionConfPath, missionConfig_);
		APLOG_TRACE << "ConfigManager: Mission config set";

		std::string alvoloConfPath;

		if (pm.add("alvolo_path", alvoloConfPath, false))
		{
			boost::property_tree::read_json(alvoloConfPath, alvoloConfig_);
			APLOG_TRACE << "ConfigManager: Alvolo Configuration Set.";
		}
		else
		{
			APLOG_WARN << "ConfigManager: Alvolo Configuration Not Set.";
		}

		std::string mode;
		pm.add("mode", mode, true);
		if (mode == "airplane")
		{
			mode_ = AIRPLANE;
			APLOG_TRACE << "Mode set to airplane";
		}
		else if (mode == "helicopter")
		{
			mode_ = HELICOPTER;
			APLOG_TRACE << "Mode set to helicopter";
		}
		else
		{
			APLOG_ERROR << "Could not set aircraft mode";
			return true;
		}
		setPIDMap(flightConfPath);
		APLOG_TRACE << "PIDS in SDM have been set with confPath: " << flightConfPath;
		break;
	}
	case RunStage::NORMAL:
	{
		ros::NodeHandle nh;
		selectMissionService_ = nh.serviceClient<radio_comm::select_mission>(
				"/radio_comm/select_mission");
		selectManeuverService_ = nh.serviceClient<radio_comm::select_maneuver>(
				"/radio_comm/select_maneuver");
		selectInspectingMetricsService_ = nh.serviceClient<radio_comm::serialized_service>(
				"/radio_comm/select_inspecting_metrics");
		genericTuningService_ = nh.serviceClient<radio_comm::tune_generic>(
				"/radio_comm/tune_generic");
		tunePIDService_ = nh.serviceClient<radio_comm::tune_pid>("/radio_comm/tune_pid");
		overrideService_ = nh.serviceClient<radio_comm::serialized_service>(
				"/radio_comm/send_override");
		controllerOutputOffsetService_ = nh.serviceClient<radio_comm::serialized_service>(
				"/radio_comm/send_controller_output_offset");
		advancedControlService_ = nh.serviceClient<radio_comm::send_advanced_control>(
				"/radio_comm/send_advanced_control");
		localFrameService_ = nh.serviceClient<radio_comm::serialized_service>(
				"/radio_comm/send_local_frame");
		break;
	}
	case RunStage::FINAL:
		break;
	default:
		break;
	}
	return false;
}

bool
ConfigManager::tunePID(const PIDTuning& tunePID)
{
	radio_comm::tune_pid req;
	req.request.id = tunePID.pid;
	req.request.kp = tunePID.params.kp;
	req.request.ki = tunePID.params.ki;
	req.request.kd = tunePID.params.kd;
	req.request.ff = tunePID.params.ff;
	req.request.imax = tunePID.params.imax;
	return tunePIDService_.call(req);
}

bool
ConfigManager::tuneLocalPlanner(const LocalPlannerParams& params)
{
	radio_comm::tune_generic ser;
	ser.request.id = static_cast<int>(Tuning::LOCAL_PLANNER);
	ser.request.proto_message = params.SerializeAsString();
	return genericTuningService_.call(ser);
}

bool
ConfigManager::sendOverride(const Override& override)
{
	radio_comm::serialized_service ser;
	ser.request.serialized = dp::serialize(override).getBuffer();
	return overrideService_.call(ser);
}

bool
ConfigManager::sendControllerOutputOffset(const ControllerOutput& offset)
{
	radio_comm::serialized_service ser;
	ser.request.serialized = dp::serialize(offset).getBuffer();
	return controllerOutputOffsetService_.call(ser);
}

bool
ConfigManager::sendManeuverSet(const std::string& maneuver)
{
	radio_comm::select_maneuver req;
	req.request.maneuver = maneuver;
	return selectManeuverService_.call(req);
}

bool
ConfigManager::sendMission(const std::string& mission)
{
	radio_comm::select_mission req;
	req.request.mission = mission;
	return selectMissionService_.call(req);
}

bool
ConfigManager::sendInspectingMetrics(const InspectingMetricsPair& pair)
{
	radio_comm::serialized_service ser;
	ser.request.serialized = dp::serialize(pair).getBuffer();
	return selectInspectingMetricsService_.call(ser);
}

bool
ConfigManager::startFDAQ() const
{
	return true; //rc_.get()->startFDAQ(); TODO fix
}

bool
ConfigManager::tuneManeuverPlanner(const ManeuverPlannerParams& params)
{
	return true;
}

bool
ConfigManager::stopFDAQ() const
{
	return true; //rc_.get()->stopFDAQ(); TODO fix
}

bool
ConfigManager::sendAdvancedControl(
		const radio_comm::send_advanced_control::Request& maneuverOverride)
{
	radio_comm::send_advanced_control req;
	req.request = maneuverOverride;
	return advancedControlService_.call(req); //rc_.get()->write(maneuverOverride); TODO fix
}

bool
ConfigManager::sendLocalFrame(const VehicleOneFrame& frame)
{
	radio_comm::serialized_service ser;
	ser.request.serialized = dp::serialize(frame).getBuffer();
	return localFrameService_.call(ser);
}

void
ConfigManager::setPIDMap(const std::string& path)
{
	boost::property_tree::ptree conf;
	boost::property_tree::read_json(path, conf);
	try
	{
		auto controllerConf = conf.get_child("controller");
		auto pidConf = controllerConf.get_child("pids");

		for (auto& it : pidConf)
		{
			auto id = EnumMap<PIDs>::convert(it.first);
			if (id == PIDs::INVALID)
			{
				APLOG_WARN << "Airplane PID name " << it.first << " invalid.";
				continue;
			}
			Control::PID::Parameters params;
			params.configure(it.second);

			PIDInfo info(it.first, params);
			pidParams_.insert(std::make_pair((int) id, info));
		}
	} catch (boost::property_tree::ptree_error& err)
	{
		APLOG_ERROR << "Error in config file: " << err.what();
		return;
	}
}

const PIDParametersMap&
ConfigManager::getPIDMap() const
{
	return pidParams_;
}
