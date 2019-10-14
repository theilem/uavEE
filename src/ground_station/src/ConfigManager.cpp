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
#include <uavAP/FlightAnalysis/StateAnalysis/SteadyStateAnalysis.h>
#include <uavAP/Core/Frames/VehicleOneFrame.h>
#include <uavAP/Core/DataPresentation/BinarySerialization.hpp>
#include <radio_comm/engine.h>
#include <radio_comm/select_mission.h>
#include <radio_comm/select_maneuver.h>
#include <radio_comm/serialized_service.h>
#include <radio_comm/tune_pid.h>
#include <radio_comm/tune_generic.h>
#include <radio_comm/request_data.h>
#include <uavAP/Core/DataPresentation/DataPresentation.h>

#include "ground_station/LayoutGenerator.h"
#include "ground_station/ConfigManager.h"

ConfigManager::ConfigManager() :
		mode_(UNDEFINED)
{
}

std::shared_ptr<ConfigManager>
ConfigManager::create(const Configuration& config)
{
	auto cm = std::make_shared<ConfigManager>();
	cm->configure(config);
	return cm;
}

bool
ConfigManager::configure(const Configuration& config)
{
	std::string path;
	PropertyMapper<Configuration> propertyMapper(config);
	if (propertyMapper.add("ground_station_config_path", path, true))
		boost::property_tree::read_json(path, gsConfig_);
	else
		APLOG_ERROR << "Cannot find groundstation config path.";
	if (!propertyMapper.add("ground_station_resource_path", resourcePath_, true))
		APLOG_ERROR << "Cannot find groundstation resource path.";

	return propertyMapper.map();
}

Configuration
ConfigManager::getMainConfig() const
{
	PropertyMapper<Configuration> propertyMapper(gsConfig_);
	Configuration mainconf;
	propertyMapper.add("main_config", mainconf, true);
	return mainconf;
}

Configuration
ConfigManager::getWidgetConfigs() const
{
	Configuration widgetconf;
	auto c = getMainConfig();
	PropertyMapper<Configuration> mc(c);
	mc.add("widget_configs", widgetconf, true);
	return widgetconf;
}

Configuration
ConfigManager::getWidgetConfigByName(const std::string& key) const
{
	auto mc = getWidgetConfigs();
	PropertyMapper<Configuration> wc(mc);
	Configuration widgetConf;
	if (!wc.add(key, widgetConf, true))
		APLOG_ERROR << "Could not find widget configs for " << key;
	return widgetConf;
}

const Configuration&
ConfigManager::getMissionConfig() const
{
	return missionConfig_;
}

const Configuration&
ConfigManager::getFlightConfig() const
{
	return flightConfig_;
}

const Configuration&
ConfigManager::getGSConfig() const
{
	return gsConfig_;
}

const Configuration&
ConfigManager::getAlvoloConfig() const
{
	return alvoloConfig_;
}

void
ConfigManager::notifyAggregationOnUpdate(const Aggregator& agg)
{
	dataPresentation_.setFromAggregationIfNotSet(agg);
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
		PropertyMapper<Configuration> pm(mainConfig);

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

		requestDataService_ = nh.serviceClient<radio_comm::request_data>(
				"/radio_comm/request_data");
		engineService_ = nh.serviceClient<radio_comm::engine>("/x_plane_interface/engine");
		sensorDataPublisherGroundStation_ = nh.advertise<simulation_interface::sensor_data>(
				"/ground_station/sensor_data", 20);

		pidParamsSub_ = nh.subscribe("radio_comm/pid_params", 20,
				&ConfigManager::onPIDParams, this);


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
ConfigManager::onPIDParams(const std_msgs::String& string)
{
	auto dp = dataPresentation_.get();
	Packet p(string.data);

	PIDParams params = dp->deserialize<PIDParams>(p);
	setPIDMap(params);
}


bool
ConfigManager::tunePID(const PIDTuning& tunePID)
{
	radio_comm::tune_pid req;
	req.request.id = tunePID.pid;
	req.request.kp = tunePID.params.kp();
	req.request.ki = tunePID.params.ki();
	req.request.kd = tunePID.params.kd();
	req.request.ff = tunePID.params.ff();
	req.request.imax = tunePID.params.imax();
	return tunePIDService_.call(req);
}

bool
ConfigManager::sendOverride(const Override& override)
{
	auto dp = dataPresentation_.get();
	radio_comm::serialized_service ser;
	ser.request.serialized = dp->serialize(override).getBuffer();
	return overrideService_.call(ser);
}

bool
ConfigManager::sendControllerOutputOffset(const ControllerOutput& offset)
{
	auto dp = dataPresentation_.get();
	radio_comm::serialized_service ser;
	ser.request.serialized = dp->serialize(offset).getBuffer();
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
	auto dp = dataPresentation_.get();
	radio_comm::serialized_service ser;
	ser.request.serialized = dp->serialize(pair).getBuffer();
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
	auto dp = dataPresentation_.get();
	radio_comm::serialized_service ser;
	ser.request.serialized = dp->serialize(frame).getBuffer();
	return localFrameService_.call(ser);
}

bool
ConfigManager::engine(const bool& start)
{
	radio_comm::engine engineService;
	engineService.request.start = start;
	return engineService_.call(engineService);
}

void
ConfigManager::publishGroundStationSensorData(const simulation_interface::sensor_data& sensorData)
{
	sensorDataPublisherGroundStation_.publish(sensorData);
}

void
ConfigManager::setPIDMap(const std::string& path)
{
	Configuration conf;
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
			Control::PIDParameters params;
			PropertyMapper<Configuration> pm(it.second);
			params.configure(pm);

			PIDInfo info(it.first, params);
			pidParams_.insert(std::make_pair((int) id, info));
		}
	} catch (ConfigurationError& err)
	{
		APLOG_ERROR << "Error in config file: " << err.what();
		return;
	}
}

void
ConfigManager::setPIDMap(const PIDParams& pidParams)
{

	pidParams_.clear();
	for (const auto& pid : pidParams)
	{
		PIDInfo info(EnumMap<PIDs>::convert(pid.first), pid.second);
		pidParams_.insert(std::make_pair((int) pid.first, info));
	}

}

const PIDParametersMap&
ConfigManager::getPIDMap() const
{
	return pidParams_;
}

void
ConfigManager::requestPIDParams()
{
	radio_comm::request_data req;
	req.request.data_request_id = static_cast<int32_t>(DataRequest::PID_PARAMS);
	requestDataService_.call(req);
}
