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

#include "ground_station/ConfigManager.h"
#include <QJsonDocument>
#include <QFile>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <radio_comm/select_mission.h>
#include <radio_comm/send_control_override.h>
#include <radio_comm/tune_pid.h>
#include "ground_station/LayoutGenerator.h"

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

void
ConfigManager::notifyAggregationOnUpdate(Aggregator&)
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
        selectMissionService_ = nh.serviceClient<radio_comm::select_mission>("/radio_comm/select_mission");
        tunePIDService_ = nh.serviceClient<radio_comm::tune_pid>("/radio_comm/tune_pid");
        manueverOverrideService_ = nh.serviceClient<radio_comm::send_control_override>("/radio_comm/send_control_override");
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
ConfigManager::sendManeuverOverride(const radio_comm::send_control_override::Request& maneuverOverride)
{
    radio_comm::send_control_override req;
    req.request = maneuverOverride;
    return manueverOverrideService_.call(req); //rc_.get()->write(maneuverOverride); TODO fix
}

bool
ConfigManager::sendManeuverSequence(const std::string& maneuver)
{
    return true; //rc_.get()->writeManeuver(maneuver); TODO fix
}

bool
ConfigManager::sendMission(const std::string& mission)
{
    radio_comm::select_mission req;
    req.request.mission = mission;
    return selectMissionService_.call(req);
}

bool
ConfigManager::startFDAQ() const
{
    return true; //rc_.get()->startFDAQ(); TODO fix
}

bool
ConfigManager::stopFDAQ() const
{
    return true; //rc_.get()->stopFDAQ(); TODO fix
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
            auto map = AirplanePIDBimapRight.find(it.first);
            if (map == AirplanePIDBimapRight.end())
            {
                APLOG_WARN << "Airplane PID name " << it.first << " invalid.";
                continue;
            }
            Control::PID::Parameters params;
            params.configure(it.second);
            std::string name;
            if (mode_ == AIRPLANE)
            {
                auto map2 = AirplanePIDBimapLeft.find((AirplanePIDs) (int) map->second); //TODO Check Airplane or Helicopter
                if (map2 == AirplanePIDBimapLeft.end())
                {
                    name = "Invalid";
                }
                else
                {
                    name = map2->second;
                }
            }
            else if (mode_ == HELICOPTER)
            {
                auto map2 = HelicopterPIDBimapLeft.find((HelicopterPIDs) (int) map->second); //TODO Check Airplane or Helicopter
                if (map2 == HelicopterPIDBimapLeft.end())
                {
                    name = "Invalid";
                }
                else
                {
                    name = map2->second;
                }
            }
            PIDInfo info(name, params);
            pidParams_.insert(std::make_pair((int) map->second, info));
        }
    }
    catch (boost::property_tree::ptree_error& err)
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
