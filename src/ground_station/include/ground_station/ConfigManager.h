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
 *   @brief UAV Ground Station Configuration Manager header file
 */

#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H
#include <QJsonObject>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/FlightControl/Controller/ControlElements/EvaluableControlElements.h>
#include <uavAP/FlightControl/Controller/PIDController/detail/PIDHandling.h>
#include <uavAP/MissionControl/MissionPlanner/ControlOverride.h>
#include "IConfigManager.h"
#include "IPIDConfigurator.h"
#include <ros/ros.h>

class LayoutGenerator;

/**
 * @brief   The Mode enum is used to symbolize what type of aircraft is being
 *          flown (airplane, helicopter, etc). Certain widgets need to know
 *          because they behave differently for differnt aircraft
 */
enum Mode
{
    AIRPLANE, HELICOPTER, UNDEFINED
};

/**
 * @brief   The ConfigManager class manages configuration data. It gives configs
 *          to widgets and sends configs to the aircraft
 */
class ConfigManager: public IAggregatableObject,
    public IRunnableObject,
    public IPIDConfigurator,
    public IConfigManager
{
public:
    /**
     * @brief   ConfigManager's constructor - does nothing. Create is used instead
     */
    ConfigManager();

    /**
     * @brief   creates a ConfigManager based on json config
     * @param   config is directly passed to configure
     * @return  std::shared_ptr<ConfigManager> to newly instantiated ConfigManager
     */
    static std::shared_ptr<ConfigManager>
    create(const boost::property_tree::ptree& config);

    /**
     * @brief   called by create,
     * @param   config is a json configuration that should specify
     *          ground_station_config_path and ground_station_resource_path
     * @return true on success, false on failure
     */
    bool
    configure(const boost::property_tree::ptree& config);

    const PIDParametersMap&
    getPIDMap() const override;

    boost::property_tree::ptree
    getWidgetConfigByName(const std::string& key) const override;

    const boost::property_tree::ptree&
    getMissionConfig() const override;

    const boost::property_tree::ptree&
    getFlightConfig() const override;

    const boost::property_tree::ptree&
    getGSConfig() const override;

    void
    notifyAggregationOnUpdate(Aggregator&) override;

    const std::string&
    getResourcePath() const override;

    bool
    run(RunStage stage) override;

    bool
    tunePID(const PIDTuning& tunePID) override;

    bool
    sendManeuverOverride(const radio_comm::send_control_override::Request& maneuverOverride) override;

    bool
    sendManeuverSequence(const std::string& maneuver) override;

    bool
    sendMission(const std::string& mission) override;

    /**
     * @brief   startFDAQ starts autopilot data logging
     * @return  true if request was recieved
     */
    bool
    startFDAQ() const;

    /**
     * @brief   stopFDAQ stops autopilot data logging
     * @return  true if request was recieved
     */
    bool
    stopFDAQ() const;

private:
    /**
     * @brief   getWidgetConfigs is a private helper function to specifically get
     *          widget configs from ground station config
     * @return  json containing widget configs
     */
    boost::property_tree::ptree
    getWidgetConfigs() const;

    /**
     * @brief   getMainConfig is a private helper function to specifically get
     *          main config from ground station config
     * @return  json containing main config
     */
    boost::property_tree::ptree
    getMainConfig() const;

    /**
     * @brief   setPIDMap sets internal PIDParameter map to PIDs defined in
     *          flight controller config
     * @param   path string path to aircraft flight controller config
     */
    void
    setPIDMap(const std::string& path);

    ///! property tree representing json configuration for ground station
    boost::property_tree::ptree gsConfig_;

    ///! property tree representing json configuration for aircraft flight config
    boost::property_tree::ptree flightConfig_;

    ///! propery tree representing json configuration for aircraft mission config
    boost::property_tree::ptree missionConfig_;

    ///! string representation of path to resource folder
    std::string resourcePath_;

    ///! Mapping between integer PID controller ID and PIDInfo, which contains
    ///  human readable name and parameters
    PIDParametersMap pidParams_;

    ///! enum representing what mode the aircraft is set to
    Mode mode_;

    ///! ROS service called to tune PIDs
    ros::ServiceClient tunePIDService_;

    ///! ROS service called to request a mission
    ros::ServiceClient selectMissionService_;

    ///! ROS service called to request an override
    ros::ServiceClient manueverOverrideService_;
};

#endif // CONFIGMANAGER_H
