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
 *   @file IConfigManager.h
 *   @date [DD/MM/YY] 31/3/2018
 *   @brief
 */

#include <string>
#include <boost/property_tree/ptree.hpp>

#include <radio_comm/send_control_override.h>

#ifndef ICONFIGMANAGER_H
#define ICONFIGMANAGER_H

class IConfigManager
{
public:
    virtual
    ~IConfigManager()
    {
    }

    /**
     * @brief   getWidgetConfigByName Returns json configuration of widget based
     *          off input widget name
     * @param   key is the name of the widget
     * @return  json configuration for widget that matches input name
     */
    virtual boost::property_tree::ptree
    getWidgetConfigByName(const std::string& key) const = 0;

    /**
     * @brief   getMissionConfig returns mission control json that the aircraft
     *          is using - contains data like maneuvers and waypoints
     * @return  mission control json specified in ground station config json
     *          that should be set to same json aircraft is using
     */
    virtual const boost::property_tree::ptree&
    getMissionConfig() const = 0;

    /**
     * @brief   getFlightConfig returns flight control json that the aircraft is
     *          using - contains data like pid gains, and path planning constants
     * @return  flight control json specified in ground station config json that
     *          should be set to same json aircraft is using
     */
    virtual const boost::property_tree::ptree&
    getFlightConfig() const = 0;

    /**
     * @brief   getGSConfig returns json that the ground station was launched
     *          with, contains data like widget layout, paths to aircraft jsons,
     *          and widget configs
     * @return  ground station config json
     */
    virtual const boost::property_tree::ptree&
    getGSConfig() const = 0;

    /**
     * @brief   getResourcePath gets the path to folder of resource files
     * @return  string path of resource directory
     */
    virtual const std::string&
    getResourcePath() const = 0;

    /**
     * @brief   write sends a aircraft maneuver override command to the aircraft
     * @param   maneuverOverride a ros service request containing maneuver override command
     * @return  true if request was recieved
     */
    virtual bool
    sendManeuverOverride(const radio_comm::send_control_override::Request& maneuverOverride) = 0;

    /**
     * @brief   writeManeuver sends a maneuver command telling the aircraft what
     *          sequence of maneuvers to fly.
     * @param   maneuver string identifier of requested maneuver sequence
     * @return  true if request was recieved
     */
    virtual bool
    sendManeuverSequence(const std::string& maneuver) = 0;

    /**
     * @brief   writeMission sends a mission command telling the aircraft what
     *          sequence of waypoints to fly
     * @param   mission string identifier of requested waypoint sequence
     * @return  true if request was recieved
     */
    virtual bool
    sendMission(const std::string& mission) = 0;
};

#endif // ICONFIGMANAGER_H
