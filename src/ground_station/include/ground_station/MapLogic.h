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
#ifndef MAPLOGIC_H
#define MAPLOGIC_H
#include <uavAP/Core/Object/IAggregatableObject.h>
#include "ground_station/MapLocation.h"
#include <uavAP/MissionControl/GlobalPlanner/Trajectory.h>
#include <uavAP/MissionControl/MissionPlanner/Mission.h>
#include <uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/LinearLocalPlanner.h>
#include <uavAP/Core/protobuf/messages/Shapes.pb.h>
#include <simulation_interface/sensor_data.h>
#include <radio_comm/request_data.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <uavAP/Core/Runner/IRunnableObject.h>

class IConfigManager;

#define FLIGHT_PATH_SIZE 600

class MapLogic: public IAggregatableObject, public IRunnableObject
{
    friend class DataManager;
public:
    MapLogic();

    static std::shared_ptr<MapLogic>
    create(const boost::property_tree::ptree&);

    void
    notifyAggregationOnUpdate(Aggregator& agg) override;

    bool
    run(RunStage stage) override;

    const std::vector<Waypoint>&
    getWaypoints() const;

    const Trajectory&
    getPath() const;

    const std::vector<MapLocation>&
    getPathHistory() const;

    int
    getCurrentPathSection() const;

    const LocalPlannerStatus&
    getLocalPlannerStatus() const;

    bool
    askForMission();

    bool
    askForTrajectory();

    bool
    askForSafetyNet();

    void
    setSafetyBounds(const Rectangle& rect);

    const Rectangle&
    getSafetyBounds() const;

    const simulation_interface::sensor_data&
    getSensorData() const;

    std::string
    getMapTileDirectory() const;

    std::string
    getIconPath() const;

private:

    void
    setLocalPlannerStatus(const LocalPlannerStatus& lpStatus);

    void
    addLocation(const Vector3& pos);

    void
    setMission(const Mission& mission);

    void
    setPath(const Trajectory& traj);

    Rectangle safetyRect_;
    ObjectHandle<IConfigManager> configManager_;
    Mission waypoints_;
    Trajectory pathSections_;
    int currentPath_;
    std::vector<MapLocation> pathHistory_;
    LocalPlannerStatus lpStatus_;
    ControllerTarget controllerTarget_;
    simulation_interface::sensor_data sensorData_;

    ros::ServiceClient requestDataService_;
};

#endif // MAPLOGIC_H
