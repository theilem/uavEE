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
#include <simulation_interface/sensor_data.h>
#include <radio_comm/request_data.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <uavAP/Core/Frames/VehicleOneFrame.h>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/MissionControl/Geofencing/Rectanguloid.h>

class IConfigManager;
class IScheduler;

#define FLIGHT_PATH_SIZE 600

class MapLogic: public IAggregatableObject, public IRunnableObject
{
	friend class DataManager;
public:

	static constexpr TypeId typeId = "map_logic";

	MapLogic();

	static std::shared_ptr<MapLogic>
	create(const Configuration&);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

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

	bool
	askForAll();

	bool
	askForMission();

	bool
	askForTrajectory();

	bool
	askForSafetyNet();

	bool
	askForLocalFrame();

	void
	setSafetyBounds(const Rectanguloid& rect);

	void
	setLocalFrame(const VehicleOneFrame& frame);

	const Rectanguloid&
	getSafetyBounds() const;

	const simulation_interface::sensor_data&
	getSensorData() const;

	std::string
	getMapTileDirectory() const;

	std::string
	getIconPath() const;

	void
	setCriticalPoints(const std::vector<Waypoint>& crit);


	const std::vector<Waypoint>&
	getCriticalPoints() const;

private:

	void
	addLocation(const Vector3& pos);

	void
	setMission(const Mission& mission);

	void
	setPath(const Trajectory& traj);

	Rectanguloid safetyRect_;
	ObjectHandle<IConfigManager> configManager_;
	ObjectHandle<IScheduler> scheduler_;
	Mission waypoints_;
	Trajectory pathSections_;
	std::vector<Waypoint> criticalPoints_;
	int currentPath_;
	std::vector<MapLocation> pathHistory_;
	ControllerTarget controllerTarget_;
	simulation_interface::sensor_data sensorData_;
	VehicleOneFrame localFrame_;

	ros::ServiceClient requestDataService_;
};

#endif // MAPLOGIC_H
