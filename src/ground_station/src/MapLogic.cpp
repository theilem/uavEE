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
#include "ground_station/MapLogic.h"
#include <uavAP/Core/Logging/APLogger.h>
#include "ground_station/IConfigManager.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <radio_comm/request_data.h>
#include <uavAP/Core/DataPresentation/Content.h>

#include "/usr/local/include/uavAP/Core/Scheduler/IScheduler.h"
MapLogic::MapLogic() :
		waypoints_(), currentPath_(-1)
{
}

std::shared_ptr<MapLogic>
MapLogic::create(const Configuration&)
{
	return std::make_shared<MapLogic>();
}

void
MapLogic::notifyAggregationOnUpdate(const Aggregator& agg)
{
	configManager_.setFromAggregationIfNotSet(agg);
	scheduler_.setFromAggregationIfNotSet(agg);
}

bool
MapLogic::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!configManager_.isSet())
		{
			APLOG_ERROR << "MapLogic: IConfigManager is missing.";
			return true;
		}
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "MapLogic: IScheduler is missing.";
			return true;
		}
		break;
	}
	case RunStage::NORMAL:
	{
		ros::NodeHandle nh;
		requestDataService_ = nh.serviceClient<radio_comm::request_data>(
				"/radio_comm/request_data");

		auto scheduler = scheduler_.get();

		scheduler->schedule(std::bind(&MapLogic::askForAll, this), Seconds(1));
		break;
	}
	case RunStage::FINAL:
		break;
	default:
		break;
	}
	return false;
}

const std::vector<Waypoint>&
MapLogic::getWaypoints() const
{
	return waypoints_.waypoints;
}

const Trajectory&
MapLogic::getPath() const
{
	return pathSections_;
}

const std::vector<MapLocation>&
MapLogic::getPathHistory() const
{
	return pathHistory_;
}

int
MapLogic::getCurrentPathSection() const
{
	return currentPath_;
}

bool
MapLogic::askForAll()
{
	return askForMission() && askForTrajectory() && askForSafetyNet() && askForLocalFrame();
}

bool
MapLogic::askForMission()
{
	radio_comm::request_data req;
	req.request.data_request_id = static_cast<int32_t>(DataRequest::MISSION);
	requestDataService_.call(req);
	return req.response.valid_request;
}

bool
MapLogic::askForTrajectory()
{
	radio_comm::request_data req;
	req.request.data_request_id = static_cast<int32_t>(DataRequest::TRAJECTORY);
	requestDataService_.call(req);
	return req.response.valid_request;
}

bool
MapLogic::askForSafetyNet()
{
	radio_comm::request_data req;
	req.request.data_request_id = static_cast<int32_t>(DataRequest::SAFETY_BOUNDS);
	requestDataService_.call(req);
	return req.response.valid_request;
}

void
MapLogic::setSafetyBounds(const Rectanguloid& rect)
{
	safetyRect_ = rect;
}

const Rectanguloid&
MapLogic::getSafetyBounds() const
{
	return safetyRect_;
}

const simulation_interface::sensor_data&
MapLogic::getSensorData() const
{
	return sensorData_;
}

void
MapLogic::addLocation(const Vector3& pos)
{
	MapLocation loc = MapLocation(pos.x(), pos.y());
	if (pathHistory_.size() == 0)
	{
		for (int i = 0; i < FLIGHT_PATH_SIZE; i++)
		{
			pathHistory_.push_back(loc);
		}
	}
	else
	{
		pathHistory_.pop_back();
		pathHistory_.insert(pathHistory_.begin(), loc);
	}

}

void
MapLogic::setMission(const Mission& mission)
{
	waypoints_ = mission;
}

bool
MapLogic::askForLocalFrame()
{
	radio_comm::request_data req;
	req.request.data_request_id = static_cast<int32_t>(DataRequest::LOCAL_FRAME);
	requestDataService_.call(req);
	return req.response.valid_request;
}

void
MapLogic::setPath(const Trajectory& traj)
{
	pathSections_ = traj;
}

std::string
MapLogic::getMapTileDirectory() const
{
	auto configManager = configManager_.get();
	if (!configManager)
	{
		APLOG_ERROR << "MapLogic: ConfigManager objectHandle unset!";
		return "";
	}
	return configManager->getResourcePath() + "/map_tiles/";
}

std::string
MapLogic::getIconPath() const
{
	auto configManager = configManager_.get();
	if (!configManager)
	{
		APLOG_ERROR << "MapLogic: ConfigManager objectHandle unset!";
		return "";
	}
	return configManager->getResourcePath() + "/icons/";
}

void
MapLogic::setLocalFrame(const VehicleOneFrame& frame)
{
	localFrame_ = frame;
}

void
MapLogic::setCriticalPoints(const std::vector<Waypoint>& crit)
{
	criticalPoints_ = crit;
}

const std::vector<Waypoint>&
MapLogic::getCriticalPoints() const
{
	return criticalPoints_;
}
