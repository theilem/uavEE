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
 *   @file DataManager.cpp
 *   @date [dd/mm/yyyy] 3/9/2018
 *   @brief UAV Ground Station Data Manager source file
 */
#include <ros/node_handle.h>
#include "ground_station/DataManager.h"
#include <QFile>
#include <uavAP/FlightControl/Controller/ControlElements/EvaluableControlElements.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <QJsonDocument>
#include "ground_station/ConfigManager.h"
#include "ground_station/MapLogic.h"

std::shared_ptr<DataManager>
DataManager::create(const boost::property_tree::ptree&)
{
    return std::make_shared<DataManager>();
}

void
DataManager::setMission(const radio_comm::serialized_object& mission)
{
	Content content = Content::INVALID;
	Packet packet(mission.serialized);
	boost::any any = dataPresentation_.deserialize(packet, content);
	if (content != Content::MISSION)
	{
		APLOG_ERROR << "Trajectory received cannot be deserialized.";
		return;
	}

	Mission t;
	try
	{
		t = boost::any_cast<Mission>(any);
	} catch (boost::bad_any_cast& err)
	{
		APLOG_ERROR << "Bad any cast for trajectory." << err.what();
		return;
	}
    mapLogic_.get()->setMission(t);
    emit onMission(t);
}

void
DataManager::setPath(const radio_comm::serialized_object& traj)
{
	Trajectory t;
	Content content = Content::INVALID;
	Packet packet(traj.serialized);
	boost::any any = dataPresentation_.deserialize(packet, content);
	if (content != Content::TRAJECTORY)
	{
		APLOG_ERROR << "Trajectory received cannot be deserialized.";
		return;
	}

	try
	{
		t = boost::any_cast<Trajectory>(any);
	} catch (boost::bad_any_cast& err)
	{
		APLOG_ERROR << "Bad any cast for trajectory." << err.what();
		return;
	}
    mapLogic_.get()->setPath(t);
    emit onTrajectory(t);
}

void
DataManager::addSensorData(const simulation_interface::sensor_data &sd)
{
    //adding location to location history
    mapLogic_.get()->addLocation(Vector3(sd.position.x, sd.position.y, sd.position.z));
    mapLogic_.get()->sensorData_ = sd;

    emit onSensorData(sd);
}

void
DataManager::setLocalPlannerStatus(const radio_comm::local_planner_status& status)
{
    LocalPlannerStatus s;
    s.mutable_linear_status()->set_current_path_section(status.linear_status.current_path_section);
    s.mutable_linear_status()->mutable_velocity_target()->set_velocity_x(status.linear_status.velocity_target.velocity_x);
    s.mutable_linear_status()->mutable_velocity_target()->set_velocity_y(status.linear_status.velocity_target.velocity_y);
    s.mutable_linear_status()->mutable_velocity_target()->set_velocity_z(status.linear_status.velocity_target.velocity_z);
    s.mutable_linear_status()->set_yaw_rate_target(status.linear_status.yaw_rate_target);
    s.mutable_linear_status()->mutable_airplane_status()->set_heading_target(status.linear_status.airplane_status.heading_target);
    if (s.has_linear_status())
    {
        lpStatus_ = s;
        mapLogic_.get()->setLocalPlannerStatus(s);
        if (s.linear_status().current_path_section()
                != lpStatus_.linear_status().current_path_section())
        {
            emit onPathSectionChange((int) lpStatus_.linear_status().current_path_section());
        }
    }

}

void
DataManager::onPredictedPower(const power_modeling::power_info& power)
{

}

void
DataManager::addPIDStati(const radio_comm::pidstati &stati)
{
    emit onPIDStati(stati);
}

void
DataManager::notifyAggregationOnUpdate(Aggregator &agg)
{
    mapLogic_.setFromAggregationIfNotSet(agg);
}

void
DataManager::subscribeOnRos()
{
    ros::NodeHandle nh;
    sensorDataSubscriptionRos_ = nh.subscribe("radio_comm/sensor_data", 20, &DataManager::addSensorData, this);
    trajectorySubscriptionRos_ = nh.subscribe("radio_comm/trajectory", 20, &DataManager::setPath, this);
    missionSubscriptionRos_ = nh.subscribe("radio_comm/mission", 20, &DataManager::setMission, this);
    localPlannerDataSubscriptionRos_ = nh.subscribe("radio_comm/local_planner_status", 20, &DataManager::setLocalPlannerStatus, this);
    PIDStatiSubscriptionRos_ = nh.subscribe("radio_comm/pid_stati",20,&DataManager::addPIDStati, this);
    powerModelSubscriptionRos_ = nh.subscribe("power_model/predicted_power", 20, &DataManager::onPredictedPower, this);
    APLOG_DEBUG<<"reached subscribeOnRos";
}

bool
DataManager::run(RunStage stage)
{
    switch (stage)
    {
    case RunStage::INIT:
    {
        if (!mapLogic_.isSet())
        {
            APLOG_ERROR << "DataManager: MapLogic not set";
            return true;
        }
        break;
    }
    case RunStage::NORMAL:
    {
        subscribeOnRos();
        break;
    }
    case RunStage::FINAL:
        break;
    default:
        break;
    }
    return false;
}
