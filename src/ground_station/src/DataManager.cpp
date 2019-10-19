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

#include <QFile>
#include <QJsonDocument>
#include <ros/node_handle.h>

#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/Frames/VehicleOneFrame.h>
#include "uavAP/FlightControl/Controller/ControlElements/EvaluableControlElements.h"
#include "ground_station/ConfigManager.h"
#include "ground_station/MapLogic.h"
#include "ground_station/DataManager.h"
#include <uavAP/Core/DataPresentation/BinarySerialization.hpp>

std::shared_ptr<DataManager>
DataManager::create(const Configuration&)
{
	return std::make_shared<DataManager>();
}

void
DataManager::setMission(const radio_comm::serialized_object& mission)
{
	auto dp = dataPresentation_.get();
	Packet packet(mission.serialized);
	Mission t = dp->deserialize<Mission>(packet);

	mapLogic_.get()->setMission(t);
	emit onMission(t);
}

void
DataManager::setPath(const radio_comm::serialized_object& traj)
{
	auto dp = dataPresentation_.get();
	Packet packet(traj.serialized);
	Trajectory t = dp->deserialize<Trajectory>(packet);

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
DataManager::addXPlaneSensorData(const simulation_interface::sensor_data &sd)
{
	emit onXPlaneSensorData(sd);
}

void
DataManager::setLocalPlannerStatus(const radio_comm::serialized_proto& status)
{
}

void
DataManager::onPredictedPower(const power_modeling::power_info& power)
{

}

void
DataManager::addOverride(const radio_comm::serialized_object& override)
{
	auto dp = dataPresentation_.get();
	Packet packet(override.serialized);
	Override t = dp->deserialize<Override>(packet);

	emit onOverride(t);
}

void
DataManager::addControllerOutputTrim(const radio_comm::serialized_object& trim)
{
	auto dp = dataPresentation_.get();
	Packet packet(trim.serialized);
	ControllerOutput t = dp->deserialize<ControllerOutput>(packet);

	emit onControllerOutputTrim(t);
}

void
DataManager::addWindAnalysisStatus(const radio_comm::serialized_object& windAnalysisStatus)
{
	auto dp = dataPresentation_.get();
	Packet packet(windAnalysisStatus.serialized);
	WindAnalysisStatus t = dp->deserialize<WindAnalysisStatus>(packet);

	emit onWindAnalysisStatus(t);
}

void
DataManager::addPIDStati(const radio_comm::pidstati &stati)
{
	emit onPIDStati(stati);
}

void
DataManager::addInspectingMetrics(const radio_comm::serialized_object& inspectingMetrics)
{
	auto dp = dataPresentation_.get();
	Packet packet(inspectingMetrics.serialized);
	SteadyStateMetrics t = dp->deserialize<SteadyStateMetrics>(packet);

	emit onInspectingMetrics(t);
}

void
DataManager::notifyAggregationOnUpdate(const Aggregator &agg)
{
	mapLogic_.setFromAggregationIfNotSet(agg);
	dataPresentation_.setFromAggregationIfNotSet(agg);
}

void
DataManager::setSafetyBounds(const std_msgs::String& bounds)
{
	auto dp = dataPresentation_.get();
	Packet p(bounds.data);

	Rectanguloid rect = dp->deserialize<Rectanguloid>(p);
	auto ml = mapLogic_.get();

	if (!ml)
	{
		APLOG_ERROR << "DataManager: Map Logic Missing.";
		return;
	}

	ml->setSafetyBounds(rect);
}

void
DataManager::setLocalFrame(const radio_comm::serialized_object& localFrame)
{
	auto dp = dataPresentation_.get();
	Packet packet(localFrame.serialized);
	VehicleOneFrame frame = dp->deserialize<VehicleOneFrame>(packet);

	mapLogic_.get()->setLocalFrame(frame);
	emit onLocalFrame(frame);
}

void
DataManager::subscribeOnRos()
{
	ros::NodeHandle nh;
	sensorDataSubscriptionRos_ = nh.subscribe("radio_comm/sensor_data", 20,
			&DataManager::addSensorData, this);
	sensorDataSubscriptionXPlane_ = nh.subscribe("/x_plane_interface/sensor_data", 20,
			&DataManager::addXPlaneSensorData, this);
	trajectorySubscriptionRos_ = nh.subscribe("radio_comm/trajectory", 20, &DataManager::setPath,
			this);
	missionSubscriptionRos_ = nh.subscribe("radio_comm/mission", 20, &DataManager::setMission,
			this);
	overrideSubscriptionRos_ = nh.subscribe("radio_comm/override", 20,
				&DataManager::addOverride, this);
	controllerOutputTrimSubscriptionRos_ = nh.subscribe("radio_comm/controller_output_trim", 20,
				&DataManager::addControllerOutputTrim, this);
	windAnalysisStatusSubscriptionRos_ = nh.subscribe("radio_comm/wind_analysis_status", 20,
				&DataManager::addWindAnalysisStatus, this);
	localFrameSubscriptionRos_ = nh.subscribe("radio_comm/local_frame", 20,
			&DataManager::setLocalFrame, this);
	localPlannerDataSubscriptionRos_ = nh.subscribe("radio_comm/local_planner_status", 20,
			&DataManager::setLocalPlannerStatus, this);
	PIDStatiSubscriptionRos_ = nh.subscribe("radio_comm/pid_stati", 20, &DataManager::addPIDStati,
			this);
	inspectingMetricsSubscriptionRos_ = nh.subscribe("radio_comm/inspecting_metrics", 20,
			&DataManager::addInspectingMetrics, this);
	powerModelSubscriptionRos_ = nh.subscribe("power_model/predicted_power", 20,
			&DataManager::onPredictedPower, this);

	safetyBoundsSubscriptionRos_ = nh.subscribe("radio_comm/safety_bounds", 20,
			&DataManager::setSafetyBounds, this);
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
