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
#include <ros/ros.h>

#include "uavAP/Core/Frames/VehicleOneFrame.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/FlightControl/Controller/ControlElements/EvaluableControlElements.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDMapping.h"
#include "uavAP/MissionControl/ManeuverPlanner/Override.h"
#include "IConfigManager.h"
#include "IPIDConfigurator.h"
#include <ros/subscriber.h>
#include <std_msgs/String.h>

class LayoutGenerator;
class DataPresentation;

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

	static constexpr TypeId typeId = "config_manager";
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
	create(const Configuration& config);

	/**
	 * @brief   called by create,
	 * @param   config is a json configuration that should specify
	 *          ground_station_config_path and ground_station_resource_path
	 * @return true on success, false on failure
	 */
	bool
	configure(const Configuration& config);

	const PIDParametersMap&
	getPIDMap() const override;

	Configuration
	getWidgetConfigByName(const std::string& key) const override;

	const Configuration&
	getMissionConfig() const override;

	const Configuration&
	getFlightConfig() const override;

	const Configuration&
	getGSConfig() const override;

	const Configuration&
	getAlvoloConfig() const override;

	void
	notifyAggregationOnUpdate(const Aggregator&) override;

	const std::string&
	getResourcePath() const override;

	bool
	run(RunStage stage) override;

	bool
	tunePID(const PIDTuning& tunePID) override;

	bool
	tuneManeuverPlanner(const ManeuverPlannerParams& params) override;

	bool
	sendOverride(const Override& override) override;

	bool
	sendControllerOutputOffset(const ControllerOutput& offset) override;

	bool
	sendWindAnalysisStatus(const WindAnalysisStatus& windAnalysisStatus) override;

	bool
	sendAdvancedControl(const radio_comm::send_advanced_control::Request& maneuverOverride)
			override;

	bool
	sendLocalFrame(const VehicleOneFrame& frame) override;

	bool
	sendManeuverSet(const std::string& maneuver) override;

	bool
	sendInspectingMetrics(const InspectingMetricsPair& pair) override;

	bool
	sendMission(const std::string& mission) override;

	bool
	engine(const bool& start) override;

	void
	publishGroundStationSensorData(const simulation_interface::sensor_data& sensorData) override;

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

	void
	requestPIDParams() override;

private:
	/**
	 * @brief   getWidgetConfigs is a private helper function to specifically get
	 *          widget configs from ground station config
	 * @return  json containing widget configs
	 */
	Configuration
	getWidgetConfigs() const;

	/**
	 * @brief   getMainConfig is a private helper function to specifically get
	 *          main config from ground station config
	 * @return  json containing main config
	 */
	Configuration
	getMainConfig() const;

	/**
	 * @brief   setPIDMap sets internal PIDParameter map to PIDs defined in
	 *          flight controller config
	 * @param   path string path to aircraft flight controller config
	 */
	void
	setPIDMap(const std::string& path);

	void
	setPIDMap(const PIDParams& pidParams);

	void
	onPIDParams(const std_msgs::String& string);

	ObjectHandle<DataPresentation> dataPresentation_;

	///! property tree representing json configuration for ground station
	Configuration gsConfig_;

	///! property tree representing json configuration for aircraft flight config
	Configuration flightConfig_;

	///! propery tree representing json configuration for aircraft mission config
	Configuration missionConfig_;

	///! propery tree representing json configuration for aircraft alvolo config
	Configuration alvoloConfig_;

	///! string representation of path to resource folder
	std::string resourcePath_;

	///! Mapping between integer PID controller ID and PIDInfo, which contains
	///  human readable name and parameters
	PIDParametersMap pidParams_;

	///! enum representing what mode the aircraft is set to
	Mode mode_;

	///! ROS service called to tune generic protobuf params
	ros::ServiceClient genericTuningService_;

	///! ROS service called to tune PIDs
	ros::ServiceClient tunePIDService_;

	///! ROS service called to request a mission
	ros::ServiceClient selectMissionService_;

	///! ROS service called to request a maneuver set
	ros::ServiceClient selectManeuverService_;

	///! ROS service called to request an inspecting metrics
	ros::ServiceClient selectInspectingMetricsService_;

	///! ROS service called to request an override
	ros::ServiceClient overrideService_;

	///! ROS service called to request a controller output offset
	ros::ServiceClient controllerOutputOffsetService_;

	///! ROS service called to request a wind analysis status
	ros::ServiceClient windAnalysisStatusService_;

	///! ROS service called to request an advanced control
	ros::ServiceClient advancedControlService_;

	///! ROS service called to request a local frame
	ros::ServiceClient localFrameService_;

	///! ROS service called to start or stop aircraft engine in X-Plane
	ros::ServiceClient engineService_;

	ros::ServiceClient requestDataService_;

	ros::Subscriber pidParamsSub_;

	///! publication for ground station sensor data
	ros::Publisher sensorDataPublisherGroundStation_;
};


#endif // CONFIGMANAGER_H
