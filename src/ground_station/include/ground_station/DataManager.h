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
 *   @file DataManager.h
 *   @date [dd/mm/yyyy] 3/9/2018
 *   @brief UAV Ground Station Data Manager header file
 */
#ifndef DATAMANAGER_H
#define DATAMANAGER_H
#include <map>
#include <QObject>
#include <uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h>
#include <uavAP/Core/Runner/SimpleRunner.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <QJsonObject>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/protobuf/messages/LocalPlanner.pb.h>
#include "ground_station/IDataSignals.h"
#include <boost/property_tree/ptree.hpp>
#include <ros/subscriber.h>
#include <radio_comm/local_planner_status.h>
#include <radio_comm/serialized_object.h>
#include <power_modeling/power_info.h>
#include <uavAP/Core/DataPresentation/Content.h>
#include <uavAP/Core/DataPresentation/APDataPresentation/APDataPresentation.h>

class MapLogic;
class ConfigManager;

/**
 * @brief   The DataManager class manages all data going through the ground station.
 */
class DataManager: public IDataSignals,
        public IRunnableObject,
        public IAggregatableObject
{
Q_OBJECT
public:

    /**
     * @brief default constructor
     */
	DataManager() = default;

    /**
     * @brief   creates a DataManager. Has a placeholder parameter for json config
     *          that is not used
     * @return  std::shared_ptr<DataManager> to newly instantiated ConfigManager
     */
    static std::shared_ptr<DataManager>
	create(const boost::property_tree::ptree&);

    /**
     * @brief   notifyAggregationOnUpdate sets references to ConfigManager and MapLogic
     * @param   agg is the reference to aggregator that contains all the references
     */
	void
	notifyAggregationOnUpdate(Aggregator& agg) override;

    /**
     * @brief   run calls the runstages. init checks to see if the objectHandles
     *          are set properly and normal subscribes on ros messages
     * @param   stage enum defining which runstage is being run
     * @return  false on success, true if error occured
     */
	bool
	run(RunStage stage) override;

signals:
	void
	onActuationData(const simulation_interface::actuation&) override;
	void
	onSensorData(const simulation_interface::sensor_data&) override;
	void
	onMission(const Mission&) override;
	void
	onTrajectory(const Trajectory&) override;
	void
	onPathSectionChange(int) override;
	void
	onLocalPlannerStatus(const LocalPlannerStatus&) override;
	void
	onPIDStati(const radio_comm::pidstati&) override;

private:

    /**
     * @brief setMission handles onMission. It sends the mission to the linked
     * mapLogic and signals all widgets subscribed to onMission
     * @param mission the mission recieved
     */
	void
	setMission(const radio_comm::serialized_object& mission);

    /**
     * @brief setPath handles onTrajectory. It sends the trajectory to the
     * linked mapLogic and signals all widgets subscribed to onTrajectory
     * @param traj is the trajectroy recieved
     */
	void
	setPath(const radio_comm::serialized_object& traj);

    /**
     * @brief addSensorData is the handler function of onSensorData. It sets the
     * member sensor data to the new recieved sensor data and signals all the
     * widgets subscribed to onSensorData
     * @param sd is the recieved sensor data
     */
	void
	addSensorData(const simulation_interface::sensor_data &sd);

    /**
     * @brief setLocalPlannerStatus handles onLocalPlannerStatus. It sets the
     *        member local planner status to the new recieved local planner status and
     *        signals all the widgets subscribed onLocalPlannerStatus
     *
     * @param status
     */
	void
	setLocalPlannerStatus(const radio_comm::local_planner_status& status);

    /**
     * @brief onPredictedPower handlers ROS messages containing predicted power
     * @param power ROS power message
     */
    void
	onPredictedPower(const power_modeling::power_info& power);

    /**
     * @brief addPIDStati handles onPIDStati. It signals all connected widgets
     * that there is new PIDStati
     * @param stati
     */
	void
	addPIDStati(const radio_comm::pidstati &stati);

    /**
     * @brief subscribeOnRos is an initializer function that initializes
     */
	void
	subscribeOnRos();

    ///! member used to check if the current path section has changed
	LocalPlannerStatus lpStatus_;

    ///! reference to MapLogic for map widgets to have latest data
	ObjectHandle<MapLogic> mapLogic_;

    ///! dataPresentation_ used to deserialize incoming missions that are sent
    ///  as raw binary data
	APDataPresentation<Content, Target> dataPresentation_;

    ///! subscription to incoming ROS sensor data messages
	ros::Subscriber sensorDataSubscriptionRos_;

    ///! subscription to incoming ROS trajectory messages
	ros::Subscriber trajectorySubscriptionRos_;

    ///! subscription to incoming ROS mission messages
    ros::Subscriber missionSubscriptionRos_;

    ///! subscription to incoming ROS PIDStati messages
	ros::Subscriber PIDStatiSubscriptionRos_;

    ///! subscription to incoming ROS local planner status messages
    ros::Subscriber localPlannerDataSubscriptionRos_;

    ///! subscription to incoming ROS power info messages
	ros::Subscriber powerModelSubscriptionRos_;
};

#endif // DATAMANAGER_H
