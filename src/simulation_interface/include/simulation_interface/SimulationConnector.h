////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/*
 * SimulationConnector.h
 *
 *  Created on: Jul 28, 2017
 *      Author: mircot
 */

#ifndef UAVAP_SIMULATION_SIMULATIONCONNECTOR_H_
#define UAVAP_SIMULATION_SIMULATIONCONNECTOR_H_
#include <boost/property_tree/ptree.hpp>
#include <boost/signals2.hpp>
#include <uavAP/Core/IDC/IDCSender.h>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/Core/Time.h>

#include "simulation_interface/actuation.h"
#include "simulation_interface/sensor_data.h"

#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <uavAP/Core/DataPresentation/Content.h>
#include <uavAP/Core/IDC/IDCSender.h>
#include <atomic>
#include <memory>
#include <mutex>

class IScheduler;
class ITimeProvider;
class DataPresentation;
class SensorData;
class IDC;

class SimulationConnector: public IRunnableObject, public IAggregatableObject
{
public:

	static constexpr TypeId typeId = "simulation_connector";

	SimulationConnector();

	~SimulationConnector();

	static std::shared_ptr<SimulationConnector>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	bool
	run(RunStage stage) override;

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

private:

	void
	actuate(const simulation_interface::actuation& out);

	void
	sense(const Packet& packet);

	void
	sendSensorData(simulation_interface::sensor_data data);

	void
	handleSimDelay(const simulation_interface::sensor_data& data);

	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<ITimeProvider> timeProvider_;
	ObjectHandle<IDC> idc_;
	ObjectHandle<DataPresentation> dataPresentation_;

	ros::Publisher sensorPublisher_;
	ros::Publisher delayPublisher_;
	ros::Subscriber actuationSubscriber_;

	IDCSender actuationSender_;
	boost::signals2::connection sensorReceiver_;

	uint32_t lastSequenceNr_;
	std::mutex timePointsMutex_;
	std::map<uint32_t, TimePoint> timepoints_;

	bool correctSimDelay_;
	int correctionCounter_;
	TimePoint correctionInit_;
};

#endif /* UAVAP_SIMULATION_SIMULATIONCONNECTOR_H_ */
