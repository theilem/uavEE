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
 * SharedMemoryInterface.h
 *
 *  Created on: Jul 6, 2018
 *      Author: mircot
 */

#ifndef AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_DIRECTINTERFACE_DIRECTINTERFACE_H_
#define AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_DIRECTINTERFACE_DIRECTINTERFACE_H_
#include <autopilot_interface/AutopilotInterface/IAutopilotInterface.h>
#include <boost/property_tree/ptree.hpp>
#include <uavAP/Core/IPC/Publisher.h>
#include <uavAP/Core/IPC/Subscription.h>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/API/AutopilotAPI.hpp>

#include <memory>

class DirectInterface: public IAutopilotInterface,
		public IAggregatableObject,
		public IRunnableObject
{

public:

	static constexpr TypeId typeId = "direct";

	DirectInterface() = default;

	~DirectInterface() = default;

	static std::shared_ptr<DirectInterface>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	boost::signals2::connection
	subscribeOnControllerOut(const OnControllerOut::slot_type& slot) override;

	void
	sendSensorData(const SensorData& sd) override;

	void
	sendDataSample(const data_sample_t& sample) override;

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

private:

	AutopilotAPI uavapAPI_;

};

#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_DIRECTINTERFACE_DIRECTINTERFACE_H_ */
