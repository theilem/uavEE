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
 * AutopilotInterface.h
 *
 *  Created on: May 25, 2018
 *      Author: mircot
 */

#ifndef AUTOPILOTINTERFACE_H_
#define AUTOPILOTINTERFACE_H_

#include <autopilot_interface/AutopilotInterface/IAutopilotInterface.h>
#include <boost/process/child.hpp>
#include <boost/property_tree/ptree.hpp>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Runner/IRunnableObject.h>

#include <string>
#include <memory>

#include <boost/signals2.hpp>
#include <uavAP/API/ChannelMixing.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>

class ApExtInterface: public IAutopilotInterface, public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "ap_ext";

	ApExtInterface();

	~ApExtInterface();

	static std::shared_ptr<ApExtInterface>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

	boost::signals2::connection
	subscribeOnControllerOut(const OnControllerOut::slot_type& out) override;

	void
	sendSensorData(const SensorData& sd) override;

	void
	sendDataSample(const data_sample_t& sample) override;

private:

	void
	getActuation();

	unsigned int numChannels_;
	uint32_t lastSequenceNr_;

	std::string alvoloConfig_;
	ChannelMixing channelMixing_;
	bool internalImu_;
	bool externalGps_;
	bool useAirspeed_;
	bool traceSeqNr_;

	data_sample_t dataSample_;

	OnControllerOut onControllerOut_;

	data_sample_t lastSample_;

};

#endif /* AUTOPILOTINTERFACE_H_ */
