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
 * DeviceBridge.cpp
 *
 *  Created on: May 24, 2018
 *      Author: mircot
 */

#include "autopilot_interface/AutopilotInterface/DeviceBridge/DeviceBridge.h"
#include "autopilot_interface/detail/uavAPConversions.h"
#include <uavAP/Core/DataPresentation/Content.h>
#include <uavAP/Core/DataPresentation/IDataPresentation.h>
#include <uavAP/Core/IDC/IDC.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <functional>

DeviceBridge::DeviceBridge()
{
}

DeviceBridge::~DeviceBridge()
{
}

std::shared_ptr<DeviceBridge>
DeviceBridge::create(const boost::property_tree::ptree& config)
{
	auto bridge = std::make_shared<DeviceBridge>();
	bridge->configure(config);
	return bridge;
}

bool
DeviceBridge::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	return pm.map();
}

void
DeviceBridge::notifyAggregationOnUpdate(const Aggregator& agg)
{
	idc_.setFromAggregationIfNotSet(agg);
	dataPresentation_.setFromAggregationIfNotSet(agg);
}

bool
DeviceBridge::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!idc_.isSet())
		{
			APLOG_ERROR << "Device Bridge: IDC missing.";
			return true;
		}
		if (!dataPresentation_.isSet())
		{
			APLOG_ERROR << "Device Bridge: Data presentation missing.";
			return true;
		}
		break;
	}
	case RunStage::NORMAL:
	{
		auto idc = idc_.get();
		sensorDataSender_ = idc->createSender("emulation");
		actuationReceiver_ = idc->subscribeOnPacket("emulation",
				std::bind(&DeviceBridge::onPacket, this, std::placeholders::_1));
		break;
	}
	case RunStage::FINAL:
		break;
	default:
		break;
	}
	return false;
}

void
DeviceBridge::sendSensorData(const SensorData& sd)
{
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "DataPresentation missing. Cannot send sensordata.";
		return;
	}
	auto packet = dp->serialize(sd, Content::SENSOR_DATA);
	sensorDataSender_.sendPacket(packet);
}

boost::signals2::connection
DeviceBridge::subscribeOnControllerOut(const OnControllerOut::slot_type& out)
{
	return onControllerOut_.connect(out);
}

void
DeviceBridge::sendDataSample(const data_sample_t& sample)
{
}

void
DeviceBridge::onPacket(const Packet& packet)
{
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "DataPresentation missing. Cannot handle packet.";
		return;
	}
	Content content;
	auto any = dp->deserialize(packet, content);

	ControllerOutput control;
	if (content == Content::CONTROLLER_OUTPUT)
	{
		control = boost::any_cast<ControllerOutput>(any);
	}
	else
	{
		APLOG_ERROR << "Received invalid packet. Expected ControllerOutput(Light). Received: "
				<< static_cast<int>(content);
		return;
	}

	onControllerOut_(control);
}
