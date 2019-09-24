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
 * SharedMemoryInterface.cpp
 *
 *  Created on: Jul 6, 2018
 *      Author: mircot
 */
#include <autopilot_interface/AutopilotInterface/DirectInterface/DirectInterface.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/SensorData.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>

#include <functional>

std::shared_ptr<DirectInterface>
DirectInterface::create(const Configuration& config)
{
	auto smi = std::make_shared<DirectInterface>();
	smi->configure(config);
	return smi;
}

bool
DirectInterface::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);

	Configuration apiConfig;
	pm.add("api", apiConfig, true);

	uavapAPI_.configure(apiConfig);

	return pm.map();
}

boost::signals2::connection
DirectInterface::subscribeOnControllerOut(const OnControllerOut::slot_type& slot)
{
	return uavapAPI_.subscribeOnControllerOut(slot);
}

void
DirectInterface::sendSensorData(const SensorData& sd)
{
	uavapAPI_.setSensorData(sd);
}

void
DirectInterface::sendDataSample(const data_sample_t& sample)
{
	APLOG_ERROR << "send data sample not implemented.";
}

void
DirectInterface::notifyAggregationOnUpdate(const Aggregator& agg)
{
}

bool
DirectInterface::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::NORMAL:
	{
		APLOG_DEBUG << "Initialize API";
		uavapAPI_.initialize();
		break;
	}
	default:
		break;
	}
	return false;
}
