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
 * AutopilotLauncher.cpp
 *
 *  Created on: May 26, 2018
 *      Author: sim
 */
#include <autopilot_interface/AutopilotLauncher.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>


AutopilotLauncher::AutopilotLauncher()
{
}

std::shared_ptr<AutopilotLauncher>
AutopilotLauncher::create(const boost::property_tree::ptree& config)
{
	auto al = std::make_shared<AutopilotLauncher>();
	al->configure(config);
	return al;
}

bool
AutopilotLauncher::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	pm.add("watchdog_binary", watchdogBinary_, true);
	pm.add("watchdog_config", watchdogConfig_, true);
	return pm.map();
}

void
AutopilotLauncher::notifyAggregationOnUpdate(const Aggregator& agg)
{
}

AutopilotLauncher::~AutopilotLauncher()
{
	if (watchdogProcess_.running())
	{
		kill(watchdogProcess_.id(), SIGINT);
		watchdogProcess_.join();
	}
}

bool
AutopilotLauncher::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::FINAL:
	{
		watchdogProcess_ = boost::process::child(watchdogBinary_, watchdogConfig_);
		break;
	}
	default:
		break;
	}
	return false;
}
