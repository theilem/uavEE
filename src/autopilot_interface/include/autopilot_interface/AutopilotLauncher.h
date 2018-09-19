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
 * AutopilotLauncher.h
 *
 *  Created on: May 26, 2018
 *      Author: sim
 */

#ifndef AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTLAUNCHER_H_
#define AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTLAUNCHER_H_
#include <boost/process/child.hpp>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Runner/IRunnableObject.h>

#include <memory>

#include <boost/property_tree/ptree.hpp>

class AutopilotLauncher: public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "autopilot_launcher";

	AutopilotLauncher();

	~AutopilotLauncher();

	static std::shared_ptr<AutopilotLauncher>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

private:

	std::string watchdogBinary_;
	std::string watchdogConfig_;

	boost::process::child watchdogProcess_;
};



#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTLAUNCHER_H_ */
