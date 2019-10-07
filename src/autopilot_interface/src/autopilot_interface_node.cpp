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
 * autopilot_interface_node.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: mircot
 */

#include "autopilot_interface/AutopilotInterfaceHelper.h"
#include <ros/ros.h>
#include <string>
#include <boost/process.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/Object/SignalHandler.h>
#include <uavAP/Core/Runner/SimpleRunner.h>

int
main(int argc, char** argv)
{
	APLogger::instance()->setLogLevel(LogLevel::DEBUG);
	APLogger::instance()->setModuleName("AutopilotInterface");

	ros::init(argc, argv, "autopilot_interface_node");
	ros::NodeHandle node;

	std::string config;
	if (!node.getParam("/autopilot_interface_node/config_path", config))
	{
		APLOG_ERROR << "Config path missing.";
		return 1;
	}

	AutopilotInterfaceHelper helper;

	APRosInterface apInterface;
	ros::Rate loopRate(10000);

	Aggregator aggregator = helper.createAggregation(config);
	SimpleRunner run(aggregator);

	auto sh = aggregator.getOne<SignalHandler>();

	if (run.runAllStages())
	{
		APLOG_ERROR << "Run all stages failed.";
		return 2;
	}

	sh->subscribeOnSigint(std::bind(ros::shutdown));

	while (ros::ok())
	{
		ros::spinOnce();
		loopRate.sleep();
	}

	//Terminated -> Cleanup
	aggregator.cleanUp();
	return 0;

	return 0;
}
