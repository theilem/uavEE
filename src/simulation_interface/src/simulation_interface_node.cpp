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
/*
 * simulation_interface_node.cpp
 *
 *  Created on: Nov 1, 2017
 *      Author: mircot
 */

#include <boost/property_tree/json_parser.hpp>
#include <simulation_interface/sensor_data.h>
#include <simulation_interface/codec/Codec.h>
#include <boost/thread/thread_time.hpp>
#include <ros/ros.h>
#include <SimulationInterfaceHelper.h>
#include <string>

#include <uavAP/Core/Runner/SimpleRunner.h>

int
main(int argc, char** argv)
{
	APLogger::instance()->setLogLevel(LogLevel::WARN);
	APLogger::instance()->setModuleName("SimulationInterface");
	ros::init(argc, argv, "simulation_interface");

	auto nodeHandle = std::make_shared<ros::NodeHandle>();

	std::string simulationConfigPath;
	if (!nodeHandle->getParam("/simulation_interface_node/simulation_config", simulationConfigPath))
	{
		APLOG_ERROR << "Alvolo config path missing";
		return 1;
	}

	ros::Rate loopRate(10000);
	SimulationInterfaceHelper helper;
	Aggregator aggregator = helper.createAggregation(simulationConfigPath);
	SimpleRunner run(aggregator);

	auto con = aggregator.getOne<SimulationConnector>();
	con->setNodeHandle(nodeHandle);

	if (run.runAllStages())
	{
		APLOG_ERROR << "Run all stages failed.";
		return 1;
	}

	while (ros::ok())
	{
		ros::spinOnce();
		loopRate.sleep();
	}
	return 0;

}

