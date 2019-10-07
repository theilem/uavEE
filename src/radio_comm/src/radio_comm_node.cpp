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
 * radio_comm_node.cpp
 *
 *  Created on: Dec 9, 2017
 *      Author: mircot
 */
#include <ros/ros.h>
#include <uavAP/Core/Runner/SimpleRunner.h>

#include "radio_comm/RadioCommHelper.h"

int
main(int argc, char** argv)
{
	APLogger::instance()->setLogLevel(LogLevel::DEBUG);
	APLogger::instance()->setModuleName("RadioComm");
	ros::init(argc, argv, "radio_comm");

	ros::NodeHandle nh;

	std::string config;
	nh.getParam("/radio_comm_node/config_path", config);

	RadioCommHelper helper;

	Aggregator aggregator = helper.createAggregation(config);
	SimpleRunner run(aggregator);

	auto sched = aggregator.getOne<IScheduler>();
	sched->setMainThread();

	if (run.runAllStages())
	{
		APLOG_ERROR << "Run all stages failed.";
		return 1;
	}

	sched->schedule(ros::spinOnce, Milliseconds(0), Milliseconds(1));

//	ros::Rate loopRate(1000);
//	while (ros::ok())
//	{
//		ros::spinOnce();
//		loopRate.sleep();
//	}

	sched->startSchedule();


	//Terminated -> Cleanup
	aggregator.cleanUp();
	return 0;

}
