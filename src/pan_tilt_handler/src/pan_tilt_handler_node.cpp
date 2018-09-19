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
﻿/*
 * pan_tilt_handler_node.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: seedship
 */

#include <ros/ros.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/Runner/SimpleRunner.h>
#include "pan_tilt_handler/PanTiltHandlerHelper.h"

int
main(int argc, char** argv)
{
	APLogger::instance()->setLogLevel(LogLevel::DEBUG);
	APLogger::instance()->setModuleName("AntennaHandler");
	ros::init(argc, argv, "pan_tilt_handler_node");
	ros::NodeHandle node;

	std::string IMUPath, arduinoPath;

	if (!node.getParam("/pan_tilt_handler_node/imu_path", IMUPath))
	{
		APLOG_ERROR << "PanTiltHandler: No IMU path set!";
		return 1;
	}

	if (!node.getParam("/pan_tilt_handler_node/arduino_path", arduinoPath))
	{
		APLOG_ERROR << "PanTiltHandler: No Arduino Path set!";
		return 1;
	}

	boost::property_tree::ptree config;
	boost::property_tree::ptree panTiltConfig;
	panTiltConfig.add("imu_path", IMUPath);
	panTiltConfig.add("arduino_path", arduinoPath);
	config.add_child("pan_tilt_handler", panTiltConfig);

	PanTiltHandlerHelper helper;

	Aggregator aggregator = helper.createAggregation(config);
	SimpleRunner run(aggregator);

	if (run.runAllStages())
	{
		APLOG_ERROR << "Run all stages failed.";
		return 1;
	}

	ros::Rate loopRate(1000);
	while (ros::ok())
	{
		ros::spinOnce();
		loopRate.sleep();
	}
	return 0;
}
