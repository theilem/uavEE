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
 * autopilot_interface_node.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: mircot
 */

#include "autopilot_interface/AutopilotInterface.h"
#include <ros/ros.h>
#include <string>
#include <boost/process.hpp>
#include <boost/property_tree/json_parser.hpp>

int
main(int argc, char** argv)
{
	APLogger::instance()->setLogLevel(LogLevel::TRACE);
	APLogger::instance()->setModuleName("AutopilotInterface");

	ros::init(argc, argv, "autopilot_interface_node");
	ros::NodeHandle node;

	std::string serialPort;
	bool deviceBridge = false;
	if (node.getParam("/autopilot_interface_node/serial_port", serialPort))
	{
		deviceBridge = true;
	}

	AutopilotInterface apInterface;
	ros::Rate loopRate(10000);

	if (deviceBridge)
	{
		apInterface.setDeviceBridge(serialPort);
		APLOG_DEBUG << "Device bridge set up.";


		while (ros::ok())
		{
			ros::spinOnce();
			loopRate.sleep();
		}
	}
	else
	{
		std::string watchdogConfig, watchdogBinary, apExtPath;
		if (!node.getParam("/autopilot_interface_node/watchdog_config", watchdogConfig)
				|| !node.getParam("/autopilot_interface_node/watchdog_binary", watchdogBinary))
		{
			APLOG_ERROR << "Watchdog config and binary missing";
			return 1;
		}

		std::string alvoloConfigPath;
		if (!node.getParam("/autopilot_interface_node/alvolo_config", alvoloConfigPath))
		{
			APLOG_ERROR << "Alvolo config path missing";
			return 1;
		}
		boost::property_tree::ptree alvoloConfig;
		boost::property_tree::read_json(alvoloConfigPath, alvoloConfig);
		if (!apInterface.configure(alvoloConfig))
		{
			APLOG_ERROR << "ApExternal configuration failed.";
			return 1;
		}

		if (!apInterface.startApExt())
		{
			APLOG_ERROR << "ApExternal did not start successfully.";
			return 1;
		}
		APLOG_DEBUG << "Launching watchdog: " << watchdogBinary << "With config: " << watchdogConfig;
		boost::process::child watchdog(watchdogBinary, watchdogConfig);

		while (ros::ok())
		{
			ros::spinOnce();
			loopRate.sleep();
		}

		kill(watchdog.id(), SIGINT);
		watchdog.join();
	}

	return 0;
}

