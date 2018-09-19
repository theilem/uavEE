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
 * power_modeling_node.cpp
 *
 *  Created on: Jan 5, 2018
 *      Author: mircot
 */
#include <boost/property_tree/ptree.hpp>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <simulation_interface/sensor_data.h>

#include "power_modeling/PowerModel.h"
#include "power_modeling/PowerModelingHelper.h"

int
main(int argc, char** argv)
{
	APLogger::instance()->setLogLevel(LogLevel::DEBUG);
	APLogger::instance()->setModuleName("power_modeling");
	ros::init(argc, argv, "power_model");

	ros::NodeHandle nh;

	PowerModel powerModel;

	if (!powerModel.configure())
	{
		return 1;
	}

	ros::Rate rate(400);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;

}
