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
/**
 * @file xplane_ros_plugin.cpp
 * @author Mirco Theile, mircot@illinois.edu
 * @date [DD.MM.YYYY] 26.3.2018
 * @brief
 */

#include <cstring>
#include <csignal>
#include <ros/ros.h>
#include <uavAP/Core/Runner/SimpleRunner.h>

#include "xPlane/CHeaders/XPLM/XPLMDefs.h"
#include "xPlane/CHeaders/XPLM/XPLMPlugin.h"
#include "xPlane/CHeaders/XPLM/XPLMDisplay.h"
#include "xPlane/CHeaders/XPLM/XPLMGraphics.h"
#include "xPlane/CHeaders/Widgets/XPWidgets.h"
#include "xPlane/CHeaders/Widgets/XPStandardWidgets.h"
#include "xPlane/CHeaders/XPLM/XPLMMenus.h"
#include "xPlane/CHeaders/XPLM/XPLMUtilities.h"
#include "x_plane_interface/XPlaneRosNode.h"
#include "x_plane_interface/XPlaneHelper.h"

static Aggregator* aggregator = nullptr;

void
rosInterfaceHandler(void* mRef, void* iRef);

PLUGIN_API int
XPluginStart(char* outName, char* outSig, char* outDesc)
{
	strcpy(outName, "uavEE");
	strcpy(outSig, "uavee");
	strcpy(outDesc, "uavEE X-Plane Simulation Interface");

	XPLMMenuID id;
	int item;

	item = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "uavEE", NULL, 1);

	id = XPLMCreateMenu("UAVEE", XPLMFindPluginsMenu(), item, rosInterfaceHandler, NULL);

	XPLMAppendMenuItem(id, "Start ROS Node", (void*) "STARTROS", 1);
	XPLMAppendMenuItem(id, "Enable Joystick", (void*) "ENABLEJS", 1);
	XPLMAppendMenuItem(id, "Disable Joystick", (void*) "DISABLEJS", 1);
	XPLMAppendMenuItem(id, "Enable Autopilot", (void*) "ENABLEAP", 1);
	XPLMAppendMenuItem(id, "Disable Autopilot", (void*) "DISABLEAP", 1);

	if (!aggregator)
	{
		XPlaneHelper helper;
		boost::property_tree::ptree config;
		aggregator = new Aggregator;
		*aggregator = helper.createAggregation(config);
	}

	return 1;
}

PLUGIN_API void
XPluginStop(void)
{
	if (aggregator)
	{
		aggregator->cleanUp();
		delete aggregator;
	}
}

PLUGIN_API void
XPluginDisable(void)
{
}

PLUGIN_API int
XPluginEnable(void)
{
	return 1;
}

PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID, intptr_t, void*)
{
}

void
rosInterfaceHandler(void* mRef, void* iRef)
{
	if (!strcmp((char*) iRef, "STARTROS"))
	{
		if (aggregator)
		{
			SimpleRunner runner(*aggregator);
			if (runner.runAllStages())
			{
				APLOG_ERROR << "Running all stages failed.";
				return;
			}
		}
	}

	if (!strcmp((char*) iRef, "ENABLEJS"))
	{
		if (aggregator)
		{
			auto node = aggregator->getOne<XPlaneRosNode>();

			if (!node)
			{
				return;
			}

			node->enableJoystick();
		}
	}

	if (!strcmp((char*) iRef, "DISABLEJS"))
	{
		if (aggregator)
		{
			auto node = aggregator->getOne<XPlaneRosNode>();

			if (!node)
			{
				return;
			}

			node->disableJoystick();
		}
	}

	if (!strcmp((char*) iRef, "ENABLEAP"))
	{
		if (aggregator)
		{
			auto node = aggregator->getOne<XPlaneRosNode>();

			if (!node)
			{
				return;
			}

			node->enableAutopilot();
		}
	}

	if (!strcmp((char*) iRef, "DISABLEAP"))
	{
		if (aggregator)
		{
			auto node = aggregator->getOne<XPlaneRosNode>();

			if (!node)
			{
				return;
			}

			node->disableAutopilot();
		}
	}
}
