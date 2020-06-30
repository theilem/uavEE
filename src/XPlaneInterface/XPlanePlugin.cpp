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
 * @author Mirco Theile, mirco.theile@tum.edu
 * @author Richard Nai, richard.nai@tum.de
 * @date [DD.MM.YYYY] 26.3.2018
 * @brief
 */

#include <cstring>
#include <csignal>

#include <cpsCore/Logging/CPSLogger.h>

#ifdef UNIX
#define LIN
#endif

#include <cpsCore/Synchronization/SimpleRunner.h>

#include "xPlane/CHeaders/XPLM/XPLMDefs.h"
#include "xPlane/CHeaders/XPLM/XPLMMenus.h"


#include "uavEE/XPlaneInterface/XPlaneInterface.h"
#include "uavEE/XPlaneInterface/XPlaneInterfaceHelper.h"


void
handler(void* mRef, void* iRef);

Aggregator agg;
bool runBegan;

PLUGIN_API int
XPluginStart(char* outName, char* outSig, char* outDesc)
{
	CPSLogger::instance()->setLogLevel(LogLevel::TRACE);
	CPSLOG_TRACE << "Begin XPlanePlugin";
	strcpy(outName, "uavEE");
	strcpy(outSig, "uavee");
	strcpy(outDesc, "uavEE X-Plane Simulation Interface");

	XPLMMenuID id;
	int item;

	item = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "uavEE", NULL, 1);

	id = XPLMCreateMenu("UAVEE", XPLMFindPluginsMenu(), item, handler, NULL);

	XPLMAppendMenuItem(id, "Start Node", (void*) "STARTNODE", 1);
	XPLMAppendMenuItem(id, "Enable Autopilot", (void*) "ENABLEAP", 1);
	XPLMAppendMenuItem(id, "Disable Autopilot", (void*) "DISABLEAP", 1);

	CPSLOG_TRACE << "End XPlanePlugin";
	return 1;
}

PLUGIN_API void
XPluginStop(void)
{
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
handler(void* mRef, void* iRef)
{
	CPSLogger::instance()->setLogLevel(LogLevel::TRACE);
	if (!strcmp((char*) iRef, "STARTNODE"))
	{
		CPSLOG_DEBUG << "STARTNODE\n";
		if (!runBegan)
		{
			Configuration c;
			agg = XPlaneInterfaceHelper::createAggregation(c);
			SimpleRunner runner(agg);
			if (runner.runAllStages())
			{
				CPSLOG_ERROR << "Running all stages failed.";
				return;
			}
			runBegan = true;
		}
	}
	else if (!strcmp((char*) iRef, "ENABLEAP"))
	{
		CPSLOG_DEBUG << "ENABLEAP\n";
		auto node = agg.getOne<XPlaneInterface>();
		node->enableAutopilot();
	}
	else if (!strcmp((char*) iRef, "DISABLEAP"))
	{
		CPSLOG_DEBUG << "DISABLEAP\n";
		auto node = agg.getOne<XPlaneInterface>();
		node->disableAutopilot();
	}
	else
	{
		CPSLOG_WARN << "Unknown iRef in XPlanePlugin " << __FUNCTION__ << ". Line:" << __LINE__;
		CPSLOG_WARN << "iRef was: " << (char*) iRef;
	}
}