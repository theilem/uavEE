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

#include <filesystem>

#include <cpsCore/Logging/CPSLogger.h>
#include <cpsCore/Synchronization/SimpleRunner.h>

#ifdef UNIX
#define LIN
#endif

#include "xPlane/CHeaders/XPLM/XPLMDefs.h"
#include "xPlane/CHeaders/XPLM/XPLMMenus.h"


#include "uavEE/XPlaneInterface/XPlaneInterface.h"
#include "uavEE/XPlaneInterface/XPlaneInterfaceHelper.h"

#include "uavEE/XPlaneInterface/XPlanePlugin.h"

Aggregator agg;
bool runBegan;
std::string configPath;
XPLMMenuID configMenu;

PLUGIN_API int
XPluginStart(char* outName, char* outSig, char* outDesc)
{
	CPSLogger::instance()->setLogLevel(LogLevel::TRACE);
	CPSLOG_TRACE << "Begin XPlanePlugin";
	strcpy(outName, "uavEE");
	strcpy(outSig, "uavee");
	strcpy(outDesc, "uavEE X-Plane Simulation Interface");

	XPLMMenuID id;
	int item, parentIdx;

	item = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "uavEE", NULL, 1);

	id = XPLMCreateMenu("UAVEE", XPLMFindPluginsMenu(), item, handler, NULL);

	XPLMAppendMenuItem(id, "Start Node", (void*) "STARTNODE", 1);
	XPLMAppendMenuItem(id, "Enable Autopilot", (void*) "ENABLEAP", 1);
	XPLMAppendMenuItem(id, "Disable Autopilot", (void*) "DISABLEAP", 1);
	parentIdx = XPLMAppendMenuItem(id, "Select Config", (void*) "SETCONF", 1);
	registerCommand(id, "Reset Config", "Resets config file path for XPlanePlugin", resetConfig);

//	XPLMCommandRef cmd = XPLMCreateCommand("Reset Config", "Resets config file path for XPlanePlugin");
//	XPLMRegisterCommandHandler(cmd, (XPLMCommandCallback_f) resetConfig, 0, NULL);
//
//	XPLMAppendMenuItemWithCommand(id, "Reset Config", cmd);
	configMenu = addDirectoryInfo(id, parentIdx);

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

int
resetConfig(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon)
{
	if (inPhase == xplm_CommandBegin)
	{
		configPath = std::string();
		std::cout << "Resetting Config\n";
	}
	return 0;
}

//int
//startNode(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon)
//{
//	if (!runBegan && inPhase == xplm_CommandBegin)
//	{
//		CPSLOG_DEBUG << "STARTNODE";
//		if (!configPath.length())
//		{
//			CPSLOG_DEBUG << "No Config Path specified, using default";
//			agg = XPlaneInterfaceHelper::createAggregation();
//		}
//		else
//		{
//			agg = XPlaneInterfaceHelper::createAggregation(configPath);
//		}
//		SimpleRunner runner(agg);
//		if (runner.runAllStages())
//		{
//			CPSLOG_ERROR << "Running all stages failed.";
//			return 0;
//		}
//		runBegan = true;
//	}
//	return 0;
//}
//
//int
//setAutopilotState(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon)
//{
//	if (inPhase == xplm_CommandBegin)
//	{
//		if(auto node = agg.getOne<XPlaneInterface>())
//		{
//			bool command = (bool) inRefcon;
//			if (command)
//			{
//				CPSLOG_DEBUG << "Enabling Autopilot";
//				node->enableAutopilot();
//			} else {
//				CPSLOG_DEBUG << "Enabling Autopilot";
//				node->disableAutopilot();
//			}
//		}
//	}
//	return 0;
//}

XPLMMenuID
addDirectoryInfo(XPLMMenuID parentMenu, int menuIdx)
{
	char path[512];
	XPLMGetSystemPath(path);

	std::filesystem::path configDir(path);
	configDir.append("uavEEConfig");

	XPLMMenuID configMenuId = XPLMCreateMenu("Select Config", parentMenu, menuIdx, configSelector, NULL);
	for (const auto& entry : std::filesystem::directory_iterator(configDir))
	{
		CPSLOG_TRACE << entry.path().string() << "\n";
		XPLMAppendMenuItem(configMenuId, entry.path().string().data(), (void*) entry.path().string().data(), 1);
	}
	return configMenuId;
}

void
configSelector(void* mRef, void* iRef)
{
	configPath = std::string((char*) iRef);
	CPSLOG_DEBUG << "Setting config file path to:" << configPath;
}

void
registerCommand(XPLMMenuID menuID, const char* name, const char* description,
				int (* func)(XPLMCommandRef, XPLMCommandPhase, void*), int inBefore, void* inRefcon)
{
	XPLMCommandRef cmd = XPLMCreateCommand(name, description);
	XPLMRegisterCommandHandler(cmd, (XPLMCommandCallback_f) func, 0, NULL);

	XPLMAppendMenuItemWithCommand(menuID, name, cmd);
}
