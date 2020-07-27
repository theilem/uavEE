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
XPLMMenuID configMenu, rootMenu;
std::unordered_map<int, std::string> configMap;
int parentIdx;

PLUGIN_API int
XPluginStart(char* outName, char* outSig, char* outDesc)
{
	CPSLogger::instance()->setLogLevel(LogLevel::TRACE);
	CPSLOG_TRACE << "Begin XPlanePlugin";
	strcpy(outName, "uavEE");
	strcpy(outSig, "uavee");
	strcpy(outDesc, "uavEE X-Plane Simulation Interface");

	int item;

	item = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "uavEE", NULL, 1);

	rootMenu = XPLMCreateMenu("UAVEE", XPLMFindPluginsMenu(), item, handler, NULL);

	registerCommand(rootMenu, "Start Node", "Starts the XPlanePlugin", startNode);
	registerCommand(rootMenu, "Enable Autopilot", "Enables the IAutopilotAPI in the XPlanePlugin", setAutopilotState, 0,
					(void*) true);
	registerCommand(rootMenu, "Disable Autopilot", "Disables the IAutopilotAPI in the XPlanePlugin", setAutopilotState,
					0, (void*) false);

	populateConfig();

	registerCommand(rootMenu, "Reset Config", "Resets config file path for XPlanePlugin", resetConfig);
	registerCommand(rootMenu, "Refresh Config", "Searches config file path for new configs", refreshConfigInfo);


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

int
startNode(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon)
{
	if (!runBegan && inPhase == xplm_CommandBegin)
	{
		CPSLOG_DEBUG << "STARTNODE";
		if (!configPath.length())
		{
			CPSLOG_DEBUG << "No Config Path specified, using default";
			agg = XPlaneInterfaceHelper::createAggregation();
		}
		else
		{
			agg = XPlaneInterfaceHelper::createAggregation(configPath);
		}
		SimpleRunner runner(agg);
		if (runner.runAllStages())
		{
			CPSLOG_ERROR << "Running all stages failed.";
			return 0;
		}
		runBegan = true;
	}
	return 0;
}

int
setAutopilotState(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon)
{
	if (inPhase == xplm_CommandBegin)
	{
		if (auto node = agg.getOne<XPlaneInterface>())
		{
			bool active = (bool) inRefcon;
			node->setAutopilotActive(active);
			CPSLOG_DEBUG << "Setting autopilot active to " << active << "\n";
		}
	}
	return 0;
}

XPLMMenuID
addDirectoryInfo(XPLMMenuID parentMenu, int menuIdx)
{
	char path[512];
	XPLMGetSystemPath(path);

	std::filesystem::path configDir(path);
	configDir.append("uavEEConfig");

	XPLMMenuID configMenuId = XPLMCreateMenu("Select Config", parentMenu, menuIdx, configSelector, NULL);
	//Using pointer as integer
	intptr_t menuId = 0;
	for (const auto& entry : std::filesystem::directory_iterator(configDir))
	{
		CPSLOG_TRACE << "Found: " << entry.path().string() << "\n";
		XPLMAppendMenuItem(configMenuId, entry.path().string().data(), (void*) menuId, 1);
		configMap[menuId] = entry.path().string();
		menuId++;
	}
	return configMenuId;
}

void
configSelector(void* mRef, void* iRef)
{
	//Using pointer as integer
	auto menuId = (intptr_t) iRef;
	configPath = configMap[menuId];
	CPSLOG_DEBUG << "Setting config file path to:" << configPath;
}

void
registerCommand(XPLMMenuID menuID, const char* name, const char* description,
				int (* func)(XPLMCommandRef, XPLMCommandPhase, void*), int inBefore, void* inRefcon)
{
	XPLMCommandRef cmd = XPLMCreateCommand(name, description);
	XPLMRegisterCommandHandler(cmd, (XPLMCommandCallback_f) func, inBefore, inRefcon);

	XPLMAppendMenuItemWithCommand(menuID, name, cmd);
}

int
refreshConfigInfo(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon)
{
	if (inPhase == xplm_CommandBegin)
	{
		configMap.clear();
		XPLMRemoveMenuItem(rootMenu, parentIdx);
		populateConfig();
	}
	return 0;
}

void
populateConfig()
{
	parentIdx = XPLMAppendMenuItem(rootMenu, "Select Config", (void*) "SETCONF", 1);
	configMenu = addDirectoryInfo(rootMenu, parentIdx);
}