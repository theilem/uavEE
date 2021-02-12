/**
 * @file xplane_ros_plugin.cpp
 * @author Mirco Theile, mirco.theile@tum.edu
 * @author Richard Nai, richard.nai@tum.de
 * @date [DD.MM.YYYY] 26.3.2018
 * @brief
 */

// Not including filesystem because older GCC compilers cant use it
#include <experimental/filesystem>

#include <cpsCore/Logging/CPSLogger.h>
#include <cpsCore/Synchronization/SimpleRunner.h>
#include <cpsCore/Configuration/JsonPopulator.h>

#ifdef UNIX
#define LIN
#endif

#include "xPlane/CHeaders/XPLM/XPLMDefs.h"
#include "xPlane/CHeaders/XPLM/XPLMMenus.h"


#include "uavEE/utils.h"
#include "uavEE/XPlaneInterface/XPlaneInterface.h"
#include "uavEE/XPlaneInterface/XPlaneInterfaceHelper.h"
#include "uavEE/XPlaneInterface/XPlanePlugin.h"

Aggregator agg;
bool runBegan;
std::string configPath;
std::unordered_map<int, std::string> configMap;

/**
 * See https://developer.x-plane.com/article/developing-plugins/#The_Required_Callbacks-2
 */
PLUGIN_API int
XPluginStart(char* outName, char* outSig, char* outDesc)
{
	setLogLevel();
	CPSLOG_TRACE << "Begin XPlanePlugin";
	strcpy(outName, "uavEE XPlaneInterface");
	strcpy(outSig, "uavee-xplaneinterface");
	strcpy(outDesc, "uavEE X-Plane Simulation Interface");

	int item = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "uavEE", NULL, 1);
	XPLMMenuID rootMenu = XPLMCreateMenu("UAVEE", XPLMFindPluginsMenu(), item, emptyHandler, NULL);

	registerCommand(rootMenu, "Start Node", "Starts the XPlanePlugin", startNode);

	// Generates dummy item
	int configMenuIdx = XPLMAppendMenuItem(rootMenu, "Select Config", NULL, 1);
	// Turns dummy item into a menu
	XPLMMenuID configMenu = XPLMCreateMenu("Select Config Menu", rootMenu, configMenuIdx, configSelector, NULL);
	// Adds config files to that menu
	addDirectoryInfo(configMenu);

	registerCommand(rootMenu, "Enable Autopilot", "Enables the IAutopilotAPI in the XPlanePlugin", setAutopilotState, 0,
					(void*) true);
	registerCommand(rootMenu, "Disable Autopilot", "Disables the IAutopilotAPI in the XPlanePlugin", setAutopilotState,
					0, (void*) false);

	registerCommand(rootMenu, "Start Logging", "Starts logging data", setLoggingState, 0,
					(void*) true);
	registerCommand(rootMenu, "Stop Logging", "Stops logging data", setLoggingState,
					0, (void*) false);

	// Telling refresh config which menu to clear
	registerCommand(rootMenu, "Refresh Config", "Searches config file path for new configs", refreshConfigInfo, 0, configMenu);
	registerCommand(rootMenu, "Generate Config", "Generates a config according to the helper into generate.json",
					generateConfig);

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

int
startNode(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void*)
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

int
setLoggingState(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon)
{
	if (inPhase == xplm_CommandBegin)
	{
		if (auto node = agg.getOne<XPlaneInterface>())
		{
			bool active = (bool) inRefcon;
			node->setLogging(active);
			CPSLOG_DEBUG << "Setting autopilot logging to " << active << "\n";
		}
	}
	return 0;
}

void
addDirectoryInfo(XPLMMenuID &configMenu)
{
	char path[512];
	XPLMGetSystemPath(path);

	std::experimental::filesystem::path configDir(path);
	configDir.append("uavEEConfig");

	//Using pointer as integer
	XPLMAppendMenuItem(configMenu, "No Config", (void*) -1, 1);
	configMap[-1] = "";
	intptr_t menuId = 0;
	for (const auto& entry : std::experimental::filesystem::directory_iterator(configDir))
	{
		CPSLOG_TRACE << "Found: " << entry.path().string() << "\n";
		XPLMAppendMenuItem(configMenu, entry.path().string().data(), (void*) menuId, 1);
		configMap[menuId] = entry.path().string();
		menuId++;
	}
}

void
configSelector(void*, void* iRef)
{
	//Using pointer as integer
	auto menuId = (intptr_t) iRef;
	configPath = configMap[menuId];
	CPSLOG_DEBUG << "Setting config file path to:" << configPath;
}

int
refreshConfigInfo(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon)
{
	if (inPhase == xplm_CommandBegin)
	{
		configMap.clear();
		XPLMClearAllMenuItems(inRefcon);
		addDirectoryInfo(inRefcon);
	}
	return 0;
}

int
generateConfig(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void*)
{
	if (inPhase == xplm_CommandBegin)
	{
		char path[512];
		XPLMGetSystemPath(path);

		std::experimental::filesystem::path configDir(path);
		configDir.append("uavEEConfig").append("generate.json");
		std::ofstream file;
		file.open(configDir.string(), std::ofstream::out);
		JsonPopulator pop(file);

		pop.populateContainer(XPlaneInterfaceHelper());
	}
	return 0;
}