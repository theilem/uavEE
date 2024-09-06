/**
 * @file xplane_ros_plugin.cpp
 * @author Mirco Theile, mirco.theile@tum.edu
 * @author Richard Nai, richard.nai@tum.de
 * @date [DD.MM.YYYY] 26.3.2018
 * @brief
 */

#if (GCC_VERSION > 80000) || __APPLE__
#include <filesystem>
namespace filesystem = std::filesystem;
#else

#include <experimental/filesystem>

namespace filesystem = std::experimental::filesystem;
#endif

#include <cpsCore/Logging/CPSLogger.h>
#include <cpsCore/Synchronization/SimpleRunner.h>
#include <cpsCore/Configuration/JsonPopulator.h>



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
	CPSLogger::instance()->setLogLevel(LogLevel::DEBUG);
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

	filesystem::path configDir(path);
	configDir.append("uavEEConfig");

	XPLMMenuID configMenuId = XPLMCreateMenu("Select Config", parentMenu, menuIdx, configSelector, NULL);
	//Using pointer as integer
	intptr_t menuId = 0;
	for (const auto& entry : filesystem::directory_iterator(configDir))
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

void
addCommandToMenu(const char* name, const char* description, int (* func)(XPLMCommandRef, XPLMCommandPhase, void*),
				 int inBefore, void* inRefcon)
{
	registerCommand(rootMenu, name, description, func, inBefore, inRefcon);
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

int
generateConfig(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon)
{
	if (inPhase == xplm_CommandBegin)
	{
		char path[512];
		XPLMGetSystemPath(path);

		filesystem::path configDir(path);
		configDir.append("uavEEConfig").append("generate.json");
		std::ofstream file;
		file.open(configDir.string(), std::ofstream::out);
		JsonPopulator pop(file);

		pop.populateContainer(XPlaneInterfaceHelper());
	}
	return 0;
}

void
populateConfig()
{
	parentIdx = XPLMAppendMenuItem(rootMenu, "Select Config", (void*) "SETCONF", 1);
	configMenu = addDirectoryInfo(rootMenu, parentIdx);
}