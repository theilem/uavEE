//
// Created by seedship on 2/11/21.
//

#include <cpsCore/Logging/CPSLogger.h>
#include <cpsCore/Synchronization/SimpleRunner.h>

#ifdef UNIX
#define LIN
#endif

#include "xPlane/CHeaders/XPLM/XPLMDefs.h"
#include "xPlane/CHeaders/XPLM/XPLMMenus.h"

#include "uavEE/utils.h"
#include "uavEE/ServoListener/ServoListenerHelper.h"
#include "uavEE/ServoListener/ServoListenerPlugin.h"

Aggregator agg;
bool runBegan = false;

PLUGIN_API int
XPluginStart(char* outName, char* outSig, char* outDesc)
{
	setLogLevel();
	CPSLOG_TRACE << "Begin Servo Logger";
	strcpy(outName, "uavEE Servo Listener");
	strcpy(outSig, "uavee-servolistener");
	strcpy(outDesc, "uavEE X-Plane Servo Listening Tool");

	int item = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "uavEE Servo Listener", NULL, 1);
	XPLMMenuID rootMenu = XPLMCreateMenu("uavEE-servo-listener", XPLMFindPluginsMenu(), item, emptyHandler, NULL);
	registerCommand(rootMenu, "Initialize Plugin", "Initializes the aggregator", startNode);
	registerCommand(rootMenu, "Start Printing", "Starts printing servo data", startPrint);
	registerCommand(rootMenu, "Stop Printing", "Stops printing servo data", stopPrint);
	CPSLOG_TRACE << "End Servo Logger";
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
		agg = ServoListenerHelper::createAggregation();
		SimpleRunner runner(agg);
		if (runner.runAllStages())
		{
			CPSLOG_ERROR << "Running all stages failed.\n";
			return 0;
		}
		CPSLOG_TRACE << "Aggregation Complete";
		runBegan = true;
	}
	return 0;
}

int
startPrint(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void*)
{
	if(inPhase == xplm_CommandBegin)
	{
		if (auto listener = agg.getOne<ServoListener>())
		{
			listener->startPrinting();
		}
		else
		{
			CPSLOG_ERROR << "Missing ServoListener";
		}
	}
	return 0;
}

int
stopPrint(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void*)
{
	if(inPhase == xplm_CommandBegin)
	{
		if (auto listener = agg.getOne<ServoListener>())
		{
			listener->stopPrinting();
		}
		else
		{
			CPSLOG_ERROR << "Missing ServoListener";
		}
	}
	return 0;
}