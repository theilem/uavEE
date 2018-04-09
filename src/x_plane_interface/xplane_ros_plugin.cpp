/**
 * @file xplane_ros_plugin.cpp
 * @author Mirco Theile, mircot@illinois.edu
 * @date [DD.MM.YYYY] 26.3.2018
 * @brief
 */

#include "xPlane/CHeaders/XPLM/XPLMDefs.h"
#include "xPlane/CHeaders/XPLM/XPLMPlugin.h"
#include "xPlane/CHeaders/XPLM/XPLMDisplay.h"
#include "xPlane/CHeaders/XPLM/XPLMGraphics.h"
#include "xPlane/CHeaders/Widgets/XPWidgets.h"
#include "xPlane/CHeaders/Widgets/XPStandardWidgets.h"
#include "xPlane/CHeaders/XPLM/XPLMMenus.h"
#include "xPlane/CHeaders/XPLM/XPLMUtilities.h"
#include <cstring>

#include <ros/ros.h>

#include "x_plane_interface/XPlaneRosNode.h"
#include "x_plane_interface/XPlaneHelper.h"

#include <uavAP/Core/Runner/SimpleRunner.h>

static Aggregator* aggregator = nullptr;

void
rosInterfaceHandler(void* mRef, void* iRef);

PLUGIN_API int
XPluginStart(char* outName, char* outSig, char* outDesc)
{
    // Plugin details
    strcpy(outName, "ROS Plugin");
    strcpy(outSig, "uavee.ros.plugin");
    strcpy(outDesc, "More information not available");

    XPLMMenuID id;
    int item;

    item = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "Ros interface", NULL, 1);

    id = XPLMCreateMenu("ROS", XPLMFindPluginsMenu(), item, rosInterfaceHandler, NULL);

    XPLMAppendMenuItem(id, "Enable/Disable ROS", (void*)"ROS", 1);
    XPLMAppendMenuItem(id, "Enable/Disable Autopilot", (void*)"AP", 1);

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
XPluginReceiveMessage(XPLMPluginID, intptr_t, void* )
{
}

void
rosInterfaceHandler(void* mRef, void* iRef)
{
    if (!strcmp((char*)iRef, "ROS"))
    {
        if (aggregator)
        {
            delete aggregator;
            aggregator = nullptr;
            return;
        }
        XPlaneHelper helper;
        boost::property_tree::ptree config;
        aggregator = new Aggregator;
        *aggregator = helper.createAggregation(config);

        SimpleRunner run(*aggregator);
        run.runAllStages();

    }
    else if (!strcmp((char*)iRef, "AP"))
    {
        if (!aggregator)
        {
            return;
        }
        auto node = aggregator->getOne<XPlaneRosNode>();
        if (!node)
            return;

        node->toggleAutopilot();

    }
}
