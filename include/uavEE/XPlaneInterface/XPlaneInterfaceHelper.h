//
// Created by seedship on 6/26/20.
//

#ifndef UAVEE_XPLANEINTERFACEHELPER_H
#define UAVEE_XPLANEINTERFACEHELPER_H

#include <cpsCore/Framework/StaticHelper.h>
#include <cpsCore/Utilities/Scheduler/SchedulerFactory.h>
#include <cpsCore/Utilities/TimeProvider/TimeProviderFactory.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/IDC/IDC.h>
#include <cpsCore/Utilities/IDC/NetworkLayer/NetworkFactory.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <uavAP/API/AggregatableAutopilotAPI.h>

#include "uavEE/XPlaneInterface/XPlaneInterface.h"
#include "uavEE/XPlaneInterface/RemoteAutopilotInterface.h"

using XPlaneInterfaceDefaultHelper = StaticHelper<SchedulerFactory,
		TimeProviderFactory,
		DataPresentation,
		IDC,
		IPC,
		XPlaneInterface
		>;

using XPlaneInterfaceHelper = StaticHelper<XPlaneInterfaceDefaultHelper,
		NetworkFactory,
		AggregatableAutopilotAPI,
		RemoteAutopilotInterface
		>;

#endif //UAVEE_XPLANEINTERFACEHELPER_H
