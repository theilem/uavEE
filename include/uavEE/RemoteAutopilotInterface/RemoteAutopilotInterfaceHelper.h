//
// Created by seedship on 7/3/20.
//

#ifndef UAVEE_REMOTEAUTOPILOTINTERFACEHELPER_H
#define UAVEE_REMOTEAUTOPILOTINTERFACEHELPER_H

#include <cpsCore/Framework/StaticHelper.h>
#include <cpsCore/Utilities/IDC/IDC.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>

using XPlaneInterfaceDefaultHelper = StaticHelper<SchedulerFactory,
		TimeProviderFactory,
		IPC,
		IDC,
		DataPresentation,>;

using XPlaneInterfaceHelper = StaticHelper<XPlaneInterfaceDefaultHelper>;


#endif //UAVEE_REMOTEAUTOPILOTINTERFACEHELPER_H
