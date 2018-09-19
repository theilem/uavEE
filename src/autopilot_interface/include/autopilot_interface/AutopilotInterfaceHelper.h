////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavEE.
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
/*
 * AutopilotInterfaceHelper.h
 *
 *  Created on: May 18, 2018
 *      Author: mircot
 */

#ifndef AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACEHELPER_H_
#define AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACEHELPER_H_


#include <uavAP/Core/DataPresentation/ContentMapping.h>
#include <uavAP/Core/DataPresentation/DataPresentationFactory.h>
#include <uavAP/Core/Framework/Helper.h>
#include <uavAP/Core/IDC/IDC.h>
#include <uavAP/Core/IDC/NetworkLayer/NetworkFactory.h>
#include <uavAP/Core/IPC/IPC.h>
#include <uavAP/Core/Scheduler/SchedulerFactory.h>
#include <uavAP/Core/TimeProvider/TimeProviderFactory.h>

#include <autopilot_interface/APRosInterface.h>
#include <autopilot_interface/AutopilotInterface/AutopilotInterfaceFactory.h>
#include <autopilot_interface/AutopilotLauncher.h>
#include <autopilot_interface/LogParser.h>


class AutopilotInterfaceHelper: public Helper
{
public:

	AutopilotInterfaceHelper()
	{
		addDefault<TimeProviderFactory>();
		addDefault<SchedulerFactory>();
		addDefault<DataPresentationFactory<Content, Target>>();
		addFactory<NetworkFactory>();
		addCreator<IDC>();

		addFactory<AutopilotInterfaceFactory>();
		addCreator<APRosInterface>();
		addCreator<LogParser>();
		addCreator<AutopilotLauncher>();
	}
};

#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACEHELPER_H_ */
