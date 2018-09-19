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
 * SimulationInterfaceHelper.h
 *
 *  Created on: Nov 8, 2017
 *      Author: mircot
 */

#ifndef SIMULATION_INTERFACE_SIMULATIONINTERFACEHELPER_H_
#define SIMULATION_INTERFACE_SIMULATIONINTERFACEHELPER_H_

#include <uavAP/Core/DataPresentation/ContentMapping.h>
#include <uavAP/Core/DataPresentation/DataPresentationFactory.h>
#include <uavAP/API/ChannelMixing.h>
#include "simulation_interface/SimulationConnector.h"
#include <uavAP/Core/Framework/Helper.h>
#include <uavAP/Core/Scheduler/SchedulerFactory.h>
#include <uavAP/Core/TimeProvider/TimeProviderFactory.h>
#include <uavAP/Core/IPC/IPC.h>
#include <uavAP/Core/IDC/NetworkLayer/NetworkFactory.h>
#include <uavAP/Core/IDC/IDC.h>

class SimulationInterfaceHelper: public Helper
{
public:
	SimulationInterfaceHelper()
	{
		addCreator<SimulationConnector>();

		addDefault<SchedulerFactory>();
		addDefault<TimeProviderFactory>();
		addDefaultCreator<IDC>();
		addDefaultCreator<IPC>();
		addDefault<DataPresentationFactory<Content, Target>>();
		addFactory<NetworkFactory>();
	}
};

#endif /* SIMULATION_INTERFACE_SIMULATIONINTERFACEHELPER_H_ */
