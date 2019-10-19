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
/*
 * RadioCommHelper.h
 *
 *  Created on: Dec 9, 2017
 *      Author: mircot
 */

#ifndef RADIO_COMM_INCLUDE_RADIO_COMM_RADIOCOMMHELPER_H_
#define RADIO_COMM_INCLUDE_RADIO_COMM_RADIOCOMMHELPER_H_

#include <uavAP/Core/DataPresentation/DataPresentation.h>
#include <uavAP/Core/Framework/Helper.h>
#include <uavAP/Core/IDC/IDC.h>
#include <uavAP/Core/IDC/NetworkLayer/NetworkFactory.h>
#include <uavAP/Core/IPC/IPC.h>
#include <uavAP/Core/Scheduler/SchedulerFactory.h>
#include <uavAP/Core/TimeProvider/TimeProviderFactory.h>

#include "radio_comm/RadioComm.h"
class RadioCommHelper: public Helper
{
public:

	RadioCommHelper()
	{
		addDefaultCreator<RadioComm>();

		addDefault<SchedulerFactory>();
		addDefaultCreator<IDC>();
		addDefault<TimeProviderFactory>();
		addConfigurable<DataPresentation>();
		addDefaultConfigurable<IPC>();
		addFactory<NetworkFactory>();
	}
};

#endif /* RADIO_COMM_INCLUDE_RADIO_COMM_RADIOCOMMHELPER_H_ */
