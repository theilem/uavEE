////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavEE.
//
// uavEE is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavEE is distributed in the hope that it will be useful,
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

#include <uavAP/Core/DataPresentation/Content.h>
#include <uavAP/Core/DataPresentation/DataPresentationFactory.h>
#include <uavAP/Core/Framework/Helper.h>
#include <uavAP/Core/IDC/IDCFactory.h>
#include <uavAP/Core/IPC/IPC.h>
#include <uavAP/Core/Scheduler/SchedulerFactory.h>
#include <uavAP/Core/TimeProvider/TimeProviderFactory.h>

#include "radio_comm/RadioComm.h"
class RadioCommHelper: public Helper
{
public:

	RadioCommHelper()
	{
		addCreator<RadioComm>("radio_comm");

		addDefault<SchedulerFactory>("scheduler");
		addDefault<IDCFactory>("idc");
		addDefault<TimeProviderFactory>("time_provider");
		addDefault<DataPresentationFactory<Content, Target>>("data_presentation");
		addDefaultCreator<IPC>("ipc");
	}
};

#endif /* RADIO_COMM_INCLUDE_RADIO_COMM_RADIOCOMMHELPER_H_ */
