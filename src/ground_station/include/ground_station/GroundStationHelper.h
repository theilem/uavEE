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
/**
 *   @author Richard Nai, rnai2@illinois.edu
 *   @file GroundStationHelper.h
 *   @date [dd/mm/yyyy] 3/18/2018
 *   @brief UAV Ground Station Helper header file
 */

#ifndef GROUND_STATION_GROUNDSTATIONHELPER_H_
#define GROUND_STATION_GROUNDSTATIONHELPER_H_

#include <uavAP/Core/DataHandling/DataHandling.h>
#include <uavAP/Core/Framework/Helper.h>
#include <uavAP/Core/IDC/IDC.h>
#include <uavAP/Core/Scheduler/SchedulerFactory.h>
#include <uavAP/Core/TimeProvider/TimeProviderFactory.h>

#include "ground_station/ConfigManager.h"
#include "ground_station/DataManager.h"
#include "ground_station/LayoutGenerator.h"
#include "ground_station/MapLogic.h"

class GroundStationHelper: public Helper
{
public:
	GroundStationHelper()
	{
		addCreator<ConfigManager>();
		addCreator<DataHandling>();

		addDefault<SchedulerFactory>();
		addDefault<TimeProviderFactory>();
		addDefaultCreator<IDC>();

		addDefaultCreator<DataManager>();
		addDefaultCreator<LayoutGenerator>();
		addDefaultCreator<MapLogic>();
	}
};

#endif /* GROUND_STATION_GROUNDSTATIONHELPER_H_ */
