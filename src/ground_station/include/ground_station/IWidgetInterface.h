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
 *   @authors Mirco Theile, mircot@illinois.edu; Richard Nai, rnai2@illinois.edu
 *   @file IWidgetInterface.h
 *   @date [dd/mm/yyyy] 18/1/2018
 *   @brief UAV Ground Station Widget Interface
 */

#ifndef GROUND_STATION_INCLUDE_GROUND_STATION_IWIDGETINTERFACE_H_
#define GROUND_STATION_INCLUDE_GROUND_STATION_IWIDGETINTERFACE_H_

#include <uavAP/Core/Object/ObjectHandle.h>

class IDataSignals;
class MapLogic;
class ConfigManager;
class IPIDConfigurator;
class IConfigManager;

/**
 * @brief   The IWidgetInterface class is an interface all classes that manage
 *          widgets must conform to. It must be able to give an IDataSignals,
 *          MapLogic, and ConfigManager
 */
class IWidgetInterface
{
public:

	virtual
	~IWidgetInterface()
	{
	}

	virtual ObjectHandle<IDataSignals>
	getIDataSignals() const = 0;

	virtual ObjectHandle<MapLogic>
	getMapLogic() const = 0;

	virtual ObjectHandle<IConfigManager>
	getConfigManager() const = 0;

	virtual ObjectHandle<IPIDConfigurator>
	getPIDConfigurator() const = 0;

};

#endif /* GROUND_STATION_INCLUDE_GROUND_STATION_IWIDGETINTERFACE_H_ */
