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
 * AutopilotInterfaceFactory.h
 *
 *  Created on: May 26, 2018
 *      Author: sim
 */

#ifndef AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_AUTOPILOTINTERFACEFACTORY_H_
#define AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_AUTOPILOTINTERFACEFACTORY_H_
#include <autopilot_interface/AutopilotInterface/ApExtInterface/ApExtInterface.h>
#include <autopilot_interface/AutopilotInterface/DeviceBridge/DeviceBridge.h>
#include <autopilot_interface/AutopilotInterface/DirectInterface/DirectInterface.h>
#include <autopilot_interface/AutopilotInterface/IAutopilotInterface.h>
#include <uavAP/Core/Framework/Factory.h>


class AutopilotInterfaceFactory : public Factory<IAutopilotInterface>
{
public:

	AutopilotInterfaceFactory()
	{
		addCreator<ApExtInterface>();
		addCreator<DeviceBridge>();
		addCreator<DirectInterface>();
	}


};


#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_AUTOPILOTINTERFACEFACTORY_H_ */
