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
﻿/*
 * PanTiltHandlerHelper.h
 *
 *  Created on: Feb 25, 2018
 *      Author: seedship
 */

#ifndef PAN_TILT_HANDLER_INCLUDE_PAN_TILT_HANDLER_PANTILTHANDLERHELPER_H_
#define PAN_TILT_HANDLER_INCLUDE_PAN_TILT_HANDLER_PANTILTHANDLERHELPER_H_

#include "pan_tilt_handler/PanTiltHandler.h"
#include "pan_tilt_handler/IMUReader.h"
#include <uavAP/Core/Framework/Helper.h>
#include <uavAP/Core/Scheduler/SchedulerFactory.h>
#include <uavAP/Core/TimeProvider/TimeProviderFactory.h>

class PanTiltHandlerHelper: public Helper
{
public:
	PanTiltHandlerHelper()
	{
		addDefaultCreator<PanTiltHandler>();
		addDefaultCreator<IMUReader>();
		addDefault<SchedulerFactory>();
		addDefault<NetworkFactory>();
	}
};

#endif /* PAN_TILT_HANDLER_INCLUDE_PAN_TILT_HANDLER_PANTILTHANDLERHELPER_H_ */
