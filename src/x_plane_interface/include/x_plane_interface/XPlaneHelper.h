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
 * @file XPlaneHelper.h
 * @author Mirco Theile, mircot@illinois.edu
 * @date [DD.MM.YYYY] 27.3.2018
 * @brief
 */
#ifndef XPLANEHELPER_H
#define XPLANEHELPER_H

#include <uavAP/Core/Framework/Helper.h>
#include <uavAP/Core/Scheduler/SchedulerFactory.h>
#include <uavAP/Core/TimeProvider/TimeProviderFactory.h>

#include "x_plane_interface/XPlaneRosNode.h"

class XPlaneHelper : public Helper
{
public:
    XPlaneHelper()
    {
        addDefault<SchedulerFactory>("scheduler");
        addDefault<TimeProviderFactory>("time_provider");
        addDefaultCreator<XPlaneRosNode>("ros_node");
    }
};

#endif // XPLANEHELPER_H
