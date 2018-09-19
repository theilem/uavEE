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
﻿/**
 *   @author Richard Nai, rnai2@illinois.edu
 *   @file PanTiltHandlerWidgetInterface.cpp
 *   @date [DD/MM/YY] 24/3/2018
 *   @brief
 */

#include "pan_tilt_handler/PanTiltHandlerWidgetInterface.h"

void
PanTiltHandlerWidgetInterface::notifyAggregationOnUpdate(const Aggregator& agg)
{
	dataSignals_.setFromAggregationIfNotSet(agg);
	pidConfigurator_.setFromAggregationIfNotSet(agg);
}

ObjectHandle<IDataSignals>
PanTiltHandlerWidgetInterface::getIDataSignals() const
{
	return dataSignals_;
}

ObjectHandle<MapLogic>
PanTiltHandlerWidgetInterface::getMapLogic() const
{
	return ObjectHandle<MapLogic>(); //NULL!!
}

ObjectHandle<IConfigManager>
PanTiltHandlerWidgetInterface::getConfigManager() const
{
	return ObjectHandle<IConfigManager>(); //NULL!!
}

ObjectHandle<IPIDConfigurator>
PanTiltHandlerWidgetInterface::getPIDConfigurator() const
{
	return pidConfigurator_;
}
