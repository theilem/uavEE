/**
 *   @author Richard Nai, rnai2@illinois.edu
 *   @file PanTiltHandlerWidgetInterface.cpp
 *   @date [DD/MM/YY] 24/3/2018
 *   @brief
 */

#include "pan_tilt_handler/PanTiltHandlerWidgetInterface.h"

void
PanTiltHandlerWidgetInterface::notifyAggregationOnUpdate(Aggregator& agg)
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
