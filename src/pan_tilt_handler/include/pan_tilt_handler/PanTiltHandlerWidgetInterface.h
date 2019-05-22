/**
 *   @author Richard Nai, rnai2@illinois.edu
 *   @file PanTiltHandlerWidgetInterface.h
 *   @date [DD/MM/YY] 24/3/2018
 *   @brief
 */

#ifndef PANTILTHANDLERWIDGETINTERFACE_H
#define PANTILTHANDLERWIDGETINTERFACE_H

#include <ground_station/IWidgetInterface.h>
#include <ground_station/IDataSignals.h>
#include <ground_station/IPIDConfigurator.h>
#include <ground_station/MapLogic.h>
#include <ground_station/IConfigManager.h>

class PanTiltHandlerWidgetInterface : public IWidgetInterface,
    public IAggregatableObject
{
public:

    //PanTiltHandlerWidgetInterface() = default;

    void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

    ObjectHandle<IDataSignals>
    getIDataSignals() const override;

    ObjectHandle<MapLogic>
    getMapLogic() const override;

    ObjectHandle<IConfigManager>
    getConfigManager() const override;

    ObjectHandle<IPIDConfigurator>
    getPIDConfigurator() const override;

private:
    ObjectHandle<IDataSignals> dataSignals_;
    ObjectHandle<IPIDConfigurator> pidConfigurator_;
};

#endif // PANTILTHANDLERWIDGETINTERFACE_H
