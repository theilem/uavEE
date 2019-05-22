/*
 * PanTiltHandlerHelper.h
 *
 *  Created on: Feb 25, 2018
 *      Author: seedship
 */

#ifndef PAN_TILT_HANDLER_INCLUDE_PAN_TILT_HANDLER_PANTILTHANDLERHELPER_H_
#define PAN_TILT_HANDLER_INCLUDE_PAN_TILT_HANDLER_PANTILTHANDLERHELPER_H_

#include "pan_tilt_handler/PanTiltHandler.h"
#include "pan_tilt_handler/IMUReader.h"
#include <uavAP/Core/IDC/IDC.h>
#include <uavAP/Core/Scheduler/SchedulerFactory.h>
#include <uavAP/Core/Framework/Helper.h>

class PanTiltHandlerHelper: public Helper
{
public:
    PanTiltHandlerHelper()
    {
		addDefaultCreator<PanTiltHandler>();
		addDefaultCreator<IMUReader>();
		addDefault<SchedulerFactory>();
//		addDefault<IDC>();
    }
};


#endif /* PAN_TILT_HANDLER_INCLUDE_PAN_TILT_HANDLER_PANTILTHANDLERHELPER_H_ */
