//
// Created by seedship on 2/12/21.
//

#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include <cpsCore/Logging/CPSLogger.h>
#include <iostream>

#include "uavEE/ServoListener/ServoListener.h"

ServoListener::ServoListener() :
		yoke_pitch_ratio_(XPLMFindDataRef("sim/joystick/yoke_pitch_ratio")),
		hstab1_elv1_(XPLMFindDataRef("sim/flightmodel/controls/hstab1_elv1def")),
		hstab1_elv2_(XPLMFindDataRef("sim/flightmodel/controls/hstab1_elv2def")),
		hstab2_elv1_(XPLMFindDataRef("sim/flightmodel/controls/hstab2_elv1def")),
		hstab2_elv2_(XPLMFindDataRef("sim/flightmodel/controls/hstab2_elv2def")),
		engine_thro_(XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro")),
		engine_thro_use_(XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro_use"))
{
}

bool
ServoListener::run(RunStage stage)
{
	std::cout << "Calling Runstages\n";
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSetAll())
			{
				CPSLOG_ERROR << "Missing Dependencies.";
				return true;
			}
			break;
		}
		default:
		{
			break;
		}
	}
	return false;
}

void
ServoListener::startPrinting()
{
	if(!printEvent_.isCanceled())
	{
		CPSLOG_WARN << "Already printing";
		return;
	}
	if(!checkIsSet<IScheduler>())
	{
		CPSLOG_WARN << "Missing Scheduler";
		return;
	}
	printEvent_ = get<IScheduler>()->schedule([this]{print_();}, Milliseconds(0), Milliseconds(PERIOD_MS));
	CPSLOG_TRACE << "Scheduled Printing";
}

void
ServoListener::stopPrinting()
{
	if(printEvent_.isCanceled())
	{
		CPSLOG_WARN << "Already not printing";
		return;
	}
	printEvent_.cancel();
	CPSLOG_TRACE << "Descheduled Printing";
}

void
ServoListener::print_()
{
	std::cout << "\n";
	std::cout << "Yoke Pitch: " << static_cast<FloatingType>(XPLMGetDataf(yoke_pitch_ratio_)) << "\n";
	std::cout << "Horizontal Stabilizer 1 Elevator 1: " << static_cast<FloatingType>(XPLMGetDataf(hstab1_elv1_)) << "\n";
	std::cout << "Horizontal Stabilizer 1 Elevator 2: " << static_cast<FloatingType>(XPLMGetDataf(hstab1_elv2_)) << "\n";
	std::cout << "Horizontal Stabilizer 2 Elevator 1: " << static_cast<FloatingType>(XPLMGetDataf(hstab2_elv1_)) << "\n";
	std::cout << "Horizontal Stabilizer 2 Elevator 2: " << static_cast<FloatingType>(XPLMGetDataf(hstab2_elv2_)) << "\n";

	float throttle[8];

	XPLMGetDatavf(engine_thro_, throttle, 0, 8);
	std::cout << "Engine Throttle %: [";
	for (unsigned idx = 0; idx < 7; idx++) {
		std:: cout << throttle[idx] << ", ";
	}
	std::cout << throttle[7] << "]\n";

	XPLMGetDatavf(engine_thro_use_, throttle, 0, 8);
	std::cout << "Engine Throttle % (User Override): [";
	for (unsigned idx = 0; idx < 7; idx++) {
		std:: cout << throttle[idx] << ", ";
	}
	std::cout << throttle[7] << "]\n";
}
