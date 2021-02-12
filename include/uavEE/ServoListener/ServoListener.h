//
// Created by seedship on 2/12/21.
//

#ifndef UAVEE_SERVOLISTENER_H
#define UAVEE_SERVOLISTENER_H

#include <cpsCore/cps_object>

#include "xPlane/CHeaders/XPLM/XPLMDataAccess.h"

class IScheduler;

class ServoListener : public AggregatableObject<IScheduler>, public IRunnableObject
{
public:
	static constexpr TypeId typeId = "servo_listener";

	ServoListener();

	bool
	run(RunStage stage) override;

	void
	startPrinting();

	void
	stopPrinting();

private:
	static constexpr int64_t PERIOD_MS = 100;

	void
	print_();

	Event printEvent_;

	XPLMDataRef yoke_pitch_ratio_;

	XPLMDataRef hstab1_elv1_;
	XPLMDataRef hstab1_elv2_;
	XPLMDataRef hstab2_elv1_;
	XPLMDataRef hstab2_elv2_;

	XPLMDataRef engine_thro_;
	XPLMDataRef engine_thro_use_;
};


#endif //UAVEE_SERVOLISTENER_H
