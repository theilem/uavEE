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

	XPLMDataRef pitch_ratio_;
	XPLMDataRef roll_ratio_;
	XPLMDataRef heading_ratio_;

	// Elevator Values
	XPLMDataRef hstab1_elv1_;
	XPLMDataRef hstab1_elv2_;
	XPLMDataRef hstab2_elv1_;
	XPLMDataRef hstab2_elv2_;

	// Aileron Values
	XPLMDataRef wing1l_ail1def_;
	XPLMDataRef wing1l_ail2def_;
	XPLMDataRef wing1r_ail1def_;
	XPLMDataRef wing1r_ail2def_;
	XPLMDataRef wing2l_ail1def_;
	XPLMDataRef wing2l_ail2def_;
	XPLMDataRef wing2r_ail1def_;
	XPLMDataRef wing2r_ail2def_;
	XPLMDataRef wing3l_ail1def_;
	XPLMDataRef wing3r_ail1def_;
	XPLMDataRef wing3r_ail2def_;
	XPLMDataRef wing4l_ail1def_;
	XPLMDataRef wing4l_ail2def_;
	XPLMDataRef wing4r_ail1def_;
	XPLMDataRef wing4r_ail2def_;

	// Rudder Values
	XPLMDataRef vstab1_rud1def_;
	XPLMDataRef vstab1_rud2def_;
	XPLMDataRef vstab2_rud1def_;
	XPLMDataRef vstab2_rud2def_;

	XPLMDataRef engine_thro_;
	XPLMDataRef engine_thro_use_;
};


#endif //UAVEE_SERVOLISTENER_H
