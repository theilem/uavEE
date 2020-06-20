//
// Created by seedship on 6/19/20.
//

#ifndef UAVEE_XPLANENODE_H
#define UAVEE_XPLANENODE_H

#include <cpsCore/cps_object>
#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include <uavAP/Core/SensorData.h>

#include "xPlane/CHeaders/XPLM/XPLMDataAccess.h"

class XPlaneNode : public AggregatableObject<IScheduler>, IRunnableObject
{
public:
	XPlaneNode();

	static std::shared_ptr<XPlaneNode>
	create(const Configuration& config);

	bool
	run(RunStage stage) override;

	void
	enableAutopilot();

	void
	disableAutopilot();

private:

	void
	processData();

	void
	actuate(const ServoData& act);

	int sensorFrequency_;
	XPLMDataRef positionRefs_[3];
	XPLMDataRef velocityRefs_[3];
	XPLMDataRef trueAirSpeedRef_;
	XPLMDataRef accelerationRefs_[3];
	XPLMDataRef attitudeRefs_[3];
	XPLMDataRef angularRateRefs_[3];

	XPLMDataRef overridesRef_[2];
	XPLMDataRef joystickAttitudeRef_[3];

	//TODO add ServoData
	XPLMDataRef aileronRef_;
	XPLMDataRef elevatorRef_;
	XPLMDataRef rudderRef_;
	XPLMDataRef throttleRef_;
	XPLMDataRef rpmRef_;

	//TODO add PowerData
	XPLMDataRef batteryVoltageRef_;
	XPLMDataRef batteryCurrentRef_;

	ServoData servoData_;
	SensorData sensorData_;
	PowerData powerData_;

	//TODO update scheduling based on sim speed
	XPLMDataRef simSpeed_;
	Event sensorDataEvent_;

};


#endif //UAVEE_XPLANENODE_H
