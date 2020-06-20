//
// Created by seedship on 6/19/20.
//

#include <uavAP/API/ap_ext/latLongToUTM.h>

#include "uavEE/XPlaneInterface/XPlaneNode.h"

#define DEG2RAD (M_PI / 180.0)

XPlaneNode::XPlaneNode() :
		sensorFrequency_(100)
{
	positionRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/latitude");
	positionRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/longitude");
	positionRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/elevation");

	velocityRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/local_vx");
	velocityRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/local_vy");
	velocityRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/local_vz");

	trueAirSpeedRef_ = XPLMFindDataRef("sim/flightmodel/position/true_airspeed");

	accelerationRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/local_ax");
	accelerationRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/local_ay");
	accelerationRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/local_az");

	attitudeRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/phi");
	attitudeRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/theta");
	attitudeRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/psi");

	angularRateRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/P");
	angularRateRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/Q");
	angularRateRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/R");

	overridesRef_[0] = XPLMFindDataRef("sim/operation/override/override_joystick");
	overridesRef_[1] = XPLMFindDataRef("sim/operation/override/override_throttles");

	joystickAttitudeRef_[0] = XPLMFindDataRef("sim/joystick/yoke_roll_ratio");
	joystickAttitudeRef_[1] = XPLMFindDataRef("sim/joystick/yoke_pitch_ratio");
	joystickAttitudeRef_[2] = XPLMFindDataRef("sim/joystick/yoke_heading_ratio");

	aileronRef_ = XPLMFindDataRef("sim/flightmodel/controls/wing1l_ail1def");
	elevatorRef_ = XPLMFindDataRef("sim/flightmodel/controls/hstab1_elv1def");
	rudderRef_ = XPLMFindDataRef("sim/flightmodel/controls/vstab1_rud1def");
	throttleRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro_use");
	rpmRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_tacrad");

	batteryVoltageRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_bat_volt");
	batteryCurrentRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_bat_amp");

	simSpeed_ = XPLMFindDataRef("sim/time/sim_speed_actual_ogl");

	sensorData_.hasGPSFix = true;
	sensorData_.autopilotActive = false;
}

std::shared_ptr<XPlaneNode>
XPlaneNode::create(const Configuration& config)
{
	return std::shared_ptr<XPlaneNode>();
}

bool
XPlaneNode::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSet<IScheduler>())
			{
				CPSLOG_ERROR << "Scheduler not set.";
				return true;
			}
			break;
		}
		case RunStage::NORMAL:
		{
			sensorDataEvent_ = get<IScheduler>()->schedule([this]
														   { processData(); }, Milliseconds(0),
														   Milliseconds(1000 / sensorFrequency_));
			break;
		}
		default:
			break;
	}

	return false;
}

void
XPlaneNode::enableAutopilot()
{
	if (!sensorData_.autopilotActive)
	{
		sensorData_.autopilotActive = true;
		XPLMSetDatai(overridesRef_[0], 1);
		XPLMSetDatai(overridesRef_[1], 1);
	}
}

void
XPlaneNode::disableAutopilot()
{
	if (sensorData_.autopilotActive)
	{
		sensorData_.autopilotActive = false;
		XPLMSetDatai(overridesRef_[0], 0);
		XPLMSetDatai(overridesRef_[1], 0);
	}
}

void
XPlaneNode::processData()
{
	//Process SensorData
	double lat = XPLMGetDatad(positionRefs_[0]);
	double lon = XPLMGetDatad(positionRefs_[1]);
	double alt = XPLMGetDatad(positionRefs_[2]);

	double north = 0;
	double east = 0;
	int zone = 0;
	char hemi = 'N';

	latLongToUTM(lat, lon, north, east, zone, hemi);

	sensorData_.position[0] = east;
	sensorData_.position[1] = north;
	sensorData_.position[2] = alt;

	sensorData_.attitude[0] = static_cast<double>(XPLMGetDataf(attitudeRefs_[0])) * DEG2RAD;
	sensorData_.attitude[1] = static_cast<double>(XPLMGetDataf(attitudeRefs_[1])) * DEG2RAD;

	double yaw = static_cast<double>(XPLMGetDataf(attitudeRefs_[2])) * DEG2RAD;
	sensorData_.attitude[2] = boundAngleRad(-(yaw - M_PI_2));

	sensorData_.velocity[0] = static_cast<double>(XPLMGetDataf(velocityRefs_[0]));
	sensorData_.velocity[1] = -static_cast<double>(XPLMGetDataf(velocityRefs_[2]));
	sensorData_.velocity[2] = static_cast<double>(XPLMGetDataf(velocityRefs_[1]));

	sensorData_.groundSpeed = sensorData_.velocity.norm();
	sensorData_.airSpeed = static_cast<double>(XPLMGetDataf(trueAirSpeedRef_));

	sensorData_.angularRate[0] = static_cast<double>(XPLMGetDataf(angularRateRefs_[0])) * DEG2RAD;
	sensorData_.angularRate[1] = -static_cast<double>(XPLMGetDataf(angularRateRefs_[1])) * DEG2RAD;
	sensorData_.angularRate[2] = static_cast<double>(XPLMGetDataf(angularRateRefs_[2])) * DEG2RAD;

	Vector3 accelerationInertial;
	accelerationInertial[0] = static_cast<double>(XPLMGetDataf(accelerationRefs_[0]));
	accelerationInertial[1] = static_cast<double>(XPLMGetDataf(accelerationRefs_[2]));
	accelerationInertial[2] = static_cast<double>(XPLMGetDataf(accelerationRefs_[1]));

	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(-sensorData_.attitude[0], Vector3::UnitX())
		* Eigen::AngleAxisd(-sensorData_.attitude[1], Vector3::UnitY())
		* Eigen::AngleAxisd(-sensorData_.attitude[2], Vector3::UnitZ());

	sensorData_.acceleration = m * accelerationInertial;

	//Process Power Data
	powerData_.batteryCurrent = static_cast<double>(XPLMGetDataf(batteryCurrentRef_));
	powerData_.batteryVoltage = static_cast<double>(XPLMGetDataf(batteryVoltageRef_));

	//Process Servo Data
	servoData_.aileron = static_cast<double>(XPLMGetDataf(aileronRef_));
	servoData_.elevator = static_cast<double>(XPLMGetDataf(elevatorRef_));
	servoData_.rudder = static_cast<double>(XPLMGetDataf(rudderRef_));

	float throttle[8];
	XPLMGetDatavf(throttleRef_, throttle, 0, 8);
	servoData_.throttle = static_cast<double>(throttle[0]);

	float rpm[8];
	XPLMGetDatavf(rpmRef_, rpm, 0, 8);
	rpm[0] = rpm[0] * 60 / M_PI / 2; // Radians Per Second to Revolution Per Minute
	servoData_.rpm = static_cast<double>(rpm[0]);

	//Add Timestamps
	auto currTime = Clock::now();
	sensorData_.timestamp = currTime;
	powerData_.timestamp = currTime;
	servoData_.timestamp = currTime;

	//TODO publish sensor, power, and actuation data
}

void
XPlaneNode::actuate(const ServoData& act)
{
	if (!sensorData_.autopilotActive)
		return;

	float throt = (static_cast<float>(act.throttle) + 1) / 2;
	float throttle[] =
			{ throt, throt, throt, throt, throt, throt, throt, throt };
	XPLMSetDatavf(throttleRef_, throttle, 0, 8);
	XPLMSetDataf(joystickAttitudeRef_[0], act.aileron);
	XPLMSetDataf(joystickAttitudeRef_[1], act.elevator);
	XPLMSetDataf(joystickAttitudeRef_[2], act.rudder);
}
