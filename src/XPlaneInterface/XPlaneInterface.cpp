//
// Created by seedship on 6/19/20.
//

#include <cpsCore/Utilities/Scheduler/IScheduler.h>

#include <uavAP/API/ap_ext/latLongToUTM.h>
#include <uavAP/API/IAutopilotAPI.h>
#include <uavAP/FlightControl/Controller/AdvancedControl.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>

#include <uavAP/Core/Orientation/ConversionUtils.h>
#include <uavAP/Core/Orientation/NED.h>
#include <iostream>

#include "uavEE/XPlaneInterface/XPlaneInterface.h"

// Not including filesystem because older GCC compilers cant use it
#include <experimental/filesystem>

XPlaneInterface::XPlaneInterface() :
		sensorFrequency_(100),
		positionRefs_{XPLMFindDataRef("sim/flightmodel/position/latitude"),
					  XPLMFindDataRef("sim/flightmodel/position/longitude"),
					  XPLMFindDataRef("sim/flightmodel/position/elevation")},
		velocityRefs_{XPLMFindDataRef("sim/flightmodel/position/local_vx"),
					  XPLMFindDataRef("sim/flightmodel/position/local_vy"),
					  XPLMFindDataRef("sim/flightmodel/position/local_vz")},
		airSpeedRef_(XPLMFindDataRef("sim/flightmodel/position/true_airspeed")),
		accelerationRefs_{XPLMFindDataRef("sim/flightmodel/position/local_ax"),
						  XPLMFindDataRef("sim/flightmodel/position/local_ay"),
						  XPLMFindDataRef("sim/flightmodel/position/local_az")},
		attitudeRefs_{XPLMFindDataRef("sim/flightmodel/position/phi"),
					  XPLMFindDataRef("sim/flightmodel/position/theta"),
					  XPLMFindDataRef("sim/flightmodel/position/psi")},
		angleOfAttackRef_(XPLMFindDataRef("sim/flightmodel/position/alpha")),
		angleOfSideslipRef_(XPLMFindDataRef("sim/flightmodel/position/beta")),
		angularRateRefs_{XPLMFindDataRef("sim/flightmodel/position/P"),
						 XPLMFindDataRef("sim/flightmodel/position/Q"),
						 XPLMFindDataRef("sim/flightmodel/position/R")},
		gpsFixRef_(XPLMFindDataRef("sim/cockpit2/radios/actuators/gps_power")),
		batteryVoltageRef_(XPLMFindDataRef("sim/flightmodel/engine/ENGN_bat_volt")),
		batteryCurrentRef_(XPLMFindDataRef("sim/flightmodel/engine/ENGN_bat_amp")),
		aileronRef_(XPLMFindDataRef("sim/flightmodel/controls/wing1l_ail1def")),
		elevatorRef_(XPLMFindDataRef("sim/flightmodel/controls/hstab1_elv1def")),
		rudderRef_(XPLMFindDataRef("sim/flightmodel/controls/vstab1_rud1def")),
		throttleRef_(XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro_use")),
		rpmRef_(XPLMFindDataRef("sim/flightmodel/engine/ENGN_tacrad")),
		joystickOverrideRef_{XPLMFindDataRef("sim/operation/override/override_joystick"),
							 XPLMFindDataRef("sim/operation/override/override_throttles")},
		joystickAttitudeRef_{XPLMFindDataRef("sim/joystick/yoke_roll_ratio"),
							 XPLMFindDataRef("sim/joystick/yoke_pitch_ratio"),
							 XPLMFindDataRef("sim/joystick/yoke_heading_ratio")},
		course_(XPLMFindDataRef("sim/flightmodel/position/hpath")),
		temp_(XPLMFindDataRef("sim/weather/temperature_le_c")),
		pressure_(XPLMFindDataRef("sim/weather/barometer_current_inhg")),
		p_dot_(XPLMFindDataRef("sim/flightmodel/position/P_dot")),
		q_dot_(XPLMFindDataRef("sim/flightmodel/position/Q_dot")),
		r_dot_(XPLMFindDataRef("sim/flightmodel/position/R_dot")),
		simSpeed_(XPLMFindDataRef("sim/time/sim_speed_actual_ogl"))
{
	sensorData_.hasGPSFix = true;
	sensorData_.autopilotActive = false;
}

bool
XPlaneInterface::run(RunStage stage)
{
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
		case RunStage::NORMAL:
		{
			sensorDataEvent_ = get<IScheduler>()->schedule([this]
														   { processData(); }, Milliseconds(0),
														   Milliseconds(1000 / sensorFrequency_));

			if (auto autopilotAPI = get<IAutopilotAPI>())
			{
				autopilotAPI->subscribeOnControllerOut(
						std::bind(&XPlaneInterface::actuate, this, std::placeholders::_1));
			}
			break;
		}
		default:
			break;
	}

	return false;
}

void
XPlaneInterface::setAutopilotActive(bool active)
{
	sensorData_.autopilotActive = active;
	XPLMSetDatai(joystickOverrideRef_[0], active);
	XPLMSetDatai(joystickOverrideRef_[1], active);
}

void
XPlaneInterface::setLogging(bool logging)
{
	if (logging)
	{
		// Start logging
		if (file_.is_open())
		{
			CPSLOG_WARN << "Already logging!";
			return;
		}
		std::experimental::filesystem::path logpath(params.log_directory.value);
		std::stringstream filename;
		filename << params.name.value;
		filename << timePointToNanoseconds(Clock::now());
		logpath.append(filename.str() + params.file_extension.value);
		CPSLOG_DEBUG << "Begin logging to path " << logpath;
		file_.open(logpath);

		//Again, there must be a better way
#define SEP <<','<<
		file_ << "u" SEP "v" SEP "w" SEP "p" SEP "q" SEP "r" SEP "phi" SEP "theta" SEP "psi" SEP "roll_ctrl" SEP
			  "pitch_ctrl" SEP "yaw_ctrl" SEP "throttle_ctrl" SEP "E" SEP "N" SEP "U" SEP "u_dot" SEP "v_dot" SEP
			  "w_dot" SEP "p_dot" SEP "q_dot" SEP "r_dot" SEP "phi_dot" SEP "theta_dot" SEP "psi_dot" SEP "delta_a" SEP
			  "delta_e" SEP "delta_r" SEP "delta_T" SEP "timestamp\n";
#undef SEP
	}
	else
	{
		// Stop logging
		if (!file_.is_open())
		{
			CPSLOG_WARN << "Not logging!";
			return;
		}
		file_.close();
		CPSLOG_DEBUG << "Closed logfile";
	}
}

void
XPlaneInterface::processData()
{
	//Add Timestamps
	auto currTime = Clock::now();
	sensorData_.timestamp = currTime;
	powerData_.timestamp = currTime;
	servoData_.timestamp = currTime;

	// X-Plane uses an internal acf (aircraft) coordinate system
	// https://developer.x-plane.com/article/screencoordinates
	// x -> right
	// y -> up
	// z -> backwards

	//Process SensorData

	// FIXME Xplane does not recommend using our own transformations
	// https://developer.x-plane.com/article/screencoordinates/#3-D_Coordinate_System
	double lat = XPLMGetDatad(positionRefs_[0]);
	double lon = XPLMGetDatad(positionRefs_[1]);

	double north;
	double east;
	int zone;
	char hemi;

	latLongToUTM(lat, lon, north, east, zone, hemi);

	sensorData_.position[0] = east;
	sensorData_.position[1] = north;
	sensorData_.position[2] = XPLMGetDatad(positionRefs_[2]);

	sensorData_.orientation = Orientation::ENU;

	// Converting from ACF to ENU inertial frame
	sensorData_.velocity[0] = static_cast<FloatingType>(XPLMGetDataf(velocityRefs_[0]));
	sensorData_.velocity[1] = -static_cast<FloatingType>(XPLMGetDataf(velocityRefs_[2]));
	sensorData_.velocity[2] = static_cast<FloatingType>(XPLMGetDataf(velocityRefs_[1]));

	sensorData_.groundSpeed = sensorData_.velocity.norm();
	sensorData_.airSpeed = static_cast<FloatingType>(XPLMGetDataf(airSpeedRef_));

	// Converting from ACF to ENU inertial frame
	sensorData_.acceleration[0] = static_cast<FloatingType>(XPLMGetDataf(accelerationRefs_[0]));
	sensorData_.acceleration[1] = -static_cast<FloatingType>(XPLMGetDataf(accelerationRefs_[2]));
	sensorData_.acceleration[2] = static_cast<FloatingType>(XPLMGetDataf(accelerationRefs_[1]));

	// Converting acceleration to body frame
	directionalConversion(sensorData_.acceleration, sensorData_.attitude, Frame::BODY, Orientation::ENU);

	sensorData_.attitude[0] = degToRad(static_cast<FloatingType>(XPLMGetDataf(attitudeRefs_[0])));
	sensorData_.attitude[1] = degToRad(static_cast<FloatingType>(XPLMGetDataf(attitudeRefs_[1])));

	FloatingType nedYaw = degToRad(static_cast<FloatingType>(XPLMGetDataf(attitudeRefs_[2])));
	sensorData_.attitude[2] = boundAngleRad(-(nedYaw - M_PI_2));

	//ENU angle of attack = -NED angle of attack
	sensorData_.angleOfAttack = -degToRad(static_cast<FloatingType>(XPLMGetDataf(angleOfAttackRef_)));
	sensorData_.angleOfSideslip = degToRad(static_cast<FloatingType>(XPLMGetDataf(angleOfSideslipRef_)));

	//enu PQR <-> QP(-R)
	sensorData_.angularRate[1] = degToRad(static_cast<FloatingType>(XPLMGetDataf(angularRateRefs_[0])));
	sensorData_.angularRate[0] = degToRad(static_cast<FloatingType>(XPLMGetDataf(angularRateRefs_[1])));
	sensorData_.angularRate[2] = -degToRad(static_cast<FloatingType>(XPLMGetDataf(angularRateRefs_[2])));
	sensorData_.angularRate.frame = Frame::BODY;

	sensorData_.hasGPSFix = static_cast<bool>(XPLMGetDatai(gpsFixRef_));

	FloatingType course = degToRad(static_cast<FloatingType>(XPLMGetDataf(course_)));
	sensorData_.courseAngle = boundAngleRad(-(course - M_PI_2));
	sensorData_.temperature = static_cast<FloatingType>(XPLMGetDataf(temp_));
	sensorData_.pressure = static_cast<FloatingType>(XPLMGetDataf(pressure_));

	//Process Power Data
	powerData_.batteryCurrent = static_cast<FloatingType>(XPLMGetDataf(batteryCurrentRef_));
	powerData_.batteryVoltage = static_cast<FloatingType>(XPLMGetDataf(batteryVoltageRef_));

	//Process Servo Data
	servoData_.aileron = static_cast<FloatingType>(XPLMGetDataf(aileronRef_));
	servoData_.elevator = static_cast<FloatingType>(XPLMGetDataf(elevatorRef_));
	servoData_.rudder = static_cast<FloatingType>(XPLMGetDataf(rudderRef_));

	sensorData_.sequenceNumber = sequenceNumber_++;
	if (sequenceNumber_ > std::numeric_limits<uint16_t>::max())
		sequenceNumber_ = 0;

	float throttle[8];
	XPLMGetDatavf(throttleRef_, throttle, 0, 8);
	servoData_.throttle = static_cast<FloatingType>(throttle[0]);

	float rpm[8];
	XPLMGetDatavf(rpmRef_, rpm, 0, 8);
	rpm[0] = rpm[0] * 60 / M_PI / 2; // Radians Per Second to Revolution Per Minute
	servoData_.rpm = static_cast<FloatingType>(rpm[0]);

	if (auto api = get<IAutopilotAPI>())
	{
		api->setSensorData(sensorData_);
		api->setPowerData(powerData_);
		api->setServoData(servoData_);
	}
	else
	{
		CPSLOG_ERROR << "Missing IAutopilotAPI";
	}

	if (file_.is_open())
	{
		SensorData sd_ned = sensorData_;
		NED::convert(sd_ned, Frame::BODY);
		FramedVector3 attitudeRate = sd_ned.angularRate;
		angularConversion(attitudeRate, sd_ned.attitude, Frame::INERTIAL, Orientation::NED);
		/*
		file_ << "u" SEP "v" SEP "w" SEP "p" SEP "q" SEP "r" SEP "phi" SEP "theta" SEP "psi" SEP "roll_ctrl" SEP
			  "pitch_ctrl" SEP "yaw_ctrl" SEP "throttle_ctrl" SEP "E" SEP "N" SEP "U" SEP "u_dot" SEP "v_dot" SEP
			  "w_dot" SEP "p_dot" SEP "q_dot" SEP "r_dot" SEP "phi_dot" SEP "theta_dot" SEP "psi_dot" SEP "delta_a" SEP
			  "delta_e" SEP "delta_r" SEP "delta_T" SEP "timestamp\n";
		 */
		//FIXME there must be a better way
#define SEP <<','<<
		file_ << sd_ned.velocity[0] SEP sd_ned.velocity[1] SEP sd_ned.velocity[2] SEP sd_ned.angularRate[0] SEP
			  sd_ned.angularRate[1] SEP sd_ned.angularRate[2] SEP sd_ned.attitude[0] SEP sd_ned.attitude[1] SEP
			  sd_ned.attitude[2] SEP coToLog_.rollOutput SEP coToLog_.pitchOutput SEP coToLog_.yawOutput SEP
			  coToLog_.throttleOutput SEP sensorData_.position[0] SEP sensorData_.position[1] SEP
			  sensorData_.position[2] SEP sd_ned.acceleration[0] SEP sd_ned.acceleration[1] SEP sd_ned.acceleration[2]
			  SEP degToRad(XPLMGetDataf(p_dot_)) SEP degToRad(XPLMGetDataf(q_dot_)) SEP
			  degToRad(XPLMGetDataf(r_dot_)) SEP attitudeRate[0] SEP attitudeRate[1] SEP attitudeRate[2] SEP
			  servoData_.aileron SEP servoData_.elevator SEP servoData_.rudder SEP servoData_.throttle SEP
			  durationToNanoseconds(sensorData_.timestamp.time_since_epoch()) << '\n';
#undef SEP
	}
}

void
XPlaneInterface::actuate(const ControllerOutput& out)
{
	CPSLOG_TRACE << "Begin Actuate\n";
	if (!sensorData_.autopilotActive)
		return;
	coToLog_ = out;
	float throt = (static_cast<float>(out.throttleOutput) + 1) / 2;
	float throttle[] =
			{throt, throt, throt, throt, throt, throt, throt, throt};
	XPLMSetDatavf(throttleRef_, throttle, 0, 8);
	XPLMSetDataf(joystickAttitudeRef_[0], out.rollOutput);
	XPLMSetDataf(joystickAttitudeRef_[1], out.pitchOutput);
	XPLMSetDataf(joystickAttitudeRef_[2], out.yawOutput);
	CPSLOG_TRACE << "End Actuate\n";
}

const SensorData&
XPlaneInterface::getSensorData() const
{
	return sensorData_;
}
