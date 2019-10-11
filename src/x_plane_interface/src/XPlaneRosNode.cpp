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
/**
 * @file	XPlaneRosNode.cpp
 * @author	Mirco Theile, mircot@illinois.edu
 * @date	[DD.MM.YYYY] 27.3.2018
 * @brief	X-Plane ROS Interface Node
 */

#include <cmath>
#include <thread>
#include <uavAP/API/ap_ext/UTM.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/Scheduler/IScheduler.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <simulation_interface/wind_layer.h>

#include "xPlane/CHeaders/XPLM/XPLMDefs.h"
#include "xPlane/CHeaders/XPLM/XPLMDataAccess.h"
#include "xPlane/CHeaders/XPLM/XPLMGraphics.h"
#include "x_plane_interface/XPlaneRosNode.h"

XPlaneRosNode::XPlaneRosNode() :
		sequenceNr_(0), sensorFrequency_(100), autopilotActive_(false)
{
	getDataRefs();
}

XPlaneRosNode::~XPlaneRosNode()
{
}

std::shared_ptr<XPlaneRosNode>
XPlaneRosNode::create(const boost::property_tree::ptree& config)
{
	return std::make_shared<XPlaneRosNode>();
}

void
XPlaneRosNode::notifyAggregationOnUpdate(const Aggregator& agg)
{
	scheduler_.setFromAggregationIfNotSet(agg);
}

bool
XPlaneRosNode::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "XPlaneRosNode: Scheduler Missing.";
			return true;
		}

		ros::VP_string config;
		ros::init(config, "xplane_interface");

		break;
	}
	case RunStage::NORMAL:
	{
		ros::NodeHandle nh;

		sensorDataPublisher_ = nh.advertise<simulation_interface::sensor_data>(
				"/x_plane_interface/sensor_data", 20);
		sensorDataSubscriber_ = nh.subscribe("/ground_station/sensor_data", 20,
				&XPlaneRosNode::setSensorData, this);
		actuationSubscriber_ = nh.subscribe("/autopilot_interface/actuation", 20,
				&XPlaneRosNode::setActuationData, this);
		engineService_ = nh.advertiseService("/x_plane_interface/engine", &XPlaneRosNode::engine,
				this);

		auto scheduler = scheduler_.get();

		if (!scheduler)
		{
			APLOG_ERROR << "XPlaneRosNode: Scheduler Missing.";
			return true;
		}

		scheduler->schedule(std::bind(&XPlaneRosNode::publishSensorData, this), Milliseconds(0),
				Milliseconds(1000 / sensorFrequency_));

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
XPlaneRosNode::enableJoystick()
{
	XPLMSetDatai(joystickOverrideRef_[0], 0);
	XPLMSetDatai(joystickOverrideRef_[1], 0);
}

void
XPlaneRosNode::disableJoystick()
{
	XPLMSetDatai(joystickOverrideRef_[0], 1);
	XPLMSetDatai(joystickOverrideRef_[1], 1);
}

void
XPlaneRosNode::enableAutopilot()
{
	if (!autopilotActive_)
	{
		autopilotActive_ = true;
		XPLMSetDatai(joystickOverrideRef_[0], 1);
		XPLMSetDatai(joystickOverrideRef_[1], 1);

		std::cout << "XPlaneRosNode: Autopilot On." << std::endl;
	}
}

void
XPlaneRosNode::disableAutopilot()
{
	if (autopilotActive_)
	{
		autopilotActive_ = false;
		XPLMSetDatai(joystickOverrideRef_[0], 0);
		XPLMSetDatai(joystickOverrideRef_[1], 0);

		std::cout << "XPlaneRosNode: Autopilot Off." << std::endl;
	}

//	/* TEST */
//	double x, y, z;
//	XPLMWorldToLocal(40.05, -88.50, 2000, &x, &y, &z);
//	XPLMSetDatad(XPLMFindDataRef("sim/flightmodel/position/local_x"), x);
//	XPLMSetDatad(XPLMFindDataRef("sim/flightmodel/position/local_y"), y);
//	XPLMSetDatad(XPLMFindDataRef("sim/flightmodel/position/local_z"), z);
//	/* TEST */
}

void
XPlaneRosNode::engineStart()
{
	int ignitionKeyValue = 4;
	int ignitionKey[] =
	{ ignitionKeyValue, ignitionKeyValue, ignitionKeyValue, ignitionKeyValue, ignitionKeyValue,
			ignitionKeyValue, ignitionKeyValue, ignitionKeyValue };
	TimePoint timestampStart = boost::posix_time::second_clock::local_time();
	TimePoint timestampCurrent = boost::posix_time::second_clock::local_time();

	while (timestampCurrent - timestampStart <= Seconds(2))
	{
		setDataRef(ignitionKeyRef_, ignitionKey, 8);
		timestampCurrent = boost::posix_time::second_clock::local_time();
	}
}

bool
XPlaneRosNode::engine(radio_comm::engine::Request& request, radio_comm::engine::Response& response)
{
	if (request.start)
	{
		std::thread engineStartThread(&XPlaneRosNode::engineStart, this);
		engineStartThread.detach();
	}
	else
	{
		int ignitionKeyValue = 0;
		int ignitionKey[] =
		{ ignitionKeyValue, ignitionKeyValue, ignitionKeyValue, ignitionKeyValue, ignitionKeyValue,
				ignitionKeyValue, ignitionKeyValue, ignitionKeyValue };
		setDataRef(ignitionKeyRef_, ignitionKey, 8);
	}

	return true;
}

void
XPlaneRosNode::getDataRefs()
{
	positionRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/latitude");
	positionRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/longitude");
	positionRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/elevation");

	positionLocalRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/local_x");
	positionLocalRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/local_y");
	positionLocalRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/local_z");

	velocityRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/local_vx");
	velocityRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/local_vy");
	velocityRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/local_vz");

	airSpeedRef_ = XPLMFindDataRef("sim/flightmodel/position/true_airspeed");

	accelerationRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/local_ax");
	accelerationRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/local_ay");
	accelerationRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/local_az");

	attitudeRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/phi");
	attitudeRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/theta");
	attitudeRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/psi");

	angleOfAttackRef_ = XPLMFindDataRef("sim/flightmodel2/misc/AoA_angle_degrees");
	angleOfSideslipRef_ = XPLMFindDataRef("sim/flightmodel/position/beta");

	angularRateRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/P");
	angularRateRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/Q");
	angularRateRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/R");

	gpsFixRef_ = XPLMFindDataRef("sim/cockpit2/radios/actuators/gps_power");

	batteryVoltageRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_bat_volt");
	batteryCurrentRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_bat_amp");

	motorSpeedRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_tacrad");

	aileronLevelRef_ = XPLMFindDataRef("sim/flightmodel/controls/wing1l_ail1def");
	elevatorLevelRef_ = XPLMFindDataRef("sim/flightmodel/controls/hstab1_elv1def");
	rudderLevelRef_ = XPLMFindDataRef("sim/flightmodel/controls/vstab1_rud1def");
	throttleLevelRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro_use");

	precipitationLevelRef_ = XPLMFindDataRef("sim/weather/rain_percent");
	storminessLevelRef_ = XPLMFindDataRef("sim/weather/thunderstorm_percent");

	windAltitudeRef_[0] = XPLMFindDataRef("sim/weather/wind_altitude_msl_m[0]");
	windAltitudeRef_[1] = XPLMFindDataRef("sim/weather/wind_altitude_msl_m[1]");
	windAltitudeRef_[2] = XPLMFindDataRef("sim/weather/wind_altitude_msl_m[2]");

	windDirectionRef_[0] = XPLMFindDataRef("sim/weather/wind_direction_degt[0]");
	windDirectionRef_[1] = XPLMFindDataRef("sim/weather/wind_direction_degt[1]");
	windDirectionRef_[2] = XPLMFindDataRef("sim/weather/wind_direction_degt[2]");

	windSpeedRef_[0] = XPLMFindDataRef("sim/weather/wind_speed_kt[0]");
	windSpeedRef_[1] = XPLMFindDataRef("sim/weather/wind_speed_kt[1]");
	windSpeedRef_[2] = XPLMFindDataRef("sim/weather/wind_speed_kt[2]");

	windTurbulenceRef_[0] = XPLMFindDataRef("sim/weather/turbulence[0]");
	windTurbulenceRef_[1] = XPLMFindDataRef("sim/weather/turbulence[1]");
	windTurbulenceRef_[2] = XPLMFindDataRef("sim/weather/turbulence[2]");

	windShearDirectionRef_[0] = XPLMFindDataRef("sim/weather/shear_direction_degt[0]");
	windShearDirectionRef_[1] = XPLMFindDataRef("sim/weather/shear_direction_degt[1]");
	windShearDirectionRef_[2] = XPLMFindDataRef("sim/weather/shear_direction_degt[2]");

	windShearSpeedRef_[0] = XPLMFindDataRef("sim/weather/shear_speed_kt[0]");
	windShearSpeedRef_[1] = XPLMFindDataRef("sim/weather/shear_speed_kt[1]");
	windShearSpeedRef_[2] = XPLMFindDataRef("sim/weather/shear_speed_kt[2]");

	joystickOverrideRef_[0] = XPLMFindDataRef("sim/operation/override/override_joystick");
	joystickOverrideRef_[1] = XPLMFindDataRef("sim/operation/override/override_throttles");

	actuationRef_[0] = XPLMFindDataRef("sim/joystick/yoke_roll_ratio");
	actuationRef_[1] = XPLMFindDataRef("sim/joystick/yoke_pitch_ratio");
	actuationRef_[2] = XPLMFindDataRef("sim/joystick/yoke_heading_ratio");

	ignitionKeyRef_ = XPLMFindDataRef("sim/cockpit2/engine/actuators/ignition_key");
}

void
XPlaneRosNode::setDataRef(const XPLMDataRef& dataRef, const double& data)
{
	if (!std::isnan(data))
	{
		XPLMSetDatad(dataRef, data);
	}
}

void
XPlaneRosNode::setDataRef(const XPLMDataRef& dataRef, const float& data)
{
	if (!std::isnan(data))
	{
		XPLMSetDataf(dataRef, data);
	}
}

void
XPlaneRosNode::setDataRef(const XPLMDataRef& dataRef, float* data, int size)
{
	for (int i = 0; i < size; i++)
	{
		if (std::isnan(data[i]))
		{
			return;
		}
	}

	XPLMSetDatavf(dataRef, data, 0, size);
}

void
XPlaneRosNode::setDataRef(const XPLMDataRef& dataRef, int* data, int size)
{
	for (int i = 0; i < size; i++)
	{
		if (std::isnan(data[i]))
		{
			return;
		}
	}

	XPLMSetDatavi(dataRef, data, 0, size);
}

void
XPlaneRosNode::setDataRef(const XPLMDataRef& dataRef, const int& data)
{
	if (!std::isnan(data))
	{
		XPLMSetDatai(dataRef, data);
	}
}

void
XPlaneRosNode::publishSensorData()
{
	double latitude = XPLMGetDatad(positionRefs_[0]);
	double longitude = XPLMGetDatad(positionRefs_[1]);
	double easting = 0;
	double northing = 0;
	double yawAngleUnbounded = 0;
	float batteryVoltage[8];
	float batteryCurrent[8];
	float motorSpeed[8];
	float throttleLevel[8];
	simulation_interface::sensor_data sensorData;
	Vector3 accelerationInertial;
	Vector3 accelerationBody;
	Eigen::Matrix3d rotationMatrix;

	LatLonToUTMXY(latitude, longitude, 0, easting, northing);

	sensorData.sequenceNr = sequenceNr_++;
	sensorData.header.stamp = ros::Time::now();

	sensorData.position.x = easting;
	sensorData.position.y = northing;
	sensorData.position.z = XPLMGetDatad(positionRefs_[2]);

	sensorData.velocity.linear.x = static_cast<double>(XPLMGetDataf(velocityRefs_[0]));
	sensorData.velocity.linear.y = static_cast<double>(XPLMGetDataf(velocityRefs_[2])) * -1;
	sensorData.velocity.linear.z = static_cast<double>(XPLMGetDataf(velocityRefs_[1]));

	sensorData.air_speed = static_cast<double>(XPLMGetDataf(airSpeedRef_));
	sensorData.ground_speed = sqrt(
			pow(sensorData.velocity.linear.x, 2) + pow(sensorData.velocity.linear.y, 2)
					+ pow(sensorData.velocity.linear.z, 2));

	accelerationInertial[0] = static_cast<double>(XPLMGetDataf(accelerationRefs_[0]));
	accelerationInertial[1] = static_cast<double>(XPLMGetDataf(accelerationRefs_[2]));
	accelerationInertial[2] = static_cast<double>(XPLMGetDataf(accelerationRefs_[1]));
	rotationMatrix = Eigen::AngleAxisd(sensorData.attitude.x, Vector3::UnitX() * -1)
			* Eigen::AngleAxisd(sensorData.attitude.y, Vector3::UnitY() * -1)
			* Eigen::AngleAxisd(sensorData.attitude.z, Vector3::UnitZ() * -1);
	accelerationBody = rotationMatrix * accelerationInertial;
	sensorData.acceleration.linear.x = accelerationBody[0];
	sensorData.acceleration.linear.y = accelerationBody[1];
	sensorData.acceleration.linear.z = accelerationBody[2];

	yawAngleUnbounded = degToRad(static_cast<double>(XPLMGetDataf(attitudeRefs_[2])));
	sensorData.attitude.x = degToRad(static_cast<double>(XPLMGetDataf(attitudeRefs_[0])));
	sensorData.attitude.y = degToRad(static_cast<double>(XPLMGetDataf(attitudeRefs_[1])));
	sensorData.attitude.z = boundAngleRad((yawAngleUnbounded - M_PI / 2) * -1);

	sensorData.angle_of_attack = degToRad(static_cast<double>(XPLMGetDataf(angleOfAttackRef_)));
	sensorData.angle_of_sideslip = degToRad(static_cast<double>(XPLMGetDataf(angleOfSideslipRef_)));

	sensorData.velocity.angular.x = degToRad(
			static_cast<double>(XPLMGetDataf(angularRateRefs_[0])));
	sensorData.velocity.angular.y = degToRad(
			static_cast<double>(XPLMGetDataf(angularRateRefs_[1])) * -1);
	sensorData.velocity.angular.z = degToRad(
			static_cast<double>(XPLMGetDataf(angularRateRefs_[2])));

	sensorData.gps_fix = static_cast<bool>(XPLMGetDatai(gpsFixRef_));
	sensorData.autopilot_active = autopilotActive_;

	XPLMGetDatavf(batteryVoltageRef_, batteryVoltage, 0, 8);
	XPLMGetDatavf(batteryCurrentRef_, batteryCurrent, 0, 8);
	sensorData.battery_voltage = batteryVoltage[0];
	sensorData.battery_current = batteryCurrent[0];

	XPLMGetDatavf(motorSpeedRef_, motorSpeed, 0, 8);
	motorSpeed[0] = motorSpeed[0] * 60 / M_PI / 2; // Radians per second to revolutions per minute
	sensorData.motor_speed = motorSpeed[0];

	XPLMGetDatavf(throttleLevelRef_, throttleLevel, 0, 8);
	sensorData.aileron_level = static_cast<double>(XPLMGetDataf(aileronLevelRef_));
	sensorData.elevator_level = static_cast<double>(XPLMGetDataf(elevatorLevelRef_));
	sensorData.rudder_level = static_cast<double>(XPLMGetDataf(rudderLevelRef_));
	sensorData.throttle_level = throttleLevel[0];

	sensorData.precipitation_level = static_cast<double>(XPLMGetDataf(precipitationLevelRef_));
	sensorData.storminess_level = static_cast<double>(XPLMGetDataf(storminessLevelRef_));

	sensorData.wind_layers.push_back(simulation_interface::wind_layer());
	sensorData.wind_layers.push_back(simulation_interface::wind_layer());
	sensorData.wind_layers.push_back(simulation_interface::wind_layer());

	for (unsigned i = 0; i < 3; i++)
	{
		sensorData.wind_layers[i].wind_layer_index = static_cast<double>(i);
		sensorData.wind_layers[i].wind_altitude = static_cast<double>(XPLMGetDataf(
				windAltitudeRef_[i]));
		sensorData.wind_layers[i].wind_direction = boundAngleRad(
				degToRad(static_cast<double>(XPLMGetDataf(windDirectionRef_[i]) - 90) * -1));
		sensorData.wind_layers[i].wind_speed = static_cast<double>(XPLMGetDataf(windSpeedRef_[i]))
				/ 1.944;
		sensorData.wind_layers[i].wind_turbulence = static_cast<double>(XPLMGetDataf(
				windTurbulenceRef_[i])) / 10;

		sensorData.wind_layers[i].wind_shear_direction = degToRad(
				static_cast<double>(XPLMGetDataf(windShearDirectionRef_[i])));
		sensorData.wind_layers[i].wind_shear_speed = static_cast<double>(XPLMGetDataf(
				windShearSpeedRef_[i])) / 1.944;
	}

	sensorDataPublisher_.publish(sensorData);

	ros::spinOnce();
}

void
XPlaneRosNode::setSensorData(const simulation_interface::sensor_data& sensorData)
{
	double latitude = 0;
	double longitude = 0;
	double easting = sensorData.position.x;
	double northing = sensorData.position.y;
	double yawAngleUnbounded = 0;
	double positionLocalX = 0;
	double positionLocalY = 0;
	double positionLocalZ = 0;
	int windLayerIndex;
	Vector3 accelerationBody;
	Vector3 accelerationInertial;
	Vector3 attitude;
	Eigen::Matrix3d rotationMatrix;

	UTMXYToLatLon(easting, northing, 16, false, latitude, longitude);
	XPLMWorldToLocal(radToDeg(latitude), radToDeg(longitude), sensorData.position.z,
			&positionLocalX, &positionLocalY, &positionLocalZ);
	setDataRef(positionLocalRefs_[0], positionLocalX);
	setDataRef(positionLocalRefs_[1], positionLocalY);
	setDataRef(positionLocalRefs_[2], positionLocalZ);

	setDataRef(velocityRefs_[0], static_cast<float>(sensorData.velocity.linear.x));
	setDataRef(velocityRefs_[2], static_cast<float>(sensorData.velocity.linear.y * -1));
	setDataRef(velocityRefs_[1], static_cast<float>(sensorData.velocity.linear.z));

	if (!std::isnan(sensorData.attitude.x) && !std::isnan(sensorData.attitude.y)
			&& !std::isnan(sensorData.attitude.z))
	{
		attitude[0] = sensorData.attitude.x;
		attitude[1] = sensorData.attitude.y;
		attitude[2] = sensorData.attitude.z;
	}
	else
	{
		yawAngleUnbounded = degToRad(static_cast<double>(XPLMGetDataf(attitudeRefs_[2])));
		attitude[0] = degToRad(static_cast<double>(XPLMGetDataf(attitudeRefs_[0])));
		attitude[1] = degToRad(static_cast<double>(XPLMGetDataf(attitudeRefs_[1])));
		attitude[2] = boundAngleRad((yawAngleUnbounded - M_PI / 2) * -1);
	}
	accelerationBody[0] = sensorData.acceleration.linear.x;
	accelerationBody[1] = sensorData.acceleration.linear.y;
	accelerationBody[2] = sensorData.acceleration.linear.z;
	if (!std::isnan(accelerationBody[0]) && !std::isnan(accelerationBody[1])
			&& !std::isnan(accelerationBody[2]))
	{
		rotationMatrix = Eigen::AngleAxisd(attitude[0], Vector3::UnitX() * -1)
				* Eigen::AngleAxisd(attitude[1], Vector3::UnitY() * -1)
				* Eigen::AngleAxisd(attitude[2], Vector3::UnitZ() * -1);
		accelerationInertial = rotationMatrix * accelerationBody;
		setDataRef(accelerationRefs_[0], static_cast<float>(accelerationInertial[0]));
		setDataRef(accelerationRefs_[1], static_cast<float>(accelerationInertial[1]));
		setDataRef(accelerationRefs_[2], static_cast<float>(accelerationInertial[2]));
	}

	setDataRef(attitudeRefs_[0], static_cast<float>(radToDeg(sensorData.attitude.x)));
	setDataRef(attitudeRefs_[1], static_cast<float>(radToDeg(sensorData.attitude.y)));
	setDataRef(attitudeRefs_[2],
			static_cast<float>(radToDeg(sensorData.attitude.z * -1 + M_PI / 2)));

	setDataRef(angularRateRefs_[0], static_cast<float>(radToDeg(sensorData.velocity.angular.x)));
	setDataRef(angularRateRefs_[1],
			static_cast<float>(radToDeg(sensorData.velocity.angular.y * -1)));
	setDataRef(angularRateRefs_[2], static_cast<float>(radToDeg(sensorData.velocity.angular.z)));

	setDataRef(gpsFixRef_, static_cast<int>(sensorData.gps_fix));
	if (sensorData.autopilot_active)
	{
		enableAutopilot();
	}
	else
	{
		disableAutopilot();
	}

	float batteryVoltageValue = static_cast<float>(sensorData.battery_voltage);
	float batteryCurrentValue = static_cast<float>(sensorData.battery_current);
	float batteryVoltage[] =
	{ batteryVoltageValue, batteryVoltageValue, batteryVoltageValue, batteryVoltageValue,
			batteryVoltageValue, batteryVoltageValue, batteryVoltageValue, batteryVoltageValue };
	float batteryCurrent[] =
	{ batteryCurrentValue, batteryCurrentValue, batteryCurrentValue, batteryCurrentValue,
			batteryCurrentValue, batteryCurrentValue, batteryCurrentValue, batteryCurrentValue };
	setDataRef(batteryVoltageRef_, batteryVoltage, 8);
	setDataRef(batteryCurrentRef_, batteryCurrent, 8);

	float motorSpeedValue = static_cast<float>(sensorData.motor_speed * M_PI * 2 / 60);
	float motorSpeed[] =
	{ motorSpeedValue, motorSpeedValue, motorSpeedValue, motorSpeedValue, motorSpeedValue,
			motorSpeedValue, motorSpeedValue, motorSpeedValue }; // Revolutions per minute to radians per second
	setDataRef(motorSpeedRef_, motorSpeed, 8);

	float throttleLevelValue = static_cast<float>(sensorData.throttle_level);
	float throttleLevel[] =
	{ throttleLevelValue, throttleLevelValue, throttleLevelValue, throttleLevelValue,
			throttleLevelValue, throttleLevelValue, throttleLevelValue, throttleLevelValue };
	setDataRef(aileronLevelRef_, static_cast<float>(sensorData.aileron_level));
	setDataRef(elevatorLevelRef_, static_cast<float>(sensorData.elevator_level));
	setDataRef(rudderLevelRef_, static_cast<float>(sensorData.rudder_level));
	setDataRef(throttleLevelRef_, throttleLevel, 8);

	setDataRef(precipitationLevelRef_, static_cast<float>(sensorData.precipitation_level));
	setDataRef(storminessLevelRef_, static_cast<float>(sensorData.storminess_level));

	if (sensorData.wind_layers.size() > 0)
	{
		simulation_interface::wind_layer windLayer = sensorData.wind_layers[0];
		windLayerIndex = windLayer.wind_layer_index;

		setDataRef(windAltitudeRef_[windLayerIndex], static_cast<float>(windLayer.wind_altitude));
		setDataRef(windDirectionRef_[windLayerIndex],
				static_cast<float>(radToDeg(windLayer.wind_direction * -1 + M_PI / 2)));
		setDataRef(windSpeedRef_[windLayerIndex], static_cast<float>(windLayer.wind_speed * 1.944));
		setDataRef(windTurbulenceRef_[windLayerIndex],
				static_cast<float>(windLayer.wind_turbulence * 10));
		setDataRef(windShearDirectionRef_[windLayerIndex],
				static_cast<float>(radToDeg(windLayer.wind_shear_direction)));
		setDataRef(windShearSpeedRef_[windLayerIndex],
				static_cast<float>(windLayer.wind_shear_speed * 1.944));
	}
}

void
XPlaneRosNode::setActuationData(const simulation_interface::actuation& actuation)
{
	if (!autopilotActive_)
	{
		return;
	}

	float roll = static_cast<float>(actuation.rollOutput);
	float pitch = static_cast<float>(actuation.pitchOutput);
	float yaw = static_cast<float>(actuation.yawOutput);
	float throttleValue = (static_cast<float>(actuation.throttleOutput) + 1) / 2;
	float throttle[] =
	{ throttleValue, throttleValue, throttleValue, throttleValue, throttleValue, throttleValue,
			throttleValue, throttleValue };

	XPLMSetDataf(actuationRef_[0], roll);
	XPLMSetDataf(actuationRef_[1], pitch);
	XPLMSetDataf(actuationRef_[2], yaw);
	XPLMSetDatavf(throttleLevelRef_, throttle, 0, 8);
}
