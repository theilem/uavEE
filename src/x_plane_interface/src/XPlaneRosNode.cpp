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
 * @file XPlaneRosNode.cpp
 * @author Mirco Theile, mircot@illinois.edu
 * @date [DD.MM.YYYY] 27.3.2018
 * @brief
 */

#include "x_plane_interface/XPlaneRosNode.h"
#include <uavAP/Core/Scheduler/IScheduler.h>
#include <uavAP/Core/Logging/APLogger.h>

#include <simulation_interface/sensor_data.h>

#include "xPlane/CHeaders/XPLM/XPLMDefs.h"
#include "xPlane/CHeaders/XPLM/XPLMDataAccess.h"
#include "xPlane/CHeaders/XPLM/XPLMGraphics.h"

#include <uavAP/API/ap_ext/latLongToUTM.h>
#include <uavAP/Core/LinearAlgebra.h>

#include <cmath>

XPlaneRosNode::XPlaneRosNode() :
		sensorFrequency_(100), sequenceNr_(0), autopilotActive_(false)
{
	ros::VP_string config;
	ros::init(config, "xplane_interface");

	nodeHandle_ = new ros::NodeHandle();

	positionRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/latitude");
	positionRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/longitude");
	positionRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/elevation");

	velocityRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/local_vx");
	velocityRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/local_vy");
	velocityRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/local_vz");

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

	throttleRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro_use");

//    double x, y, z;
//    XPLMWorldToLocal(40.0594, -88.5514, 206, &x, &y, &z);
//    XPLMSetDatad(XPLMFindDataRef("sim/flightmodel/position/local_x"), x);
//    XPLMSetDatad(XPLMFindDataRef("sim/flightmodel/position/local_y"), y);
//    XPLMSetDatad(XPLMFindDataRef("sim/flightmodel/position/local_z"), z);
}

XPlaneRosNode::~XPlaneRosNode()
{
	if (nodeHandle_)
		delete nodeHandle_;
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
			APLOG_ERROR << "Scheduler not set.";
			return true;
		}
		break;
	}
	case RunStage::NORMAL:
	{
		sensorDataPublisher_ = nodeHandle_->advertise<simulation_interface::sensor_data>(
				"sensor_data", 20);

		actuationSubscriber_ = nodeHandle_->subscribe("actuation", 20, &XPlaneRosNode::actuate,
				this);
		auto scheduler = scheduler_.get();

		scheduler->schedule(std::bind(&XPlaneRosNode::getSensorData, this), Milliseconds(0),
				Milliseconds(1000 / sensorFrequency_));
		break;
	}
	default:
		break;
	}

	return false;
}

void
XPlaneRosNode::toggleAutopilot()
{
	autopilotActive_ = autopilotActive_ ? false : true;

	if (autopilotActive_)
	{
		XPLMSetDatai(overridesRef_[0], 1);
		XPLMSetDatai(overridesRef_[1], 1);
	}
	else
	{
		XPLMSetDatai(overridesRef_[0], 0);
		XPLMSetDatai(overridesRef_[1], 0);
	}
}

void
XPlaneRosNode::getSensorData()
{
	double lat = XPLMGetDatad(positionRefs_[0]);
	double lon = XPLMGetDatad(positionRefs_[1]);
	double alt = XPLMGetDatad(positionRefs_[2]);

	double north = 0;
	double east = 0;
	int zone = 0;
	char hemi = 'N';

	latLongToUTM(lat, lon, north, east, zone, hemi);
	std::cout << lat << " " << lon << std::endl;

	simulation_interface::sensor_data sd;
	sd.position.x = east;
	sd.position.y = north;
	sd.position.z = alt;

	double deg2rad = M_PI / 180.0;

	sd.attitude.x = static_cast<double>(XPLMGetDataf(attitudeRefs_[0])) * deg2rad;
	sd.attitude.y = static_cast<double>(XPLMGetDataf(attitudeRefs_[1])) * deg2rad;

	double yaw = static_cast<double>(XPLMGetDataf(attitudeRefs_[2])) * deg2rad;

	if (yaw > M_PI)
		yaw -= 2 * M_PI;
	sd.attitude.z = yaw;

	sd.velocity.linear.x = static_cast<double>(XPLMGetDataf(velocityRefs_[0]));
	sd.velocity.linear.y = -static_cast<double>(XPLMGetDataf(velocityRefs_[2]));
	sd.velocity.linear.z = static_cast<double>(XPLMGetDataf(velocityRefs_[1]));

	sd.ground_speed = sqrt(pow(sd.velocity.linear.x,2) + pow(sd.velocity.linear.y,2) + pow(sd.velocity.linear.z,2));

	sd.velocity.angular.x = static_cast<double>(XPLMGetDataf(angularRateRefs_[0])) * deg2rad;
	sd.velocity.angular.y = -static_cast<double>(XPLMGetDataf(angularRateRefs_[1])) * deg2rad;
	sd.velocity.angular.z = static_cast<double>(XPLMGetDataf(angularRateRefs_[2])) * deg2rad;

	Vector3 accelerationInertial;
	accelerationInertial[0] = static_cast<double>(XPLMGetDataf(accelerationRefs_[0]));
	accelerationInertial[1] = static_cast<double>(XPLMGetDataf(accelerationRefs_[2]));
	accelerationInertial[2] = static_cast<double>(XPLMGetDataf(accelerationRefs_[1]));

	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(-sd.attitude.x, Vector3::UnitX())
			* Eigen::AngleAxisd(-sd.attitude.y, Vector3::UnitY())
			* Eigen::AngleAxisd(-sd.attitude.z, Vector3::UnitZ());

	Vector3 accelerationBody = m * accelerationInertial;

	sd.acceleration.linear.x = accelerationBody[0];
	sd.acceleration.linear.y = accelerationBody[1];
	sd.acceleration.linear.z = accelerationBody[2];

	sd.sequenceNr = sequenceNr_++;

	sd.header.stamp = ros::Time::now();

	sensorDataPublisher_.publish(sd);

	ros::spinOnce();
}

void
XPlaneRosNode::actuate(const simulation_interface::actuation& act)
{
	if (!autopilotActive_)
		return;

	std::cout << "Override" << std::endl;

	float throt = (static_cast<float>(act.throttleOutput) + 1) / 2;
	float throttle[] =
	{ throt, throt, throt, throt, throt, throt, throt, throt };
	float roll = static_cast<float>(act.rollOutput);
	float pitch = static_cast<float>(act.pitchOutput);
	float yaw = static_cast<float>(act.yawOutput);
	XPLMSetDatavf(throttleRef_, throttle, 0, 8);
	XPLMSetDataf(joystickAttitudeRef_[0], roll);
	XPLMSetDataf(joystickAttitudeRef_[1], pitch);
	XPLMSetDataf(joystickAttitudeRef_[2], yaw);

}
