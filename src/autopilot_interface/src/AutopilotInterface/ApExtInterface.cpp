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
/*
 * AutopilotInterface.cpp
 *
 *  Created on: May 25, 2018
 *      Author: mircot
 */

#include <autopilot_interface/detail/uavAPConversions.h>
#include "autopilot_interface/detail/UTMToLatLong.h"
#include <uavAP/API/ap_ext/ap_ext.h>
#include <uavAP/API/ap_ext/ApExtManager.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include <uavAP/Core/SensorData.h>
#include "autopilot_interface/AutopilotInterface/ApExtInterface/ApExtInterface.h"
#include <cmath>

#include <functional>
#include <iostream>

#include <boost/property_tree/json_parser.hpp>

ApExtInterface::ApExtInterface() :
		lastSequenceNr_(0), numChannels_(0), traceSeqNr_(false), useAirspeed_(false), useEuler_(
				false), externalGps_(false), internalImu_(false)
{

}

ApExtInterface::~ApExtInterface()
{
	ap_ext_teardown();
}

std::shared_ptr<ApExtInterface>
ApExtInterface::create(const Configuration& config)
{
	auto ap = std::make_shared<ApExtInterface>();
	ap->configure(config);
	return ap;
}

bool
ApExtInterface::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);

	Configuration alvoloConfig;
	pm.add("alvolo_config", alvoloConfig_, true);
	pm.add("alvolo_config", alvoloConfig, true);

	PropertyMapper<Configuration> pmAlvolo(alvoloConfig);

	Configuration interfaceConfig;
	pmAlvolo.add("interface", interfaceConfig, true);

	PropertyMapper<Configuration> pmInterface(interfaceConfig);

//	pmInterface.add<ChannelMixing>("", channelMixing_, true);
//	pmInterface.add<unsigned int>("num_output_channel", numChannels_, true);
	pmInterface.add<bool>("internal_imu", internalImu_, false);
	pmInterface.add<bool>("external_gps", externalGps_, false);
	pmInterface.add<bool>("use_airspeed", useAirspeed_, false);
	pmInterface.add<bool>("use_euler", useEuler_, false);
	pmInterface.add<bool>("trace_seq_nr", traceSeqNr_, false);
	return pm.map() && pmAlvolo.map();
}

void
ApExtInterface::notifyAggregationOnUpdate(const Aggregator& agg)
{
}

bool
ApExtInterface::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		//Setup dataSample
		if (internalImu_)
			dataSample_.int_imu_sample = new int_imu_sample_t;
		else
			dataSample_.imu_sample = new imu_sample_t;

		if (externalGps_)
			dataSample_.pic_sample = new pic_sample_t;

		if (useAirspeed_)
			dataSample_.airs_sample = new airs_sample_t;

		setConfigPath(alvoloConfig_);

		if (ap_ext_setup() != 0)
			return true;
		break;
	}
	case RunStage::NORMAL:
	{
		break;
	}
	case RunStage::FINAL:
	{
		break;
	}
	default:
		break;
	}
	return false;
}

boost::signals2::connection
ApExtInterface::subscribeOnControllerOut(const OnControllerOut::slot_type& out)
{
	return onControllerOut_.connect(out);
}

void
ApExtInterface::sendSensorData(const SensorData& sensorData)
{

	double roll = sensorData.attitude[0];
	double pitch = sensorData.attitude[1];
	double yaw = sensorData.attitude[2];

	//Add gravity to acceleration
	Vector3 gravityInertial(0, 0, 9.81);
	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(-roll, Vector3::UnitX()) * Eigen::AngleAxisd(-pitch, Vector3::UnitY());
	Vector3 gravityBody = m * gravityInertial;

	auto attitude = eulerToQuaternion(sensorData.attitude);

	/* North, East Down coordinates - computed aside */
	/* Compute north, east, down coordinates */
	int zone;
	char hemi;
	double latitude, longitude;

	UTMtoLL(22, sensorData.position[1], sensorData.position[0], 0, latitude, longitude);

	/* Timestamp */

	if (internalImu_)
	{
		auto imu = dataSample_.int_imu_sample;

		// Sequence number
		imu->imu_pkt = static_cast<unsigned long>(sensorData.sequenceNr);

		if (useEuler_)
		{
			imu->imu_euler_roll = roll;
			imu->imu_euler_pitch = pitch;
			imu->imu_euler_yaw = yaw;
		}
		else
		{
			imu->imu_quat_w = attitude.w();
			imu->imu_quat_x = attitude.x();
			imu->imu_quat_y = attitude.y();
			imu->imu_quat_z = attitude.z();
		}

		/* Rotation rate */
		imu->imu_rot_x = sensorData.angularRate[0];
		imu->imu_rot_y = sensorData.angularRate[1];
		imu->imu_rot_z = sensorData.angularRate[2];

		imu->imu_accel_x = sensorData.acceleration[0] - gravityBody[0];
		imu->imu_accel_y = sensorData.acceleration[1] - gravityBody[1];
		imu->imu_accel_z = sensorData.acceleration[2] - gravityBody[2];
	}
	else
	{
		auto imu = dataSample_.imu_sample;

		// Sequence number
		imu->imu_pkt = static_cast<unsigned long>(sensorData.sequenceNr);

		if (useEuler_)
		{
			imu->imu_euler_roll = roll;
			imu->imu_euler_pitch = pitch;
			imu->imu_euler_yaw = yaw;
		}
		else
		{
			imu->imu_quat_w = attitude.w();
			imu->imu_quat_x = attitude.x();
			imu->imu_quat_y = attitude.y();
			imu->imu_quat_z = attitude.z();
		}

		/* Rotation rate */
		imu->imu_rot_x = sensorData.angularRate[0];
		imu->imu_rot_y = sensorData.angularRate[1];
		imu->imu_rot_z = sensorData.angularRate[2];

		imu->imu_accel_x = sensorData.acceleration[0] - gravityBody[0];
		imu->imu_accel_y = sensorData.acceleration[1] - gravityBody[1];
		imu->imu_accel_z = sensorData.acceleration[2] - gravityBody[2];

		if (!externalGps_)
		{
			imu->imu_lon = longitude;
			imu->imu_lat = latitude;
			imu->imu_alt = sensorData.position[2];

			// Velocity
			imu->imu_vel_x = sensorData.velocity[0];
			imu->imu_vel_y = sensorData.velocity[1];
			imu->imu_vel_z = sensorData.velocity[2];

			//Valid flag for GPS fix
			imu->valid_flags = 0x80;

			auto sinceEpoch = sensorData.timestamp.time_since_epoch();

			imu->imu_time_year = std::chrono::duration_cast<Hours>(sinceEpoch).count() / (365 * 24);
			imu->imu_time_month = std::chrono::duration_cast<Hours>(sinceEpoch).count() % (365 * 24) / (365 * 24 / 12); // A bit wrong
			imu->imu_time_day = std::chrono::duration_cast<Hours>(sinceEpoch).count() % (365 * 24) / 24;
			imu->imu_time_hour = std::chrono::duration_cast<Hours>(sinceEpoch).count() % 24;
			imu->imu_time_minute = std::chrono::duration_cast<Minutes>(sinceEpoch).count() % 60;
			imu->imu_time_second = std::chrono::duration_cast<Seconds>(sinceEpoch).count() % 24;;
			imu->imu_time_nano = std::chrono::duration_cast<Nanoseconds>(sinceEpoch).count() % (int)1e9;
		}
	}

	if (externalGps_)
	{
		auto& pos = dataSample_.pic_sample->gps_sample.position;

		double horSpeed = sensorData.velocity.head(2).norm();
		double courseRad = std::asin(static_cast<double>(sensorData.velocity[0] / horSpeed));

		pos.course_gnd = courseRad * 180.0 / M_PI;
		pos.speed_gnd_kh = horSpeed * 3.6;
		pos.vert_velocity = sensorData.velocity[2];

		pos.longitude = longitude;
		pos.latitude = latitude;
		pos.msl_altitude = sensorData.position[2];

		//TODO timestamp
	}

	if (useAirspeed_)
	{
		//TODO airspeed
	}

	sendDataSample(dataSample_);
}

void
ApExtInterface::sendDataSample(const data_sample_t& sample)
{
	int sensResult = ap_ext_sense(&sample);

	if (sensResult != 0)
		APLOG_ERROR << "ap_ext_sense returned with " << sensResult;
}

void
ApExtInterface::getActuation()
{
//	unsigned long pwmSeq[numChannels_ + 1];
//	int actResult = ap_ext_actuate(pwmSeq, numChannels_ + 1);
//
//	if (actResult != 0)
//	{
//		APLOG_WARN << "Something went wrong getting actuation data.";
//		return;
//	}
//
//	uint32_t seq = static_cast<uint32_t>(pwmSeq[numChannels_]);
////Do only write when there was no update in sensordata
//	if (lastSequenceNr_ == seq)
//		return;
//	lastSequenceNr_ = seq;
//
//	unsigned long pwm[numChannels_];
//	std::copy(pwmSeq, pwmSeq + numChannels_, pwm);
//	std::vector<unsigned long> pwmVec(pwm, pwm + numChannels_);
//
//	auto channels = servoMapping_.unmap(pwmVec);
//	ControllerOutput control = channelMixing_.unmixChannels(channels);
//
//	control.sequenceNr = seq;

//	onControllerOut_(control);
}
