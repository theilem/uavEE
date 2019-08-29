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
 * LogParser.cpp
 *
 *  Created on: May 17, 2018
 *      Author: mircot
 */
#include "autopilot_interface/LogParser.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <unordered_map>

#include <boost/unordered_map.hpp>
#include <boost/assign/list_of.hpp>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include <uavAP/Core/Scheduler/IScheduler.h>

#include <autopilot_interface/AutopilotInterface/IAutopilotInterface.h>
#include <boost/token_functions.hpp>
#include <boost/tokenizer.hpp>

LogParser::LogParser() :
		dataIdxToString_(
		{
		{ DataID::IMU_PKT, "IMU Packet Number" },
		{ DataID::EULER_ROLL, "Euler Angles phi (deg) " },
		{ DataID::EULER_PITCH, "Euler Angles theta (deg) " },
		{ DataID::EULER_YAW, "Euler Angles psi (deg) " },
		{ DataID::QUAT_W, " Quaternion w" },
		{ DataID::QUAT_X, " Quaternion x" },
		{ DataID::QUAT_Y, " Quaternion y" },
		{ DataID::QUAT_Z, " Quaternion z" },
		{ DataID::ACC_X, "Acceleration x (m/s2) " },
		{ DataID::ACC_Y, "Acceleration y (m/s2) " },
		{ DataID::ACC_Z, "Acceleration z (m/s2) " },
		{ DataID::ROLL_RATE, "Rotation Rate x (rad/s) " },
		{ DataID::PITCH_RATE, "Rotation Rate y (rad/s) " },
		{ DataID::YAW_RATE, "Rotation Rate z (rad/s) " },
		{ DataID::LONGITUDE, "Longitude (deg)" },
		{ DataID::LATITUDE, "Latitude (deg)" },
		{ DataID::ALTITUDE, "Altitude (m)" },
		{ DataID::VELOCITY_X, "Velocity x (m/s) " },
		{ DataID::VELOCITY_Y, "Velocity y (m/s) " },
		{ DataID::VELOCITY_Z, "Velocity z (m/s) " },
		{ DataID::COURSE_OVER_GROUND, "Course over Ground (deg)" },
		{ DataID::SPEED_OVER_GROUND, "Speed over Ground (m/s)" },
		{ DataID::VERTICAL_VEL, "Vertical Velocity (m/s)" },
		{ DataID::TIME_DAY, "UTC Time - Day" },
		{ DataID::TIME_HOUR, "UTC Time - Hour" },
		{ DataID::TIME_MINUTE, "UTC Time - Minute" },
		{ DataID::TIME_MONTH, "UTC Time - Month" },
		{ DataID::TIME_NANOSEC, "UTC Time - Nanoseconds" },
		{ DataID::TIME_SEC, "UTC Time - Second" },
		{ DataID::TIME_YEAR, "UTC Time - Year" },
		{ DataID::GPS_FIX, "GPS Fix" },
		{ DataID::AIRSPEED, "Air Speed (m/s)" } }), internalImu_(false), externalGps_(false), useEuler_(
				false)
{
}

bool
LogParser::setupLog()
{
	logFile_.open(logFilePath_);

	if (!logFile_.is_open())
		return false;

	std::ifstream headerFile(logHeaderPath_);
	std::string header;
	std::getline(headerFile, header);

	boost::char_separator<char> sep
	{ ";" };
	boost::tokenizer<boost::char_separator<char>> token
	{ header, sep };

	std::unordered_map<std::string, int> headerIdcs;
	int k = 0;
	for (auto& tok : token)
	{
		headerIdcs.insert(std::make_pair(tok, k));
		k++;
	}

	APLOG_TRACE << "Map log header to data indeces";
	for (size_t i = 0; i < static_cast<size_t>(DataID::NUM_OF_IDX); ++i)
	{
		auto it = dataIdxToString_.find(static_cast<DataID>(i));
		if (it == dataIdxToString_.end())
		{
			APLOG_ERROR << "Cannot find string for Data Index " << i;
			return false;
		}

		auto index = headerIdcs.find(it->second);
		if (index != headerIdcs.end())
		{
			dataIDIndeces_[i] = index->second;
			APLOG_TRACE << it->second << ": " << dataIDIndeces_[i];
		}
	}
	return true;
}

void
LogParser::initializeSample()
{
	dataSample_.imu_sample = new imu_sample_t;
	dataSample_.int_imu_sample = new int_imu_sample_t;
	dataSample_.pic_sample = new pic_sample_t;
	dataSample_.airs_sample = new airs_sample_t;
}

std::shared_ptr<LogParser>
LogParser::create(const Configuration& config)
{
	auto lp = std::make_shared<LogParser>();
	lp->configure(config);
	return lp;
}

bool
LogParser::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);

	Configuration alvoloConfig;
	pm.add("alvolo_config", alvoloConfig, true);
	pm.add("log_file_path", logFilePath_, true);
	pm.add("log_header_path", logHeaderPath_, true);
	pm.add("period", period_, true);

	PropertyMapper<Configuration> pmAlvolo(alvoloConfig);

	Configuration interfaceConfig;
	pmAlvolo.add("interface", interfaceConfig, true);

	PropertyMapper<Configuration> pmInterface(interfaceConfig);

	pmInterface.add<bool>("internal_imu", internalImu_, false);
	pmInterface.add<bool>("external_gps", externalGps_, false);
	pmInterface.add<bool>("use_euler", useEuler_, false);

	return pm.map() && pmAlvolo.map();
}

void
LogParser::notifyAggregationOnUpdate(const Aggregator& agg)
{
	autopilotInterface_.setFromAggregationIfNotSet(agg);
	scheduler_.setFromAggregationIfNotSet(agg);
}

bool
LogParser::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "Log parser scheduler missing.";
			return true;
		}
		if (!autopilotInterface_.isSet())
		{
			APLOG_ERROR << "Log parser autopilot interface missing";
			return true;
		}

		if (!setupLog())
		{
			APLOG_ERROR << "Setting up log file failed";
			return true;
		}
		initializeSample();
		break;
	}
	case RunStage::NORMAL:
	{
		auto scheduler = scheduler_.get();
		scheduler->schedule(std::bind(&LogParser::createAndSendSample, this), Milliseconds(0),
				period_);
		break;
	}
	default:
		break;
	}
	return false;

}

void
LogParser::createAndSendSample()
{
	if (!logFile_.is_open())
	{
		APLOG_ERROR << "Log file not open. Cannot populate data sample.";
		return;
	}

	std::string line;
	std::getline(logFile_, line);
	std::vector<std::string> items;

	boost::split(items, line, boost::is_any_of(";"), boost::token_compress_off);

	if (internalImu_)
	{
		auto intimu = dataSample_.int_imu_sample;

		if (intimu)
		{
			intimu->imu_pkt = getValue<unsigned long>(items, DataID::IMU_PKT);

			if (useEuler_)
			{
				intimu->imu_euler_roll = getValue<double>(items, DataID::EULER_ROLL);
				intimu->imu_euler_pitch = getValue<double>(items, DataID::EULER_PITCH);
				intimu->imu_euler_yaw = getValue<double>(items, DataID::EULER_YAW);
			}
			else
			{
				intimu->imu_quat_w = getValue<double>(items, DataID::QUAT_W);
				intimu->imu_quat_x = getValue<double>(items, DataID::QUAT_X);
				intimu->imu_quat_y = getValue<double>(items, DataID::QUAT_Y);
				intimu->imu_quat_z = getValue<double>(items, DataID::QUAT_Z);
			}

			intimu->imu_accel_x = getValue<double>(items, DataID::ACC_X);
			intimu->imu_accel_y = getValue<double>(items, DataID::ACC_Y);
			intimu->imu_accel_z = getValue<double>(items, DataID::ACC_Z);

			intimu->imu_rot_x = getValue<double>(items, DataID::ROLL_RATE);
			intimu->imu_rot_y = getValue<double>(items, DataID::PITCH_RATE);
			intimu->imu_rot_z = getValue<double>(items, DataID::YAW_RATE);
		}
	}
	else
	{
		auto imu = dataSample_.imu_sample;

		if (imu)
		{
			imu->imu_pkt = getValue<unsigned long>(items, DataID::IMU_PKT);

			if (useEuler_)
			{
				imu->imu_euler_roll = getValue<double>(items, DataID::EULER_ROLL);
				imu->imu_euler_pitch = getValue<double>(items, DataID::EULER_PITCH);
				imu->imu_euler_yaw = getValue<double>(items, DataID::EULER_YAW);
			}
			else
			{
				imu->imu_quat_w = getValue<double>(items, DataID::QUAT_W);
				imu->imu_quat_x = getValue<double>(items, DataID::QUAT_X);
				imu->imu_quat_y = getValue<double>(items, DataID::QUAT_Y);
				imu->imu_quat_z = getValue<double>(items, DataID::QUAT_Z);
			}

			imu->imu_accel_x = getValue<double>(items, DataID::ACC_X);
			imu->imu_accel_y = getValue<double>(items, DataID::ACC_Y);
			imu->imu_accel_z = getValue<double>(items, DataID::ACC_Z);

			imu->imu_rot_x = getValue<double>(items, DataID::ROLL_RATE);
			imu->imu_rot_y = getValue<double>(items, DataID::PITCH_RATE);
			imu->imu_rot_z = getValue<double>(items, DataID::YAW_RATE);

			if (!externalGps_)
			{
				imu->imu_lon = getValue<double>(items, DataID::LONGITUDE);
				imu->imu_lat = getValue<double>(items, DataID::LATITUDE);
				imu->imu_alt = getValue<double>(items, DataID::ALTITUDE);

				// Velocity
				imu->imu_vel_x = getValue<double>(items, DataID::VELOCITY_X);
				imu->imu_vel_y = getValue<double>(items, DataID::VELOCITY_Y);
				imu->imu_vel_z = getValue<double>(items, DataID::VELOCITY_Z);

				//Valid flag for GPS fix
				imu->valid_flags = static_cast<unsigned long>(0x80);

				imu->imu_time_year = getValue<double>(items, DataID::TIME_YEAR);
				imu->imu_time_month = getValue<double>(items, DataID::TIME_MONTH);
				imu->imu_time_day = getValue<double>(items, DataID::TIME_DAY);
				imu->imu_time_hour = getValue<double>(items, DataID::TIME_HOUR);
				imu->imu_time_minute = getValue<double>(items, DataID::TIME_MINUTE);
				imu->imu_time_second = getValue<double>(items, DataID::TIME_SEC);
				imu->imu_time_nano = getValue<double>(items, DataID::TIME_NANOSEC);
			}
		}

		if (externalGps_)
		{
			auto pic = dataSample_.pic_sample;

			if (pic)
			{
				//only update if gps is valid:
				if (getValue<int>(items, DataID::GPS_FIX) == 7)
				{
					auto& pos = pic->gps_sample.position;

					pos.flags = 7;
					pos.longitude = getValue<double>(items, DataID::LONGITUDE);
					pos.latitude = getValue<double>(items, DataID::LATITUDE);
					pos.msl_altitude = getValue<double>(items, DataID::ALTITUDE);

					pos.course_gnd = getValue<double>(items, DataID::COURSE_OVER_GROUND);
					pos.speed_gnd_kh = getValue<double>(items, DataID::SPEED_OVER_GROUND);
					pos.vert_velocity = getValue<double>(items, DataID::VERTICAL_VEL);
				}
			}
		}
	}

	auto autopilotInterface = autopilotInterface_.get();
	if (!autopilotInterface)
	{
		APLOG_ERROR << "Cannot send sample, autopilot interface missing.";
		return;
	}

	autopilotInterface->sendDataSample(dataSample_);

}
