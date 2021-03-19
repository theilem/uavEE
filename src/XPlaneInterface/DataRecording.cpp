//
// Created by mirco on 12.03.21.
//

#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include <uavEE/XPlaneInterface/XPlanePlugin.h>
#include <uavEE/XPlaneInterface/XPlaneInterface.h>
#include <fstream>
#include "uavEE/XPlaneInterface/DataRecording.h"
#include <boost/token_functions.hpp>
#include <boost/tokenizer.hpp>
#include <uavAP/Core/SensorData.h>
#include <iomanip>

static DataRecording* dataRecording;

int
startRecordingStatic(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon)
{
	if (inPhase != xplm_CommandBegin)
		return 0;
	if (!dataRecording)
		return -1;
	dataRecording->startRecording();
	return 0;
}

int
stopRecordingStatic(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon)
{
	if (inPhase != xplm_CommandBegin)
		return 0;
	if (!dataRecording)
		return -1;
	dataRecording->stopRecording();
	return 0;
}

bool
DataRecording::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSetAll())
			{
				CPSLOG_ERROR << "Missing deps";
				return true;
			}
			if (params.headerFile().empty())
			{
				CPSLOG_ERROR << "Header file missing";
				return true;
			}
			if (params.logFile().empty())
			{
				CPSLOG_ERROR << "Log file missing";
				return true;
			}

			dataRecording = this;
			addCommandToMenu("Start Recording", "Starts recording of sensor data to logFile", startRecordingStatic);
			addCommandToMenu("Stop Recording", "Stops recording of sensor data to logFile", stopRecordingStatic);
			prepareLogging();
			break;
		}
		case RunStage::NORMAL:
		{
			break;

		}
		default:
			break;
	}
	return false;
}

void
DataRecording::startRecording()
{
	auto sched = get<IScheduler>();

	logFile_.open(params.logFile());
	loggingEvent_ = sched->schedule([this]
									{ log(); }, Milliseconds(params.period()), Milliseconds(params.period()));

	CPSLOG_DEBUG << "Logging to " << params.logFile() << " started";
}

void
DataRecording::prepareLogging()
{

	const std::unordered_map<std::string, SensorEnum> sensorMap{
			{"IMU Packet Number",               SensorEnum::SEQUENCE_NR},
			{"Int. IMU IMU Packet Number",      SensorEnum::SEQUENCE_NR},
			{"Euler Angles phi (deg) ",         SensorEnum::ATTITUDE_X},
			{"Euler Angles theta (deg) ",       SensorEnum::ATTITUDE_Y},
			{"Euler Angles psi (deg) ",         SensorEnum::ATTITUDE_Z},
			{"Acceleration x (m/s2) ",          SensorEnum::ACCELERATION_X},
			{"Acceleration y (m/s2) ",          SensorEnum::ACCELERATION_Y},
			{"Acceleration z (m/s2) ",          SensorEnum::ACCELERATION_Z},
			{"Rotation Rate x (rad/s) ",        SensorEnum::ANGULAR_RATE_X},
			{"Rotation Rate y (rad/s) ",        SensorEnum::ANGULAR_RATE_Y},
			{"Rotation Rate z (rad/s) ",        SensorEnum::ANGULAR_RATE_Z},
			{"Longitude (deg)",                 SensorEnum::POSITION_X},
			{"Latitude (deg)",                  SensorEnum::POSITION_Y},
			{"Altitude (m)",                    SensorEnum::POSITION_Z},
			{"Int. IMU Longitude (deg)",        SensorEnum::POSITION_X},
			{"Int. IMU Latitude (deg)",         SensorEnum::POSITION_Y},
			{"Int. IMU Altitude (m)",           SensorEnum::POSITION_Z},
			{"Velocity x (m/s) ",               SensorEnum::VELOCITY_X},
			{"Velocity y (m/s) ",               SensorEnum::VELOCITY_Y},
			{"Velocity z (m/s) ",               SensorEnum::VELOCITY_Z},
			{"Course over Ground (deg)",        SensorEnum::ATTITUDE_Z},
			{"Speed over Ground (m/s)",         SensorEnum::GROUND_SPEED},
			{"Vertical Velocity (m/s)",         SensorEnum::VELOCITY_Z},
			{"UTC Time - Day",                  SensorEnum::INVALID},
			{"UTC Time - Hour",                 SensorEnum::INVALID},
			{"UTC Time - Minute",               SensorEnum::INVALID},
			{"UTC Time - Month",                SensorEnum::INVALID},
			{"UTC Time - Nanoseconds",          SensorEnum::INVALID},
			{"UTC Time - Second",               SensorEnum::INVALID},
			{"UTC Time - Year",                 SensorEnum::INVALID},
			{"Int. IMU UTC Time - Day",         SensorEnum::INVALID},
			{"Int. IMU UTC Time - Hour",        SensorEnum::INVALID},
			{"Int. IMU UTC Time - Minute",      SensorEnum::INVALID},
			{"Int. IMU UTC Time - Month",       SensorEnum::INVALID},
			{"Int. IMU UTC Time - Nanoseconds", SensorEnum::INVALID},
			{"Int. IMU UTC Time - Second",      SensorEnum::INVALID},
			{"Int. IMU UTC Time - Year",        SensorEnum::INVALID},
			{"GPS Fix",                         SensorEnum::HAS_GPS_FIX},
			{"Air Speed (m/s)",                 SensorEnum::AIR_SPEED}
	};

	std::ifstream headerFile(params.headerFile());
	if (!headerFile.is_open())
	{
		CPSLOG_ERROR << "Header file at " << params.headerFile() << " cannot be opened";
		return;
	}
	std::string header;
	std::getline(headerFile, header);

	boost::char_separator<char> sep{";"};
	boost::tokenizer<boost::char_separator<char>> token{header, sep};

	for (auto& tok : token)
	{
		auto it = sensorMap.find(tok);
		if (it == sensorMap.end())
		{
			CPSLOG_WARN << tok << " cannot be logged";
			logOrder_.push_back(SensorEnum::INVALID);
			continue;
		}
		logOrder_.push_back(it->second);
	}

}

void
DataRecording::log()
{
	auto xplane = get<XPlaneInterface>();
	auto sd = xplane->getSensorData();
	for (const auto& sensor : logOrder_)
	{
		if (sensor == SensorEnum::INVALID)
			logFile_ << "0;";
		else
			logFile_ << std::setprecision(10) << enumAccess<FloatingType>(sd, sensor) << ";";
	}
	logFile_ << std::endl;
}

void
DataRecording::stopRecording()
{
	if (logFile_.is_open())
	{
		CPSLOG_DEBUG << "Logging to " << params.logFile() << " stopped";
		loggingEvent_.cancel();
		logFile_.close();
		return;
	}
	CPSLOG_ERROR << "Logging not started";
}
