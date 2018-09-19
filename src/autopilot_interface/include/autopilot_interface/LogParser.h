////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavEE.
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
 * LogParser.h
 *
 *  Created on: May 17, 2018
 *      Author: mircot
 */

#ifndef AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_LOGPARSER_H_
#define AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_LOGPARSER_H_
#include <boost/property_tree/ptree.hpp>
#include <boost/unordered_map.hpp>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/API/ap_ext/ap_ext.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/Core/Time.h>
#include <unordered_map>

class IAutopilotInterface;
class IScheduler;

class LogParser : public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "log_parser";

	LogParser();

	static std::shared_ptr<LogParser>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	enum class DataID
	{
		EULER_ROLL,
		EULER_PITCH,
		EULER_YAW,
		QUAT_W,
		QUAT_X,
		QUAT_Y,
		QUAT_Z,
		ACC_X,
		ACC_Y,
		ACC_Z,
		ROLL_RATE,
		PITCH_RATE,
		YAW_RATE,
		LATITUDE,
		LONGITUDE,
		ALTITUDE,
		COURSE_OVER_GROUND,
		SPEED_OVER_GROUND,
		VERTICAL_VEL,
		AIRSPEED,
		GPS_FIX,
		NUM_OF_IDX
	};

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

private:

	bool
	setupLog();

	void
	initializeSample();

	void
	createAndSendSample();

	template <typename T>
	T
	getValue(const std::vector<std::string>& items, DataID id);

	template <typename T>
	T
	getValue(const std::vector<std::string>& items, DataID id, T defaultVal);

	struct DataIndexHash
	{
		template <typename T>
		std::size_t operator()(T t) const
		{
			return static_cast<std::size_t>(t);
		}
	};

	const std::unordered_map<DataID, const char*, DataIndexHash> dataIdxToString_;
	data_sample_t dataSample_;

	ObjectHandle<IAutopilotInterface> autopilotInterface_;
	ObjectHandle<IScheduler> scheduler_;

	std::string logFilePath_;
	std::string logHeaderPath_;
	Duration period_;

	std::ifstream logFile_;
	int dataIDIndeces_[static_cast<size_t>(DataID::NUM_OF_IDX)] = {-1};


};

template<typename T>
inline T
LogParser::getValue(const std::vector<std::string>& items, DataID id)
{
	int idx = dataIDIndeces_[static_cast<size_t>(id)];

	if (idx == -1)
		throw std::range_error("provided data id not available");

	T ret;
	std::stringstream (items.at(idx)) >> ret;
	return ret;
}

template<typename T>
inline T
LogParser::getValue(const std::vector<std::string>& items, DataID id, T defaultVal)
{
	try
	{
		return getValue<T>(items, id);
	}
	catch (std::range_error& err)
	{
		return defaultVal;
	}
}

#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_LOGPARSER_H_ */
