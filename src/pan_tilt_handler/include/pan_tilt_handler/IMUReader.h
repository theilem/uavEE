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
﻿#ifndef IMUREADER_H
#define IMUREADER_H

#include <boost/property_tree/ptree.hpp>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/Time.h>
#include "deviceclass.h"

class PanTiltHandler;
class IScheduler;

struct PanTiltIMUData
{
	Vector3 attitude;
	Vector3 acceleration;
	Vector3 attitudeRate;
	TimePoint timepoint;
};

struct PanTiltGPSData
{
	Vector3 position;
	Vector3 velocity;
};

struct PanTiltData
{
	PanTiltIMUData imuData;
	PanTiltGPSData gpsData;
};

class IMUReader: public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "imu_reader";
	//IMUReader(const std::string& port);

	static std::shared_ptr<IMUReader>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	~IMUReader();

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

	bool
	isIdle() const;

private:
	std::string portName_;
	void
	readIMUData();
	//XsPortInfo port_;
	DeviceClass device_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<PanTiltHandler> panTiltHander_;
	bool idlemode_;
};

#endif // IMUREADER_H
