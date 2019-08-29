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
﻿#ifndef PANTILTPIDCASCADE_H
#define PANTILTPIDCASCADE_H
#include <uavAP/FlightControl/Controller/PIDController/IPIDCascade.h>
#include <uavAP/FlightControl/Controller/ControlElements/ControlEnvironment.h>
#include <uavAP/FlightControl/Controller/PIDController/detail/PIDHandling.h>
#include <simulation_interface/sensor_data.h>
#include <ground_station/IPIDConfigurator.h>
#include <uavAP/Core/SensorData.h>
#include "pan_tilt_handler/IMUReader.h"

struct PanTiltTarget
{
	double heading; //this will be current - target bounded by 180
	double pitch;
	double headingoffset;
	double pitchoffset;
	PanTiltTarget() :
			heading(0), pitch(0), headingoffset(0), pitchoffset(0)
	{
	}
	PanTiltTarget(double h, double p, double ho, double po) :
			heading(h), pitch(p), headingoffset(ho), pitchoffset(po)
	{
	}
};

struct PanTiltOutput
{
	double panOut;
	double tiltOut;
};

enum class PanTiltPIDs
{
	HEADING = 0, PITCH, HEADINGRATE, PITCHRATE,
};

const static std::map<PanTiltPIDs, std::string> PanTiltPIDBimapLeft =
{
{ PanTiltPIDs::HEADING, "heading" },
{ PanTiltPIDs::PITCH, "pitch" },
{ PanTiltPIDs::HEADINGRATE, "headingrate" },
{ PanTiltPIDs::PITCHRATE, "pitchrate" }, };

const static std::map<std::string, PanTiltPIDs> PanTiltPIDBimapRight =
{
{ "heading", PanTiltPIDs::HEADING },
{ "pitch", PanTiltPIDs::PITCH },
{ "headingrate", PanTiltPIDs::HEADINGRATE },
{ "pitchrate", PanTiltPIDs::PITCHRATE }, };

class PanTiltPIDCascade: public IPIDCascade
{
public:
	PanTiltPIDCascade(PanTiltData* panTiltSD, simulation_interface::sensor_data* aircraftSD,
			PanTiltTarget* target, PanTiltOutput* antennaOutput);

	bool
	configure(const Configuration& config) override;

	bool
	tunePID(int pid, const Control::PID::Parameters& params) override;

	bool
	tuneRollBounds(double, double) override; //unneeded here

	bool
	tunePitchBounds(double, double) override; //unneeded here

	std::map<int, PIDStatus>
	getPIDStatus() override;

	void
	evaluate() override;

	const PIDParametersMap&
	getPIDParamMap() const;

private:

	Control::ControlEnvironment controlEnv_;
	std::shared_ptr<Control::PID> headingPID_;
	std::shared_ptr<Control::PID> pitchPID_;
	std::shared_ptr<Control::PID> headingRatePID_;
	std::shared_ptr<Control::PID> pitchRatePID_;
	PanTiltData* panTiltSensorData;
	simulation_interface::sensor_data* aircraftSensorData;
	PanTiltTarget* antennaTarget;
	PIDParametersMap pidParams_;
	double headingDeviation_; // targetYaw-sensorDataYaw
	double pitchDeviation_; // targetYaw-sensorDataYaw
	double theta_;
	double thetaDot_;
	double psiDot_;
};

#endif // PANTILTPIDCASCADE_H
