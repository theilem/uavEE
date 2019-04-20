////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavEE.
//
// uavEE is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavEE is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/**
 *   @author Richard Nai, rnai2@illinois.edu
 *   @file IDataSignals.h
 *   @date [dd/mm/yyyy] 3/18/2018
 *   @brief UAV Ground Station signals interface file
 */

#ifndef IDATASIGNALS_H
#define IDATASIGNALS_H

#include <QtCore/QObject>
#include <uavAP/Core/Frames/VehicleOneFrame.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>
#include <uavAP/FlightAnalysis/StateAnalysis/Metrics.h>
#include <uavAP/MissionControl/ManeuverPlanner/Override.h>
#include <simulation_interface/sensor_data.h>
#include <simulation_interface/actuation.h>

#include "radio_comm/pidstati.h"
#include "radio_comm/serialized_proto.h"
#include "radio_comm/serialized_object.h"

class Mission;
class Trajectory;
class LocalPlannerStatus;

/**
 * @brief The IDataSignals interface defines all the signals
 */
class IDataSignals: public QObject
{
Q_OBJECT
public:

	virtual
	~IDataSignals()
	{
	}

signals:

	/**
	 * @brief onActuationData signal is called when new actuation data is received
	 */
	virtual void
	onActuationData(const simulation_interface::actuation&) = 0;

	/**
	 * @brief onSensorData signal is called when new sensor data is received
	 */
	virtual void
	onSensorData(const simulation_interface::sensor_data&) = 0;

	/**
	 * @brief onMission signal is called when new Mission is received
	 */
	virtual void
	onMission(const Mission&) = 0;

	/**
	 * @brief onTrajectory signal is called when new Trajectory is received
	 */
	virtual void
	onTrajectory(const Trajectory&) = 0;

	/**
	 * @brief onPathSectionChange signal is called when path section changes, with parameter defining what the current path section is
	 */
	virtual void
	onPathSectionChange(int) = 0;

	/**
	 * @brief onLocalPlannerStatus is called whenever a new LocalPlannerStatus is received
	 */
	virtual void
	onLocalPlannerStatus(const radio_comm::serialized_proto&) = 0;

	/**
	 * @brief onLocalFrame is called whenever new local frame is received
	 */
	virtual void
	onLocalFrame(const VehicleOneFrame&) = 0;

	/**
	 * @brief onOverride is called whenever new override is received
	 */
	virtual void
	onOverride(const Override&) = 0;

	/**
	 * @brief onControllerOutputTrim is called whenever new controller output trim is received
	 */
	virtual void
	onControllerOutputTrim(const ControllerOutput&) = 0;

	/**
	 * @brief onPIDStati is called whenever new PID stati is received
	 */
	virtual void
	onPIDStati(const radio_comm::pidstati&) = 0;

	/**
	 * @brief onInspectingMetrics is called whenever new inspecting metrics is received
	 */
	virtual void
	onInspectingMetrics(const SteadyStateMetrics&) = 0;

};

#endif // IDATASIGNALS_H
