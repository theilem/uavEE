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
 * IAutopilotInterface.h
 *
 *  Created on: May 26, 2018
 *      Author: sim
 */

#ifndef AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_IAUTOPILOTINTERFACE_H_
#define AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_IAUTOPILOTINTERFACE_H_

#include <uavAP/FlightControl/Controller/ControllerOutput.h>
#include <boost/signals2/signal.hpp>
#include <uavAP/API/ap_ext/ap_ext.h>
#include <uavAP/Core/SensorData.h>

class IAutopilotInterface
{
public:

	virtual
	~IAutopilotInterface() = default;

	using OnControllerOut = boost::signals2::signal<void (const ControllerOutput&)>;

	virtual boost::signals2::connection
	subscribeOnControllerOut(const OnControllerOut::slot_type& slot) = 0;

	virtual void
	sendSensorData(const SensorData& sd) = 0;

	virtual void
	sendDataSample(const data_sample_t& sample) = 0;
};


#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_IAUTOPILOTINTERFACE_H_ */
