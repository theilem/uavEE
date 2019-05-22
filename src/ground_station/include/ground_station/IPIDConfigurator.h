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
 *   @date [dd/mm/yyyy] 3/21/2018
 *   @brief UAV Ground Station PID Configuration Interface
 */

#ifndef IPIDCONFIGURATOR_H
#define IPIDCONFIGURATOR_H

#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"

/**
 * @brief The PIDInfo struct contains PID Params and a human readable name
 */
struct PIDInfo
{
	std::string name;
	Control::PID::Parameters params;
	PIDInfo(std::string n, Control::PID::Parameters p) :
			name(n), params(p)
	{
	}
};

using PIDParametersMap = std::map<PIDs, PIDInfo>;

/**
 * @brief   The IPIDConfigurator class serves as an interface for all classes
 *          that have PID tuning functionality.
 */
class IPIDConfigurator
{
public:
	virtual
	~IPIDConfigurator()
	{
	}

	/**
	 * @brief   tunePID tunes a certain PID with Kp, kI, kD, Imax, and ff gains
	 * @param   tunePID PIDTuning struct containing integer PID controller ID and gains
	 * @return  true if request was recieved
	 */
	virtual bool
	tunePID(const PIDTuning& tunePID) = 0;

	/**
	 * @brief   getPIDMap gets PID map of flying aircraft's cascade
	 * @return  map from integer PID representation to PIDInfo struct containing
	 *          human readable name and PID parameters
	 */
	virtual const PIDParametersMap&
	getPIDMap() const = 0;
};

#endif // IPIDCONFIGURATOR_H
