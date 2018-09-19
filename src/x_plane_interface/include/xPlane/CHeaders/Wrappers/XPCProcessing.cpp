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
		XPLMSetFlightLoopCallbackInterval(FlightLoopCB, mCallbackTime, 1/*relative to now*/,
				reinterpret_cast<void *>(this));
}

void
XPCProcess::StartProcessCycles(int inCycles)
{
	mCallbackTime = -inCycles;
	if (!mInCallback)
		XPLMSetFlightLoopCallbackInterval(FlightLoopCB, mCallbackTime, 1/*relative to now*/,
				reinterpret_cast<void *>(this));
}

void
XPCProcess::StopProcess(void)
{
	mCallbackTime = 0;
	if (!mInCallback)
		XPLMSetFlightLoopCallbackInterval(FlightLoopCB, mCallbackTime, 1/*relative to now*/,
				reinterpret_cast<void *>(this));
}

float
XPCProcess::FlightLoopCB(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop,
		int inCounter, void * inRefcon)
{
	XPCProcess * me = reinterpret_cast<XPCProcess *>(inRefcon);
	me->mInCallback = true;
	me->DoProcessing(inElapsedSinceLastCall, inElapsedTimeSinceLastFlightLoop, inCounter);
	me->mInCallback = false;
	return me->mCallbackTime;
}
