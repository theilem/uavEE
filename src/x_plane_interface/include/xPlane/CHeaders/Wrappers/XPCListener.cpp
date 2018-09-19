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
#include "XPCListener.h"
#include "XPCBroadcaster.h"

XPCListener::XPCListener()
{
}

XPCListener::~XPCListener()
{
	while (!mBroadcasters.empty())
		mBroadcasters.front()->RemoveListener(this);
}

void
XPCListener::BroadcasterAdded(XPCBroadcaster * inBroadcaster)
{
	mBroadcasters.push_back(inBroadcaster);
}

void
XPCListener::BroadcasterRemoved(XPCBroadcaster * inBroadcaster)
{
	BroadcastVector::iterator iter = std::find(mBroadcasters.begin(), mBroadcasters.end(),
			inBroadcaster);
	if (iter != mBroadcasters.end())
		mBroadcasters.erase(iter);
}
