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
#include "XPCBroadcaster.h"
#include "XPCListener.h"

XPCBroadcaster::XPCBroadcaster() :
		mIterator(NULL)
{
}

XPCBroadcaster::~XPCBroadcaster()
{
	ListenerVector::iterator iter;
	mIterator = &iter;
	for (iter = mListeners.begin(); iter != mListeners.end(); ++iter)
	{
		(*iter)->BroadcasterRemoved(this);
	}
}

void
XPCBroadcaster::AddListener(XPCListener * inListener)
{
	mListeners.push_back(inListener);
	inListener->BroadcasterAdded(this);
}

void
XPCBroadcaster::RemoveListener(XPCListener * inListener)
{
	ListenerVector::iterator iter = std::find(mListeners.begin(), mListeners.end(), inListener);
	if (iter == mListeners.end())
		return;

	if (mIterator != NULL)
	{
		if (*mIterator >= iter)
			(*mIterator)--;
	}

	mListeners.erase(iter);
	inListener->BroadcasterRemoved(this);
}

void
XPCBroadcaster::BroadcastMessage(int inMessage, void * inParam)
{
	ListenerVector::iterator iter;
	mIterator = &iter;
	for (iter = mListeners.begin(); iter != mListeners.end(); ++iter)
	{
		(*iter)->ListenToMessage(inMessage, inParam);
	}
	mIterator = NULL;
}
