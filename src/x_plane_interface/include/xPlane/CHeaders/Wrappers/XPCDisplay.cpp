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
#include "XPCDisplay.h"

XPCKeySniffer::XPCKeySniffer(int inBeforeWindows) :
		mBeforeWindows(inBeforeWindows)
{
	XPLMRegisterKeySniffer(KeySnifferCB, mBeforeWindows, reinterpret_cast<void *>(this));
}

XPCKeySniffer::~XPCKeySniffer()
{
	XPLMUnregisterKeySniffer(KeySnifferCB, mBeforeWindows, reinterpret_cast<void *>(this));
}

int
XPCKeySniffer::KeySnifferCB(char inCharKey, XPLMKeyFlags inFlags, char inVirtualKey,
		void * inRefCon)
{
	XPCKeySniffer * me = reinterpret_cast<XPCKeySniffer *>(inRefCon);
	return me->HandleKeyStroke(inCharKey, inFlags, inVirtualKey);
}

XPCWindow::XPCWindow(int inLeft, int inTop, int inRight, int inBottom, int inIsVisible)
{
	mWindow = XPLMCreateWindow(inLeft, inTop, inRight, inBottom, inIsVisible, DrawCB, HandleKeyCB,
			MouseClickCB, reinterpret_cast<void *>(this));
}

XPCWindow::~XPCWindow()
{
	XPLMDestroyWindow(mWindow);
}

void
XPCWindow::GetWindowGeometry(int * outLeft, int * outTop, int * outRight, int * outBottom)
{
	XPLMGetWindowGeometry(mWindow, outLeft, outTop, outRight, outBottom);
}

void
XPCWindow::SetWindowGeometry(int inLeft, int inTop, int inRight, int inBottom)
{
	XPLMSetWindowGeometry(mWindow, inLeft, inTop, inRight, inBottom);
}

int
XPCWindow::GetWindowIsVisible(void)
{
	return XPLMGetWindowIsVisible(mWindow);
}

void
XPCWindow::SetWindowIsVisible(int inIsVisible)
{
	XPLMSetWindowIsVisible(mWindow, inIsVisible);
}

void
XPCWindow::TakeKeyboardFocus(void)
{
	XPLMTakeKeyboardFocus(mWindow);
}

void
XPCWindow::BringWindowToFront(void)
{
	XPLMBringWindowToFront(mWindow);
}

int
XPCWindow::IsWindowInFront(void)
{
	return XPLMIsWindowInFront(mWindow);
}

void
XPCWindow::DrawCB(XPLMWindowID inWindowID, void * inRefcon)
{
	XPCWindow * me = reinterpret_cast<XPCWindow *>(inRefcon);
	me->DoDraw();
}

void
XPCWindow::HandleKeyCB(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags, char inVirtualKey,
		void * inRefcon, int losingFocus)
{
	XPCWindow * me = reinterpret_cast<XPCWindow *>(inRefcon);
	if (losingFocus)
		me->LoseFocus();
	else
		me->HandleKey(inKey, inFlags, inVirtualKey);
}

int
XPCWindow::MouseClickCB(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse,
		void * inRefcon)
{
	XPCWindow * me = reinterpret_cast<XPCWindow *>(inRefcon);
	return me->HandleClick(x, y, inMouse);
}
