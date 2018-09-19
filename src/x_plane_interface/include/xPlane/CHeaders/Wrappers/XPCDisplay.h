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
#ifndef _XPCDisplay_h_
#define _XPCDisplay_h_

#include "XPLMDisplay.h"

class XPCKeySniffer
{
public:

	XPCKeySniffer(int inBeforeWindows);
	virtual
	~XPCKeySniffer();

	virtual int
	HandleKeyStroke(char inCharKey, XPLMKeyFlags inFlags, char inVirtualKey)=0;

private:

	int mBeforeWindows;

	static int
	KeySnifferCB(char inCharKey, XPLMKeyFlags inFlags, char inVirtualKey, void * inRefCon);
};

class XPCWindow
{
public:

	XPCWindow(int inLeft, int inTop, int inRight, int inBottom, int inIsVisible);
	virtual
	~XPCWindow();

	virtual void
	DoDraw(void)=0;
	virtual void
	HandleKey(char inKey, XPLMKeyFlags inFlags, char inVirtualKey)=0;
	virtual void
	LoseFocus(void)=0;
	virtual int
	HandleClick(int x, int y, XPLMMouseStatus inMouse)=0;

	void
	GetWindowGeometry(int * outLeft, int * outTop, int * outRight, int * outBottom);
	void
	SetWindowGeometry(int inLeft, int inTop, int inRight, int inBottom);
	int
	GetWindowIsVisible(void);
	void
	SetWindowIsVisible(int inIsVisible);
	void
	TakeKeyboardFocus(void);
	void
	BringWindowToFront(void);
	int
	IsWindowInFront(void);

private:

	XPLMWindowID mWindow;

	static void
	DrawCB(XPLMWindowID inWindowID, void * inRefcon);
	static void
	HandleKeyCB(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags, char inVirtualKey,
			void * inRefcon, int losingFocus);
	static int
	MouseClickCB(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse, void * inRefcon);

};

#endif
