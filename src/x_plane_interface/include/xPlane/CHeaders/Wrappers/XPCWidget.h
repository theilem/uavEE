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
#ifndef _XPCWidget_h_
#define _XPCWidget_h_

#include <vector>
#include <algorithm>
#include "XPWidgets.h"

class XPCWidget;

class XPCWidgetAttachment
{
public:

	virtual int
	HandleWidgetMessage(XPCWidget * inObject, XPWidgetMessage inMessage, XPWidgetID inWidget,
			intptr_t inParam1, intptr_t inParam2)=0;

};

class XPCWidget
{
public:

	XPCWidget(int inLeft, int inTop, int inRight, int inBottom, bool inVisible,
			const char * inDescriptor, bool inIsRoot, XPWidgetID inParent, XPWidgetClass inClass);
	XPCWidget(XPWidgetID inWidget, bool inOwnsWidget);
	virtual
	~XPCWidget();

	void
	SetOwnsWidget(bool inOwnsWidget);
	void
	SetOwnsChildren(bool inOwnsChildren);

	operator XPWidgetID() const;

	XPWidgetID
	Get(void) const;

	void
	AddAttachment(XPCWidgetAttachment * inAttachment, bool inOwnsAttachment, bool inPrefilter);
	void
	RemoveAttachment(XPCWidgetAttachment * inAttachment);

	virtual int
	HandleWidgetMessage(XPWidgetMessage inMessage, XPWidgetID inWidget, intptr_t inParam1,
			intptr_t inParam2);

private:

	static int
	WidgetCallback(XPWidgetMessage inMessage, XPWidgetID inWidget, intptr_t inParam1,
			intptr_t inParam2);

	typedef std::pair<XPCWidgetAttachment *, bool> AttachmentInfo;
	typedef std::vector<AttachmentInfo> AttachmentVector;

	AttachmentVector mAttachments;
	XPWidgetID mWidget;
	bool mOwnsChildren;
	bool mOwnsWidget;

	XPCWidget();
	XPCWidget(const XPCWidget&);
	XPCWidget&
	operator=(const XPCWidget&);

};

#endif
