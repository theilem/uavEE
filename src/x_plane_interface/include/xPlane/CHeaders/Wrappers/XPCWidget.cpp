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
#include "XPCWidget.h"

XPCWidget::XPCWidget(int inLeft, int inTop, int inRight, int inBottom, bool inVisible,
		const char * inDescriptor, bool inIsRoot, XPWidgetID inParent, XPWidgetClass inClass) :
		mWidget(NULL), mOwnsChildren(false), mOwnsWidget(true)
{
	mWidget = XPCreateWidget(inLeft, inTop, inRight, inBottom, inVisible ? 1 : 0, inDescriptor,
			inIsRoot ? 1 : 0, inIsRoot ? NULL : inParent, inClass);

	XPSetWidgetProperty(mWidget, xpProperty_Object, reinterpret_cast<intptr_t>(this));
	XPAddWidgetCallback(mWidget, WidgetCallback);
}

XPCWidget::XPCWidget(XPWidgetID inWidget, bool inOwnsWidget) :
		mWidget(inWidget), mOwnsChildren(false), mOwnsWidget(inOwnsWidget)
{
	XPSetWidgetProperty(mWidget, xpProperty_Object, reinterpret_cast<intptr_t>(this));
	XPAddWidgetCallback(mWidget, WidgetCallback);
}

XPCWidget::~XPCWidget()
{
	if (mOwnsWidget)
		XPDestroyWidget(mWidget, mOwnsChildren ? 1 : 0);
}

void
XPCWidget::SetOwnsWidget(bool inOwnsWidget)
{
	mOwnsWidget = inOwnsWidget;
}

void
XPCWidget::SetOwnsChildren(bool inOwnsChildren)
{
	mOwnsChildren = inOwnsChildren;
}

XPCWidget::operator XPWidgetID() const
{
	return mWidget;
}

XPWidgetID
XPCWidget::Get(void) const
{
	return mWidget;
}

void
XPCWidget::AddAttachment(XPCWidgetAttachment * inAttachment, bool inOwnsAttachment,
		bool inPrefilter)
{
	if (inPrefilter)
	{
		mAttachments.insert(mAttachments.begin(), AttachmentInfo(inAttachment, inOwnsAttachment));
	}
	else
	{
		mAttachments.push_back(AttachmentInfo(inAttachment, inOwnsAttachment));
	}
}

void
XPCWidget::RemoveAttachment(XPCWidgetAttachment * inAttachment)
{
	for (AttachmentVector::iterator iter = mAttachments.begin(); iter != mAttachments.end(); ++iter)
	{
		if (iter->first == inAttachment)
		{
			mAttachments.erase(iter);
			return;
		}
	}
}

int
XPCWidget::HandleWidgetMessage(XPWidgetMessage inMessage, XPWidgetID inWidget, intptr_t inParam1,
		intptr_t inParam2)
{
	return 0;
}

int
XPCWidget::WidgetCallback(XPWidgetMessage inMessage, XPWidgetID inWidget, intptr_t inParam1,
		intptr_t inParam2)
{
	XPCWidget * me = reinterpret_cast<XPCWidget *>(XPGetWidgetProperty(inWidget, xpProperty_Object,
			NULL));
	if (me == NULL)
		return 0;

	for (AttachmentVector::iterator iter = me->mAttachments.begin(); iter != me->mAttachments.end();
			++iter)
	{
		int result = iter->first->HandleWidgetMessage(me, inMessage, inWidget, inParam1, inParam2);
		if (result != 0)
			return result;
	}

	return me->HandleWidgetMessage(inMessage, inWidget, inParam1, inParam2);
}
