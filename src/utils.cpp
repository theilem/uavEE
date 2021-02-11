//
// Created by seedship on 2/11/21.
//

#include "uavEE/utils.h"

/**
 * Adds a command to a plugin menu
 * @param menuID - Menu to add the command to
 * @param name - Name that will show up in the menu
 * @param description - Description when mouse hovers over menu option
 * @param func - Function pointer. Should have return type int and take params (XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon)
 * @param inBefore - See https://developer.x-plane.com/sdk/XPLMRegisterCommandHandler/
 * @param inRefcon - Void parameter to pass params to function
 * @return Index of generated menu item
 */
int
registerCommand(XPLMMenuID menuID, const char* name, const char* description,
				XPLMCommandCallback_f func, XPLMCommandPhase inBefore, void* inRefcon)
{
	XPLMCommandRef cmd = XPLMCreateCommand(name, description);
	XPLMRegisterCommandHandler(cmd, func, inBefore, inRefcon);

	return XPLMAppendMenuItemWithCommand(menuID, name, cmd);
}