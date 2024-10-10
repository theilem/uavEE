//
// Created by seedship on 7/6/20.
//

#ifndef UAVEE_XPLANEPLUGIN_H
#define UAVEE_XPLANEPLUGIN_H

#ifdef UNIX
#define LIN
#endif

#include "xPlane/CHeaders/XPLM/XPLMDefs.h"
#include "xPlane/CHeaders/XPLM/XPLMMenus.h"

void
handler(void* mRef, void* iRef);

void
registerCommand(XPLMMenuID menuID, const char* name, const char* description,
				int (* func)(XPLMCommandRef, XPLMCommandPhase, void*), int inBefore = 0, void* inRefcon = nullptr);

void
addCommandToMenu(const char* name, const char* description, int (* func)(XPLMCommandRef, XPLMCommandPhase, void*),
				 int inBefore = 0, void* inRefcon = nullptr);

int
resetConfig(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);

void
startNode(void* mRef, void* iRef);

int
setAutopilotState(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);


XPLMMenuID
addDirectoryInfo(XPLMMenuID parentMenu, int menuIdx);


int
refreshConfigInfo(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);

int
generateConfig(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);

void
populateConfig();

#endif //UAVEE_XPLANEPLUGIN_H
