//
// Created by seedship on 7/6/20.
//

#ifndef UAVEE_XPLANEPLUGIN_H
#define UAVEE_XPLANEPLUGIN_H


void
handler(void* mRef, void* iRef);

void
registerCommand(XPLMMenuID menuID, const char* name, const char* description,
				int (* func)(XPLMCommandRef, XPLMCommandPhase, void*), int inBefore = 0, void* inRefcon = NULL);

int
resetConfig(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);

int
startNode(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);

int
setAutopilotState(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);


XPLMMenuID
addDirectoryInfo(XPLMMenuID parentMenu, int menuIdx);

void
configSelector(void* mRef, void* iRef);

int
refreshConfigInfo(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);

int
generateConfig(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);

void
populateConfig();

#endif //UAVEE_XPLANEPLUGIN_H
