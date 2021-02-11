//
// Created by seedship on 7/6/20.
//

#ifndef UAVEE_XPLANEPLUGIN_H
#define UAVEE_XPLANEPLUGIN_H

int
resetConfig(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void*);

int
startNode(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void*);

int
setAutopilotState(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);

int
setLoggingState(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);

void
addDirectoryInfo(XPLMMenuID &configMenu);

void
configSelector(void*, void* iRef);

int
refreshConfigInfo(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);

int
generateConfig(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void*);

#endif //UAVEE_XPLANEPLUGIN_H
