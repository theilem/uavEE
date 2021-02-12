//
// Created by seedship on 7/6/20.
//

#ifndef UAVEE_XPLANEPLUGIN_H
#define UAVEE_XPLANEPLUGIN_H


PLUGIN_API int
XPluginStart(char* outName, char* outSig, char* outDesc);

PLUGIN_API void
XPluginStop(void);

PLUGIN_API void
XPluginDisable(void);

PLUGIN_API int
XPluginEnable(void);

PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID, intptr_t, void*);

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
