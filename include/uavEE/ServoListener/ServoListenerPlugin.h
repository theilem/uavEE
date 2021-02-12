//
// Created by seedship on 2/11/21.
//

#ifndef UAVEE_SERVOLISTENERPLUGIN_H
#define UAVEE_SERVOLISTENERPLUGIN_H

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
startPrint(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void*);

int
stopPrint(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void*);

#endif //UAVEE_SERVOLISTENERPLUGIN_H
