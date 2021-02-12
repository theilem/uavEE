//
// Created by seedship on 2/11/21.
//

#ifndef UAVEE_UTILS_H
#define UAVEE_UTILS_H


int
registerCommand(XPLMMenuID menuID, const char* name, const char* description,
				XPLMCommandCallback_f func, int inBefore = 0, void* inRefcon = NULL);

// NOTE apparently all X-Plane plugins run in the same process space (the same as X-Plane), so
// setting different log levels in different plugins will overwrite each other
void
setLogLevel();

constexpr auto emptyHandler = [](void* mRef, void* iRef){};


#endif //UAVEE_UTILS_H
