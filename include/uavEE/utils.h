//
// Created by seedship on 2/11/21.
//

#ifndef UAVEE_UTILS_H
#define UAVEE_UTILS_H

#include "xPlane/CHeaders/XPLM/XPLMMenus.h"

int
registerCommand(XPLMMenuID menuID, const char* name, const char* description,
				XPLMCommandCallback_f func, int inBefore = 0, void* inRefcon = nullptr);

constexpr auto emptyHandler = [](void* mRef, void* iRef){};


#endif //UAVEE_UTILS_H
