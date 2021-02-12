//
// Created by seedship on 2/12/21.
//

#ifndef UAVEE_SERVOLISTENERHELPER_H
#define UAVEE_SERVOLISTENERHELPER_H

#include <cpsCore/Framework/StaticHelper.h>

#include <cpsCore/Utilities/Scheduler/SchedulerFactory.h>
#include <cpsCore/Utilities/TimeProvider/TimeProviderFactory.h>
#include "uavEE/ServoListener/ServoListener.h"

using ServoListenerDefaultHelper = StaticHelper<SchedulerFactory,
		TimeProviderFactory,
		ServoListener>;

using ServoListenerHelper = StaticHelper<ServoListenerDefaultHelper>;

#endif //UAVEE_SERVOLISTENERHELPER_H
