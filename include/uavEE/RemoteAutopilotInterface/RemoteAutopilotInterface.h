//
// Created by seedship on 7/3/20.
//

#ifndef UAVEE_REMOTEAUTOPILOTINTERFACE_H
#define UAVEE_REMOTEAUTOPILOTINTERFACE_H

#include <cpsCore/Aggregation/AggregatableObject.hpp>
class IDC;

#include "uavEE/RemoteAutopilotInterface/RemoteAutopilotInterface.h"

class RemoteAutopilotInterface : public AggregatableObject<IDC>, public IRunnableObject
{
public:
	void
	setSensorData(const SensorData& sd);

	void
	setServoData(const ServoData& sd);

	void
	setPowerData(const PowerData& pd);

	boost::signals2::connection
	subscribeOnControllerOut(const OnControllerOut::slot_type& slot);

private:
};


#endif //UAVEE_REMOTEAUTOPILOTINTERFACE_H
