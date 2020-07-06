//
// Created by seedship on 7/3/20.
//

#ifndef UAVEE_REMOTEAUTOPILOTINTERFACE_H
#define UAVEE_REMOTEAUTOPILOTINTERFACE_H

#include <cpsCore/cps_object>
#include <uavAP/API/IAutopilotAPI.h>
#include <cpsCore/Utilities/IDC/IDCSender.h>

class IDC;
class DataPresentation;

class RemoteAutopilotInterface
		: public AggregatableObject<IDC, DataPresentation>,
		  public IRunnableObject,
//		  public ConfigurableObject<RemoteAutopilotInterfaceParams>,
		  public IAutopilotAPI
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

	bool
	run(RunStage stage) override;

private:

	void
	onAPPacket(const Packet& packet);

	OnControllerOut onControllerOut_;

	IDCSender autopilotSender_;
};


#endif //UAVEE_REMOTEAUTOPILOTINTERFACE_H
