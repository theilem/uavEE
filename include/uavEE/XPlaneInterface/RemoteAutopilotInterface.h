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

	static constexpr auto typeId = "remote_autopilot_api";

	void
	setSensorData(const SensorData& sd) override;

	void
	setServoData(const ServoData& sd) override;

	void
	setPowerData(const PowerData& pd) override;

	boost::signals2::connection
	subscribeOnControllerOut(const OnControllerOut::slot_type& slot) override;

	virtual boost::signals2::connection
	subscribeOnAdvancedControl(const OnAdvancedControl::slot_type& slot) override;

	bool
	run(RunStage stage) override;

private:

	void
	onAPPacket(const Packet& packet);

	OnControllerOut onControllerOut_;

	IDCSender autopilotSender_;
};


#endif //UAVEE_REMOTEAUTOPILOTINTERFACE_H
