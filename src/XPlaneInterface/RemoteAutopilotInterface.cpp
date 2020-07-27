//
// Created by seedship on 7/3/20.
//

#include <uavAP/Core/SensorData.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>
#include <uavAP/Core/DataHandling/Content.hpp>
#include "uavEE/XPlaneInterface/RemoteAutopilotInterface.h"

#include <cpsCore/Utilities/IDC/IDC.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>

void
RemoteAutopilotInterface::setSensorData(const SensorData& sd)
{
	if (auto dp = get<DataPresentation>())
	{
		auto packet = dp->serialize(sd);
		dp->addHeader(packet, Content::SENSOR_DATA);
		autopilotSender_.sendPacket(packet);
	}
	else
		CPSLOG_ERROR << "Cannot send packet, dp missing";
}

void
RemoteAutopilotInterface::setServoData(const ServoData& sd)
{
	if (auto dp = get<DataPresentation>())
	{
		auto packet = dp->serialize(sd);
		dp->addHeader(packet, Content::SERVO_DATA);
		autopilotSender_.sendPacket(packet);
	}
	else
		CPSLOG_ERROR << "Cannot send packet, dp missing";
}

void
RemoteAutopilotInterface::setPowerData(const PowerData& pd)
{
	if (auto dp = get<DataPresentation>())
	{
		auto packet = dp->serialize(pd);
		dp->addHeader(packet, Content::POWER_DATA);
		autopilotSender_.sendPacket(packet);
	}
	else
		CPSLOG_ERROR << "Cannot send packet, dp missing";
}

boost::signals2::connection
RemoteAutopilotInterface::subscribeOnControllerOut(const OnControllerOut::slot_type& slot)
{
	return onControllerOut_.connect(slot);
}

boost::signals2::connection
RemoteAutopilotInterface::subscribeOnAdvancedControl(
		const boost::signals2::slot<void(const AdvancedControl&), boost::function<void(const AdvancedControl&)>>& slot)
{
	CPSLOG_ERROR << "Not Implemented!";
	return boost::signals2::connection();
}

bool
RemoteAutopilotInterface::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSetAll())
			{
				CPSLOG_ERROR << "Missing deps";
				return true;
			}
			break;
		}
		case RunStage::NORMAL:
		{
			auto idc = get<IDC>();

			autopilotSender_ = idc->createSender("to_uavap");

			idc->subscribeOnPacket("to_uavee", [this](const auto& p)
			{ this->onAPPacket(p); });
			break;
		}
		default:
			break;
	}
	return false;
}

void
RemoteAutopilotInterface::onAPPacket(const Packet& packet)
{
	if (auto dp = get<DataPresentation>())
	{
		auto p = packet;
		auto content = dp->extractHeader<Content>(p);
		switch (content)
		{
			case Content::CONTROLLER_OUTPUT:
			{
				onControllerOut_(dp->deserialize<ControllerOutput>(p));
			}
			default:
			{
				CPSLOG_WARN << "RAI does not know how to handle content: " << static_cast<int>(content);
				break;
			}
		}
	}
	else
		CPSLOG_ERROR << "Cannot handle ap packet, dp missing";
}
