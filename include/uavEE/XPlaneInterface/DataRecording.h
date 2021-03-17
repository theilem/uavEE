//
// Created by mirco on 12.03.21.
//

#ifndef UAVEE_DATARECORDING_H
#define UAVEE_DATARECORDING_H

#include <cpsCore/cps_object>
#include <cpsCore/Configuration/Parameter.hpp>
#include <uavAP/Core/SensorData.h>
#include "uavEE/XPlaneInterface/XPlanePlugin.h"

class IScheduler;
class XPlaneInterface;

struct DataRecordingParams
{

	Parameter<unsigned> period = {10, "period", true};
	Parameter<std::string> headerFile = {"", "header_file", true};
	Parameter<std::string> logFile = {"", "log_file", true};

	template<typename Config>
	void
	configure(Config& c)
	{
		c & period;
		c & headerFile;
		c & logFile;
	}
};

static int
startRecordingStatic(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);

static int
stopRecordingStatic(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);

class DataRecording
		: public AggregatableObject<IScheduler, XPlaneInterface>, public ConfigurableObject<DataRecordingParams>, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "data_recording";

	bool
	run(RunStage stage) override;

	void
	startRecording();

	void
	stopRecording();
private:

	void
	prepareLogging();

	void
	log();

	std::ofstream logFile_;
	std::vector<SensorEnum> logOrder_;
	Event loggingEvent_;

};

#endif //UAVEE_DATARECORDING_H
