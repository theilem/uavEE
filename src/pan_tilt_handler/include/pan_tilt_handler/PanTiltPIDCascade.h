#ifndef PANTILTPIDCASCADE_H
#define PANTILTPIDCASCADE_H
#include <uavAP/FlightControl/Controller/PIDController/IPIDCascade.h>
#include <uavAP/FlightControl/Controller/ControlElements/ControlEnvironment.h>
#include <uavAP/FlightControl/Controller/PIDController/PIDHandling.h>
#include <simulation_interface/sensor_data.h>
#include <ground_station/IPIDConfigurator.h>
#include <uavAP/Core/SensorData.h>
#include "pan_tilt_handler/IMUReader.h"

struct PanTiltTarget
{
	double heading; //this will be current - target bounded by 180
	double pitch;
	double headingoffset;
	double pitchoffset;
	PanTiltTarget() : heading(0), pitch(0), headingoffset(0), pitchoffset(0) {}
	PanTiltTarget(double h, double p, double ho, double po) : heading(h), pitch(p), headingoffset(ho), pitchoffset(po) {}
};

struct PanTiltOutput
{
	double panOut;
	double tiltOut;
};

enum class PanTiltPIDs
{
	HEADING = 0,
	PITCH,
	HEADINGRATE,
	PITCHRATE,
};

const static std::map<PanTiltPIDs, std::string> PanTiltPIDBimapLeft =
{
	{PanTiltPIDs::HEADING, "heading"},
	{PanTiltPIDs::PITCH, "pitch"},
	{PanTiltPIDs::HEADINGRATE, "headingrate"},
	{PanTiltPIDs::PITCHRATE, "pitchrate"},
};

const static std::map<std::string, PanTiltPIDs> PanTiltPIDBimapRight =
{
	{"heading", PanTiltPIDs::HEADING},
	{"pitch", PanTiltPIDs::PITCH},
	{"headingrate", PanTiltPIDs::HEADINGRATE},
	{"pitchrate", PanTiltPIDs::PITCHRATE},
};

class PanTiltPIDCascade: public IPIDCascade
{
public:
	PanTiltPIDCascade(PanTiltData* panTiltSD, simulation_interface::sensor_data* aircraftSD, PanTiltTarget* target, PanTiltOutput* antennaOutput);

	bool
	configure(const boost::property_tree::ptree& config) override;

	bool
	tunePID(PIDs pid, const Control::PID::Parameters& params) override;

	bool
	tuneRollBounds(double, double) override; //unneeded here

	bool
	tunePitchBounds(double, double) override; //unneeded here

	std::map<PIDs, PIDStatus>
	getPIDStatus() override;

	void
	evaluate() override;

	const PIDParametersMap&
	getPIDParamMap() const;

private:

	Control::ControlEnvironment controlEnv_;
	std::shared_ptr<Control::PID> headingPID_;
	std::shared_ptr<Control::PID> pitchPID_;
	std::shared_ptr<Control::PID> headingRatePID_;
	std::shared_ptr<Control::PID> pitchRatePID_;
	PanTiltData* panTiltSensorData;
	simulation_interface::sensor_data* aircraftSensorData;
	PanTiltTarget* antennaTarget;
	PIDParametersMap pidParams_;
	double headingDeviation_; // targetYaw-sensorDataYaw
	double pitchDeviation_; // targetYaw-sensorDataYaw
	double theta_;
	double thetaDot_;
	double psiDot_;
};



#endif // PANTILTPIDCASCADE_H
