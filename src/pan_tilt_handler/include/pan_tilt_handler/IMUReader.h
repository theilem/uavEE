#ifndef IMUREADER_H
#define IMUREADER_H

#include <boost/property_tree/ptree.hpp>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/Time.h>
#include "deviceclass.h"

class PanTiltHandler;
class IScheduler;

struct PanTiltIMUData
{
	Vector3 attitude;
	Vector3 acceleration;
	Vector3 attitudeRate;
	TimePoint timepoint;
};

struct PanTiltGPSData
{
	Vector3 position;
	Vector3 velocity;
};

struct PanTiltData
{
	PanTiltIMUData imuData;
	PanTiltGPSData gpsData;
};


class IMUReader : public IAggregatableObject, public IRunnableObject
{
public:
	static constexpr TypeId typeId = "imureader";

	static std::shared_ptr<IMUReader>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	~IMUReader();

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

	bool
	isIdle() const;
private:
	std::string portName_;
	void
	readIMUData();
	//XsPortInfo port_;
	DeviceClass device_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<PanTiltHandler> panTiltHander_;
	bool idlemode_;
};

#endif // IMUREADER_H
