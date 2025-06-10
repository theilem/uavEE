//
// Created by seedship on 6/19/20.
//

#ifndef UAVEE_XPLANEINTERFACE_H
#define UAVEE_XPLANEINTERFACE_H
#include <mutex>
#include <optional>
#include <cpsCore/cps_object>
#include <cpsCore/Utilities/Scheduler/Event.h>
#include <uavAP/Core/SensorData.h>

#include "xPlane/CHeaders/XPLM/XPLMDataAccess.h"
#include <uavGS/XPlane/XPlaneManager.h>

struct Teleportation;
class DataPresentation;
class IPC;
class IAutopilotAPI;
class IScheduler;
struct ControllerOutput;


class XPlaneInterface
    : public AggregatableObject<IScheduler, IAutopilotAPI, IPC, DataPresentation>,
      public IRunnableObject
{
public:
    static constexpr TypeId typeId = "xplanenode";

    XPlaneInterface();

    bool
    run(RunStage stage) override;

    void
    setAutopilotActive(bool active);

    const SensorData&
    getSensorData() const;

    void
    teleportTo(const Teleportation& tele);

    float
    flightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop,
                     int inCounter, void* inRefcon);

private:
    void
    processData();

    void
    actuate(const ControllerOutput& out);

    int sensorFrequency_;
    std::mutex dataMutex_;
    XPLMDataRef positionRefs_[3];
    XPLMDataRef velocityRefs_[3];
    XPLMDataRef airSpeedRef_;
    XPLMDataRef accelerationRefs_[3];
    XPLMDataRef attitudeRefs_[3];
    XPLMDataRef angleOfAttackRef_;
    XPLMDataRef angleOfSideslipRef_;
    XPLMDataRef angularRateRefs_[3];
    XPLMDataRef gpsFixRef_;
    XPLMDataRef batteryVoltageRef_;
    XPLMDataRef batteryCurrentRef_;
    XPLMDataRef aileronRef_;
    XPLMDataRef elevatorRef_;
    XPLMDataRef rudderRef_;
    XPLMDataRef throttleRef_;
    XPLMDataRef rpmRef_;
    XPLMDataRef joystickOverrideRef_[2];
    XPLMDataRef joystickAttitudeRef_[3];
    XPLMDataRef course_;
    XPLMDataRef temp_;
    XPLMDataRef pressure_;

    XPLMDataRef fuel_;

    //TODO update scheduling based on sim speed
    XPLMDataRef simSpeed_;
    XPLMDataRef timestamp_;

    TimePoint lastSensorUpdate_;

    uint32_t sequenceNumber_{0};

    int zone_;
    char hemi_;

    ServoData servoData_;
    SensorData sensorData_;
    PowerData powerData_;

    std::optional<Teleportation> teleportTo_;

    Event sensorDataEvent_;
};

#endif //UAVEE_XPLANEINTERFACE_H
