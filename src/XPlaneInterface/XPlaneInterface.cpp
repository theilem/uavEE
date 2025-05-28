//
// Created by seedship on 6/19/20.
//

#include <cpsCore/Utilities/Scheduler/IScheduler.h>

#include <uavAP/API/ap_ext/latLongToUTM.h>
#include <uavAP/API/IAutopilotAPI.h>
#include <uavAP/FlightControl/Controller/AdvancedControl.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>

#include "uavEE/XPlaneInterface/XPlaneInterface.h"

XPlaneInterface::XPlaneInterface() :
    sensorFrequency_(100)
{
    positionRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/latitude");
    positionRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/longitude");
    positionRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/elevation");

    velocityRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/local_vx");
    velocityRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/local_vy");
    velocityRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/local_vz");

    airSpeedRef_ = XPLMFindDataRef("sim/flightmodel/position/true_airspeed");

    accelerationRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/local_ax");
    accelerationRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/local_ay");
    accelerationRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/local_az");

    attitudeRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/phi");
    attitudeRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/theta");
    attitudeRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/psi");

    //	angleOfAttackRef_ = XPLMFindDataRef("sim/flightmodel2/misc/AoA_angle_degrees");
    angleOfAttackRef_ = XPLMFindDataRef("sim/flightmodel/position/alpha");
    angleOfSideslipRef_ = XPLMFindDataRef("sim/flightmodel/position/beta");

    angularRateRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/P");
    angularRateRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/Q");
    angularRateRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/R");

    gpsFixRef_ = XPLMFindDataRef("sim/cockpit2/radios/actuators/gps_power");

    batteryVoltageRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_bat_volt");
    batteryCurrentRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_bat_amp");

    aileronRef_ = XPLMFindDataRef("sim/flightmodel/controls/wing1l_ail1def");
    elevatorRef_ = XPLMFindDataRef("sim/flightmodel/controls/hstab1_elv1def");
    rudderRef_ = XPLMFindDataRef("sim/flightmodel/controls/vstab1_rud1def");
    throttleRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro_use");
    rpmRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_tacrad");

    joystickOverrideRef_[0] = XPLMFindDataRef("sim/operation/override/override_joystick");
    joystickOverrideRef_[1] = XPLMFindDataRef("sim/operation/override/override_throttles");

    joystickAttitudeRef_[0] = XPLMFindDataRef("sim/joystick/yoke_roll_ratio");
    joystickAttitudeRef_[1] = XPLMFindDataRef("sim/joystick/yoke_pitch_ratio");
    joystickAttitudeRef_[2] = XPLMFindDataRef("sim/joystick/yoke_heading_ratio");

    course_ = XPLMFindDataRef("sim/flightmodel/position/hpath");
    temp_ = XPLMFindDataRef("sim/weather/temperature_le_c");
    pressure_ = XPLMFindDataRef("sim/weather/barometer_current_inhg");

    fuel_ = XPLMFindDataRef("sim/flightmodel/weight/m_fuel");

    simSpeed_ = XPLMFindDataRef("sim/time/sim_speed_actual");
    timestamp_ = XPLMFindDataRef("sim/time/total_flight_time_sec");

    sensorData_.hasGPSFix = true;
    sensorData_.autopilotActive = false;
}

bool
XPlaneInterface::run(RunStage stage)
{
    switch (stage)
    {
    case RunStage::INIT:
        {
            if (!checkIsSetAll())
            {
                CPSLOG_ERROR << "Missing Dependencies.";
                return true;
            }
            break;
        }
    case RunStage::NORMAL:
        {
            sensorDataEvent_ = get<IScheduler>()->schedule([this]
                                                           {
                                                               processData();
                                                           }, Milliseconds(0),
                                                           Milliseconds(1000 / (sensorFrequency_ * 10)));

            if (auto autopilotAPI = get<IAutopilotAPI>())
            {
                autopilotAPI->subscribeOnControllerOut([this](const ControllerOutput& out)
                {
                    actuate(out);
                });
            }
            break;
        }
    default:
        break;
    }

    return false;
}

void
XPlaneInterface::setAutopilotActive(bool active)
{
    sensorData_.autopilotActive = active;
    XPLMSetDatai(joystickOverrideRef_[0], active);
    XPLMSetDatai(joystickOverrideRef_[1], active);
}

void
XPlaneInterface::processData()
{
    auto timestamp = static_cast<double>(XPLMGetDataf(timestamp_));
    auto currTime = std::chrono::time_point_cast<Clock::duration>(
        TimePoint() + Nanoseconds(static_cast<uint64_t>(timestamp * 1e9)));

    int64_t step = std::chrono::duration_cast<Nanoseconds>(lastSensorUpdate_.time_since_epoch()).count() *
        sensorFrequency_ / int(1e9);
    int64_t currentStep = std::chrono::duration_cast<Nanoseconds>(currTime.time_since_epoch()).count() *
        sensorFrequency_ / int(1e9);
    if (currentStep <= step)
        return;

    lastSensorUpdate_ = currTime;

    if (auto api = get<IAutopilotAPI>())
    {
        //Process SensorData
        double lat = XPLMGetDatad(positionRefs_[0]);
        double lon = XPLMGetDatad(positionRefs_[1]);

        double north;
        double east;
        int zone;
        char hemi;

        latLongToUTM(lat, lon, north, east, zone, hemi);

        sensorData_.position[0] = east;
        sensorData_.position[1] = north;
        sensorData_.position[2] = XPLMGetDatad(positionRefs_[2]);

        // Don't know what frame X-Plane is in, but the following converts it to ENU inertial frame
        sensorData_.velocity[0] = static_cast<FloatingType>(XPLMGetDataf(velocityRefs_[0]));
        sensorData_.velocity[1] = -static_cast<FloatingType>(XPLMGetDataf(velocityRefs_[2]));
        sensorData_.velocity[2] = static_cast<FloatingType>(XPLMGetDataf(velocityRefs_[1]));

        sensorData_.groundSpeed = sensorData_.velocity.norm();
        sensorData_.airSpeed = static_cast<FloatingType>(XPLMGetDataf(airSpeedRef_));

        Vector3 accelerationInertial;
        // Don't know what frame X-Plane is in, but the following converts it to ENU inertial frame
        accelerationInertial[0] = static_cast<FloatingType>(XPLMGetDataf(accelerationRefs_[0]));
        accelerationInertial[1] = -static_cast<FloatingType>(XPLMGetDataf(accelerationRefs_[2]));
        accelerationInertial[2] = static_cast<FloatingType>(XPLMGetDataf(accelerationRefs_[1]));

        Matrix3 m;
        m = AngleAxis(-sensorData_.attitude[0], Vector3::UnitX())
            * AngleAxis(-sensorData_.attitude[1], Vector3::UnitY())
            * AngleAxis(-sensorData_.attitude[2], Vector3::UnitZ());

        sensorData_.acceleration = m * accelerationInertial;


        sensorData_.attitude[0] = degToRad(static_cast<FloatingType>(XPLMGetDataf(attitudeRefs_[0])));
        sensorData_.attitude[1] = degToRad(static_cast<FloatingType>(XPLMGetDataf(attitudeRefs_[1])));

        FloatingType yaw = degToRad(static_cast<FloatingType>(XPLMGetDataf(attitudeRefs_[2])));
        sensorData_.attitude[2] = boundAngleRad(-(yaw - M_PI_2));

        sensorData_.angleOfAttack = -degToRad(static_cast<FloatingType>(XPLMGetDataf(angleOfAttackRef_)));
        sensorData_.angleOfSideslip = degToRad(static_cast<FloatingType>(XPLMGetDataf(angleOfSideslipRef_)));

        sensorData_.angularRate[0] = degToRad(static_cast<FloatingType>(XPLMGetDataf(angularRateRefs_[0])));
        sensorData_.angularRate[1] = degToRad(-static_cast<FloatingType>(XPLMGetDataf(angularRateRefs_[1])));
        sensorData_.angularRate[2] = degToRad(static_cast<FloatingType>(XPLMGetDataf(angularRateRefs_[2])));

        sensorData_.hasGPSFix = static_cast<bool>(XPLMGetDatai(gpsFixRef_));

        FloatingType course = degToRad(static_cast<FloatingType>(XPLMGetDataf(course_)));
        sensorData_.courseAngle = boundAngleRad(-(course - M_PI_2));
        sensorData_.temperature = static_cast<FloatingType>(XPLMGetDataf(temp_));
        sensorData_.pressure = static_cast<FloatingType>(XPLMGetDataf(pressure_));

        sensorData_.sequenceNumber = sequenceNumber_++;
        if (sequenceNumber_ > std::numeric_limits<std::uint16_t>::max())
            sequenceNumber_ = 0;

        api->setSensorData(sensorData_);

        //Process Power Data
        powerData_.batteryCurrent = static_cast<FloatingType>(XPLMGetDataf(batteryCurrentRef_));
        powerData_.batteryVoltage = static_cast<FloatingType>(XPLMGetDataf(batteryVoltageRef_));

        api->setPowerData(powerData_);

        //Process Servo Data
        servoData_.aileron = static_cast<FloatingType>(XPLMGetDataf(aileronRef_));
        servoData_.elevator = static_cast<FloatingType>(XPLMGetDataf(elevatorRef_));
        servoData_.rudder = static_cast<FloatingType>(XPLMGetDataf(rudderRef_));

        float throttle[8];
        XPLMGetDatavf(throttleRef_, throttle, 0, 8);
        servoData_.throttle = static_cast<FloatingType>(throttle[0]);

        float rpm[8];
        XPLMGetDatavf(rpmRef_, rpm, 0, 8);
        rpm[0] = rpm[0] * 60 / M_PI / 2; // Radians Per Second to Revolution Per Minute
        servoData_.rpm = static_cast<FloatingType>(rpm[0]);

        //Add Timestamps
        sensorData_.timestamp = currTime;
        powerData_.timestamp = currTime;
        servoData_.timestamp = currTime;

        api->setServoData(servoData_);
    }
    else
    {
        CPSLOG_ERROR << "Missing IAutopilotAPI";
    }
    XPLMSetDataf(fuel_, 10.0);
}

void
XPlaneInterface::actuate(const ControllerOutput& out)
{
    CPSLOG_TRACE << "Begin Actuate\n";
    if (!sensorData_.autopilotActive)
        return;

    float throt = (static_cast<float>(out.throttleOutput) + 1) / 2;
    float throttle[] =
        {throt, throt, throt, throt, throt, throt, throt, throt};
    XPLMSetDatavf(throttleRef_, throttle, 0, 8);
    XPLMSetDataf(joystickAttitudeRef_[0], out.rollOutput);
    XPLMSetDataf(joystickAttitudeRef_[1], out.pitchOutput);
    XPLMSetDataf(joystickAttitudeRef_[2], out.yawOutput);
    CPSLOG_TRACE << "End Actuate\n";
}

const SensorData&
XPlaneInterface::getSensorData() const
{
    return sensorData_;
}
