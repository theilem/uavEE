////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavEE.
//
// uavEE is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavEE is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/*
 * AutopilotInterface.cpp
 *
 *  Created on: Nov 25, 2017
 *      Author: mircot
 */
#include "autopilot_interface/uavAPConversions.h"
#include "autopilot_interface/UTMToLatLong.h"
#include <boost/date_time/posix_time/conversion.hpp>
#include <simulation_interface/actuation.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "autopilot_interface/AutopilotInterface.h"

#include <uavAP/API/ap_ext/ap_ext.h>
#include <uavAP/Core/DataPresentation/Content.h>
#include <uavAP/Core/DataPresentation/APDataPresentation/APDataPresentation.h>
#include <uavAP/Core/IDC/Serial/SerialIDC.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include <uavAP/Core/Runner/SimpleRunner.h>
#include <uavAP/Core/Scheduler/MultiThreadingScheduler.h>
#include <uavAP/Core/Time.h>
#include <uavAP/Core/TimeProvider/SystemTimeProvider.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>
#include <uavAP/API/ap_ext/ApExtManager.h>

AutopilotInterface::AutopilotInterface() :
    setup_(false), deviceBridge_(false), numChannels_(7), lastSequenceNr_(0)
{
    ros::NodeHandle nh;

    actuationPublisherRos_ = nh.advertise<simulation_interface::actuation>("actuation", 20);
    sensorDataSubscriptionRos_ = nh.subscribe("sensor_data", 20, &AutopilotInterface::onSensorData,
                                 this);
    powerModelSubscriptionRos_ = nh.subscribe("power_model/thrust_power", 20, &AutopilotInterface::onThrustPower, this);
    aggregator_.add(std::make_shared<MultiThreadingScheduler>());
    aggregator_.add(std::make_shared<SystemTimeProvider>());
}

AutopilotInterface::~AutopilotInterface()
{
    if (!deviceBridge_)
        ap_ext_teardown();
}

bool
AutopilotInterface::configure(const boost::property_tree::ptree& config)
{
    boost::property_tree::ptree interfaceConfig;
    PropertyMapper pm(config);
    if (!pm.add("interface", interfaceConfig, true))
        return false;

    return channelMixing_.configure(interfaceConfig) && servoMapping_.configure(interfaceConfig);
}

bool
AutopilotInterface::startApExt()
{
    if (deviceBridge_)
    {
        return false;
    }
    else
    {
        dataSample_.imu_sample = new imu_sample_t;
        dataSample_.pic_sample = new pic_sample_t;

        dataSample_.pic_sample->adc_channels[20] = 3000; //Autopilot always active TODO: Maybe not?

        if (ap_ext_setup() != 0)
            return false;
        auto sched = aggregator_.getOne<IScheduler>();
        if (!sched)
            return false;
//		sched->schedule(std::bind(&AutopilotInterface::sendActuation,this), Milliseconds(0), Milliseconds(2));
        auto apMan = getApExtManager();
        apMan->notifyOnActuation(std::bind(&AutopilotInterface::sendActuation, this));
        SimpleRunner runner(aggregator_);
        if (runner.runAllStages())
            return false;
        setup_ = true;

    }
    return setup_;
}

void
AutopilotInterface::onSensorData(const simulation_interface::sensor_data& sensorData)
{
    if (!setup_)
    {
        APLOG_WARN << "Interface not setup. Cannot send sensor data to ap.";
        return;
    }

    if (deviceBridge_)
    {
        SensorData sd = rosToAp(sensorData);
        SensorDataLight sdLight = fromSensorData(sd);
        sdLight.sequenceNr = sensorData.sequenceNr;
        auto dp = aggregator_.getOne<IDataPresentation<Content, Target>>();
        if (!dp)
        {
            APLOG_ERROR << "DataPresentation missing. Cannot send sensordata.";
            return;
        }
        auto packet = dp->serialize(sdLight, Content::SENSOR_DATA_LIGHT);
        sensorDataSender_.sendPacket(packet);
        return;
    }

    imu_sample_t* imu = dataSample_.imu_sample;

    /* Rotation rate */
    imu->imu_rot_x = sensorData.velocity.angular.x;
    imu->imu_rot_y = sensorData.velocity.angular.y;
    imu->imu_rot_z = sensorData.velocity.angular.z;

    double roll = sensorData.attitude.x;
    double pitch = sensorData.attitude.y;
    double yaw = sensorData.attitude.z;

    //Add gravity to acceleration
    Vector3 gravityInertial(0, 0, 9.81);
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(-roll, Vector3::UnitX()) * Eigen::AngleAxisd(-pitch, Vector3::UnitY());
    Vector3 gravityBody = m * gravityInertial;

    imu->imu_accel_x = sensorData.acceleration.linear.x - gravityBody.x();
    imu->imu_accel_y = sensorData.acceleration.linear.y - gravityBody.y();
    imu->imu_accel_z = sensorData.acceleration.linear.z - gravityBody.z();

    double toDegree = 180. / M_PI;
    roll *= toDegree;
    pitch *= toDegree;
    yaw *= toDegree;

    imu->imu_euler_roll = roll;
    imu->imu_euler_pitch = pitch;
    imu->imu_euler_yaw = yaw;

    /* North, East Down coordinates - computed aside */
    /* Compute north, east, down coordinates */
    int zone;
    char hemi;

    UTMtoLL(22, sensorData.position.y, sensorData.position.x, 0, imu->imu_lat, imu->imu_lon);
    imu->imu_alt = sensorData.position.z;

    /* Velocity */
    imu->imu_vel_x = sensorData.velocity.linear.x;
    imu->imu_vel_y = sensorData.velocity.linear.y;
    imu->imu_vel_z = sensorData.velocity.linear.z;

    //Valid flag for GPS fix
    imu->valid_flags = 0x80;

    /* Timestamp */

    time_t secFromEpoch = sensorData.header.stamp.sec;

    auto time = boost::posix_time::to_tm(boost::posix_time::from_time_t(secFromEpoch));

    imu->imu_time_year = time.tm_year + 1900; //Year offset of ROS
    imu->imu_time_month = time.tm_mon + 1;
    imu->imu_time_day = time.tm_mday;
    imu->imu_time_hour = time.tm_hour;
    imu->imu_time_minute = time.tm_min;
    imu->imu_time_second = time.tm_sec;
    imu->imu_time_nano = sensorData.header.stamp.nsec;
    dataSample_.pic_sample->pwm_channels[21] = static_cast<unsigned long>(sensorData.sequenceNr);
    int sensResult = ap_ext_sense(&dataSample_);

    if (sensResult != 0)
    {
        APLOG_WARN << "Something went wrong sending the data sample.";
    }

}

void
AutopilotInterface::onThrustPower(const power_modeling::power_info&)
{

}

void
AutopilotInterface::sendActuation()
{
    unsigned long pwmSeq[numChannels_ + 1];

    int actResult = ap_ext_actuate(pwmSeq, numChannels_ + 1);

    uint32_t seq = static_cast<uint32_t>(pwmSeq[numChannels_]);

    //Do only write when there was no update in sensordata
    if (lastSequenceNr_ == seq)
        return;

    lastSequenceNr_ = seq;

    unsigned long pwm[numChannels_];

    std::copy(pwmSeq, pwmSeq + numChannels_, pwm);
//	int actResult = ap_ext_actuate(pwm, numChannels_);

    if (actResult != 0)
    {
        APLOG_WARN << "Something went wrong getting actuation data.";
        return;
    }

    simulation_interface::actuation act = reverseChannelMixing(pwm);

    act.sequenceNr = seq;

    actuationPublisherRos_.publish(act);
}

void
AutopilotInterface::setDeviceBridge(const std::string& serialPort)
{
    deviceBridge_ = true;
    serialPort_ = serialPort;

    auto idc = std::make_shared<SerialIDC>();

    aggregator_.add(idc);
    aggregator_.add(std::make_shared<APDataPresentation<Content, Target>>());

    SerialIDCParams params(serialPort_, 115200, "*-*\n");
    idc->subscribeOnPacket(params, std::bind(&AutopilotInterface::onPacket, this, std::placeholders::_1));
    sensorDataSender_ = idc->createSender(params);

    SimpleRunner runner(aggregator_);
    setup_ = !runner.runAllStages();
}

simulation_interface::actuation
AutopilotInterface::reverseChannelMixing(unsigned long* pwm)
{
    std::vector<unsigned long> pwmVec(pwm, pwm + numChannels_);

    auto channels = servoMapping_.unmap(pwmVec);

    ControllerOutput control = channelMixing_.unmixChannels(channels);

    simulation_interface::actuation actuation;

    actuation.rollOutput = control.rollOutput;
    actuation.pitchOutput = control.pitchOutput;
    actuation.yawOutput = control.yawOutput;
    actuation.flapOutput = control.flapOutput;
    actuation.collectiveOutput = control.collectiveOutput;
    actuation.throttleOutput = control.throttleOutput;

    return actuation;
}

void
AutopilotInterface::onPacket(const Packet& packet)
{
    auto dp = aggregator_.getOne<IDataPresentation<Content, Target>>();
    if (!dp)
    {
        APLOG_ERROR << "DataPresentation missing. Cannot handle packet.";
        return;
    }
    Content content;
    auto any = dp->deserialize(packet, content);

    ControllerOutput control;
    if (content == Content::CONTROLLER_OUTPUT)
    {
        control = boost::any_cast<ControllerOutput>(any);
    }
    else if (content == Content::CONTROLLER_OUTPUT_LIGHT)
    {
        control = fromControllerOutputLight(boost::any_cast<ControllerOutputLight>(any));
    }
    else
    {
        APLOG_ERROR << "Received invalid packet. Expected ControllerOutput(Light). Received: " << static_cast<int>(content);
        return;
    }
    simulation_interface::actuation actuation;

    actuation.rollOutput = control.rollOutput;
    actuation.pitchOutput = control.pitchOutput;
    actuation.yawOutput = control.yawOutput;
    actuation.flapOutput = control.flapOutput;
    actuation.collectiveOutput = control.collectiveOutput;
    actuation.throttleOutput = control.throttleOutput;
    actuation.sequenceNr = control.sequenceNr;

    actuationPublisherRos_.publish(actuation);
}
