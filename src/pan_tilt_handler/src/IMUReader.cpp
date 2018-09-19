////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/*IMUReader::IMUReader(const std::string &portname)
 {
 const XsString name(portname.c_str());
 port_ = XsPortInfo(name, XBR_115k2);
 }*/

std::shared_ptr<IMUReader>
IMUReader::create(const boost::property_tree::ptree& config)
{
	auto pth = std::make_shared<IMUReader>();
	pth->configure(config);
	return pth;
}

bool
IMUReader::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper propertyMapper(config);
	if (!propertyMapper.add("imu_path", portName_, true))
		APLOG_ERROR << "Cannot find IMU path.";
	return propertyMapper.map();
}

IMUReader::~IMUReader()
{
	APLOG_TRACE << "closing device";
	device_.close();
}

void
IMUReader::notifyAggregationOnUpdate(const Aggregator& agg)
{
	scheduler_.setFromAggregationIfNotSet(agg);
	panTiltHander_.setFromAggregationIfNotSet(agg);
	//configManager_.setFromAggregationIfNotSet(agg);
}

bool
IMUReader::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "IMUReader: Scheduler missing";
			return true;
		}
		if (!panTiltHander_.isSet())
		{
			APLOG_ERROR << "IMUReader: PanTiltHander missing";
			return true;
		}
		/*if(!configManager_.isSet()){
		 APLOG_ERROR<<"IMUReader: ConfigManager missing";
		 return true;
		 }*/

		break;
	}
	case RunStage::NORMAL:
	{
		idlemode_ = false;
		//std::string imupath = configManager_.get()->getAntennaConfig().get_child("imu_path").get_value<std::string>();
		XsPortInfo port(portName_, XBR_115k2);
		if (!device_.openPort(port))
		{
			APLOG_ERROR << "Could not open port " << port.portName().toStdString()
					<< ", IMUReader going into idle mode";
			idlemode_ = true;
			break;
		}

		if (!device_.gotoConfig())
		{
			APLOG_ERROR << "Device at " << port.portName().toStdString()
					<< " could not go into config mode, IMUReader going into idle mode";
			idlemode_ = true;
			break;
		}
		XsOutputMode outputMode = XOM_Calibrated | XOM_Orientation | XOM_GpsPvt_Pressure; // output orientation data
		XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion; // For some reason IMU outputs quaternian angles as euler

		// set the device configuration
		if (!device_.setDeviceMode(outputMode, outputSettings))
		{
			APLOG_ERROR << "Could not configure MT device, IMUReader going into idle mode";
			idlemode_ = true;
			break;
		}

		scheduler_.get()->schedule(std::bind(&IMUReader::readIMUData, this), Milliseconds(0),
				Milliseconds(10));
		break;
	}
	case RunStage::FINAL:
	{
		if (!idlemode_ && !device_.gotoMeasurement())
		{
			APLOG_ERROR << "Could not go into measurement mode, IMUReader going into idle mode";
			idlemode_ = true;
		}
		break;
	}
	default:
		break;
	}
	return false;
}

bool
IMUReader::isIdle() const
{
	return idlemode_;
}

void
IMUReader::readIMUData()
{

	XsByteArray data;
	XsMessageArray msgs;
	device_.readDataToBuffer(data);
	device_.processBufferedData(data, msgs);

	LegacyDataPacket lpacket(1, false);
	XsDataPacket packet;
	if (!msgs.size())
		return;
	lpacket.setMessage(msgs.last());
	lpacket.setDataFormat(XOM_Calibrated | XOM_Orientation | XOM_GpsPvt_Pressure,
			XOS_OrientationMode_Quaternion, 0);
	XsDataPacket_assignFromLegacyDataPacket(&packet, &lpacket, 0);

	XsQuaternion quaternion = packet.orientationQuaternion();
	auto panTilt = panTiltHander_.get();
	if (!panTilt)
	{
		APLOG_ERROR << "Mimimimi, no pan tilt handler... mimimi";
		return;
	}
	std::lock_guard<std::mutex> lock(panTilt->panTiltIMUDataMutex_);
	panTilt->panTiltIMUData_.imuData.attitude = Vector3(quaternion.w() * M_PI / 180,
			quaternion.x() * M_PI / 180, quaternion.y() * M_PI / 180);
	XsVector gyrData = packet.calibratedGyroscopeData();
	panTilt->panTiltIMUData_.imuData.attitudeRate = Vector3(gyrData[0], gyrData[1], gyrData[2]);

	//time_t time = (time_t)packet.timeOfArrival().msTime();
	panTilt->panTiltIMUData_.imuData.timepoint = boost::posix_time::microsec_clock::local_time();

	XsGpsPvtData gps_data = packet.gpsPvtData();

	if (!panTilt->overrideGPS_)
	{
		//all coordinates being converted to ENU
		double utmE, utmN;
		int zone;
		LLtoUTM(eWGS84, (double) gps_data.m_latitude / 1E7, (double) gps_data.m_longitude / 1E7,
				utmN, utmE, zone);
		panTilt->panTiltIMUData_.gpsData.position = Vector3(utmE, utmN,
				-(double) gps_data.m_height / 1E3);
		panTilt->panTiltIMUData_.gpsData.velocity = Vector3((double) gps_data.m_vele / 100,
				(double) gps_data.m_veln / 100, -(double) gps_data.m_veld / 100);
		XsVector accl = packet.calibratedAcceleration();
		panTilt->panTiltIMUData_.imuData.acceleration = Vector3(accl[1], accl[0], -accl[2]);
	}
	else
	{
		panTilt->panTiltIMUData_.gpsData.velocity = Vector3();
		panTilt->panTiltIMUData_.imuData.acceleration = Vector3();
	}

	msgs.clear();
	//panTilt->updateSensorData();//TODO better design
}
