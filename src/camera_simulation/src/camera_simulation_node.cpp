#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <ros/ros.h>
#include <simulation_interface/sensor_data.h>
#include <autopilot_interface/detail/uavAPConversions.h>

void
callback(const simulation_interface::sensor_data& sd)
{
	SensorData sensorData = rosToAp(sd);
	APLOG_DEBUG << "Sensor Data Timestamp ROS: " << to_simple_string(sd.header.stamp.toBoost());
	APLOG_DEBUG << "Sensor Data Position E uavAP: " << sensorData.position.x();
}

int
main(int argc, char** argv)
{
	APLogger::instance()->setLogLevel(LogLevel::DEBUG);
	APLogger::instance()->setModuleName("CameraSimulation");
	ros::init(argc, argv, "camera_simulation");

	ros::NodeHandle nh;

	std::string config;
	nh.getParam("/camera_simulation_node/config_path", config);

	ros::Subscriber sensorDataSubscriptionRos = nh.subscribe("radio_comm/sensor_data", 20,
			callback);

	ros::Rate loopRate(1000);
	while (ros::ok())
	{
		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}
