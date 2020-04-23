#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <ros/ros.h>
#include <simulation_interface/sensor_data.h>
#include <autopilot_interface/detail/uavAPConversions.h>
#include "image_cropper.hpp"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static string base_img_path;
static string output_path;
static double convert_ratio;
static double udp_radius;

void
callback(const simulation_interface::sensor_data& sd)
{
	SensorData sensorData = rosToAp(sd);
	APLOG_DEBUG << "Sensor Data Timestamp ROS: " << to_simple_string(sd.header.stamp.toBoost());
	APLOG_DEBUG << "Sensor Data Position X uavAP: " << sensorData.position.x();
	APLOG_DEBUG << "Sensor Data Position Y uavAP: " << sensorData.position.y();
	APLOG_DEBUG << "Sensor Data Position Z uavAP: " << sensorData.position.z();

	cropper(base_img_path, sensorData.position.x(), sensorData.position.y(), udp_radius, convert_ratio);
}

void init(ros::NodeHandle nh)
{
	nh.getParam("/camera_simulation_node/base_img_path", base_img_path);
	nh.getParam("/camera_simulation_node/output_path", output_path);
	nh.getParam("/camera_simulation_node/convert_ratio", convert_ratio);
	nh.getParam("/camera_simulation_node/udp_radius", udp_radius);
	nh.getParam("/camera_simulation_node/udp_origin_x", udp_origin_x);
	nh.getParam("/camera_simulation_node/udp_origin_y", udp_origin_y);

	APLOG_DEBUG << "image_location: " << base_img_path << " save_location: " << output_path << " ratio: " << convert_ratio;
	APLOG_DEBUG << base_img_path << output_path << convert_ratio << udp_radius << udp_origin_x << udp_origin_y;

	output_video = VideoWriter("/home/pure/devel/uavEE/build/test_results/camera_results/output.avi", CV_FOURCC('M','J','P','G'), 30, 
	Size(udp_radius * convert_ratio * 2, udp_radius * convert_ratio * 2));
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

	init(nh);

	ros::Subscriber sensorDataSubscriptionRos = nh.subscribe("radio_comm/sensor_data", 20,
			callback);

	ros::Rate loopRate(1000);
	while (ros::ok())
	{
		ros::spinOnce();
		loopRate.sleep();
	}

	output_video.release();

	return 0;
}
