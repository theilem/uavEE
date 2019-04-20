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
 * ground_station_node.cpp
 *
 *  Created on: Jan 6, 2018
 *      Author: mircot
 */

#include <iostream>
#include <string>
#include <functional>
#include <QTextStream>
#include <QString>
#include <QtWidgets/QApplication>
#include <QtCore/QMetaType>
#include <QtWidgets/QFileDialog>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/Frames/VehicleOneFrame.h>
#include <uavAP/Core/Time.h>
#include <uavAP/Core/Scheduler/IScheduler.h>
#include <uavAP/FlightAnalysis/StateAnalysis/Metrics.h>
#include <uavAP/MissionControl/GlobalPlanner/Trajectory.h>
#include <uavAP/MissionControl/MissionPlanner/Mission.h>
#include <uavAP/MissionControl/ManeuverPlanner/Override.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>
#include <simulation_interface/sensor_data.h>
#include <simulation_interface/actuation.h>
#include <radio_comm/pidstati.h>
#include <ros/ros.h>

#include "ground_station/GroundStationHelper.h"

Q_DECLARE_METATYPE(simulation_interface::sensor_data)
Q_DECLARE_METATYPE(simulation_interface::actuation)
Q_DECLARE_METATYPE(radio_comm::pidstati)
Q_DECLARE_METATYPE(VehicleOneFrame)
Q_DECLARE_METATYPE(SteadyStateMetrics)
Q_DECLARE_METATYPE(Override)
Q_DECLARE_METATYPE(Mission)
Q_DECLARE_METATYPE(Trajectory)
Q_DECLARE_METATYPE(ControllerOutput)

int
main(int argc, char** argv)
{
	APLogger::instance()->setLogLevel(LogLevel::DEBUG);
	APLogger::instance()->setModuleName("GroundStation");
	ros::init(argc, argv, "ground_station_node");
	ros::NodeHandle node;
	std::string resourcePath, confPath;
	//requires Qt>=5.6, which not all lab machines have
	//QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
	QApplication app(argc, argv);
	if (!node.getParam("/ground_station_node/config_path", confPath))
	{
		QFileDialog dialog;
		dialog.setWindowTitle("Open Configuration");
		dialog.setFileMode(QFileDialog::ExistingFile);
		dialog.setAcceptMode(QFileDialog::AcceptOpen);
		if (dialog.exec())
		{
			confPath = dialog.selectedFiles().front().toStdString();
		}
		else
		{
			APLOG_TRACE << "Cancelled config loading.";
			return 0;
		}
	}

	if (!node.getParam("/ground_station_node/resource_path", resourcePath))
	{
		QFileDialog dialog;
		dialog.setWindowTitle("Open Resource Folder");
		dialog.setFileMode(QFileDialog::Directory);
		dialog.setAcceptMode(QFileDialog::AcceptOpen);
		if (dialog.exec())
			resourcePath = dialog.selectedFiles().front().toStdString() + "/";
	}

	QFile f(QString::fromStdString(resourcePath + "qdarkstyle/style.qss"));
	if (f.exists())
	{
		f.open(QFile::ReadOnly | QFile::Text);
		QTextStream ts(&f);
		app.setStyleSheet(ts.readAll());
		APLOG_TRACE << "Loaded custom stylesheet.";
	}
	else
	{
		APLOG_WARN << "Could not open resource file at " << f.symLinkTarget().toStdString();
	}

	boost::property_tree::ptree config;
	boost::property_tree::ptree configManagerConfig;
	configManagerConfig.add("ground_station_config_path", confPath);
	configManagerConfig.add("ground_station_resource_path", resourcePath);
	config.add_child("config_manager", configManagerConfig);

	qRegisterMetaType<simulation_interface::sensor_data>();
	qRegisterMetaType<simulation_interface::actuation>();
	qRegisterMetaType<radio_comm::pidstati>();
	qRegisterMetaType<VehicleOneFrame>();
	qRegisterMetaType<SteadyStateMetrics>();
	qRegisterMetaType<Override>();
	qRegisterMetaType<Mission>();
	qRegisterMetaType<Trajectory>();
	qRegisterMetaType<ControllerOutput>();

	GroundStationHelper helper;

	Aggregator aggregator = helper.createAggregation(config);
	SimpleRunner run(aggregator);

	auto sched = aggregator.getOne<IScheduler>();
	sched->schedule(ros::spinOnce, Milliseconds(0), Milliseconds(50));

	if (run.runAllStages())
	{
		APLOG_ERROR << "Run all stages failed.";
		return 1;
	}

	app.exec();

	ros::Rate loopRate(1000);
	while (ros::ok())
	{
		ros::spinOnce();
		loopRate.sleep();
	}
	return 0;
}
