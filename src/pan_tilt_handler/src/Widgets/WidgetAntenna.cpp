////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavEE.
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
﻿#include "pan_tilt_handler/Widgets/WidgetAntenna.h"
#include "pan_tilt_handler/Widgets/WidgetAntennaDataManipulator.h"
#include "ui_WidgetAntenna.h"
#include <uavAP/Core/Logging/APLogger.h>

#include <ground_station/Widgets/WidgetSensorData.h>
#include <ground_station/Widgets/PID/WidgetCPGrid.h>

WidgetAntenna::WidgetAntenna(QWidget* parent) :
		QWidget(parent), ui(new Ui::WidgetAntenna)
{
	ui->setupUi(this);
}

void
WidgetAntenna::connect(std::shared_ptr<PanTiltHandler> interface)
{
	if (interface->isIdle())
	{
		APLOG_WARN << "PanTilt is idle, cannot create WidgetAntenna";
		return;
	}
	auto gsInterface = interface->getInterface();
	ui->cpGrid_Layout->addWidget(
			dynamic_cast<WidgetCPGrid*>(WidgetCPGrid::createGSWidget(gsInterface, this)));
	WidgetAntennaDataManipulator* dataManipulator = new WidgetAntennaDataManipulator(this);
	dataManipulator->connect(interface);
	ui->verticalLayout->addWidget(dataManipulator);
	ui->verticalLayout->addWidget(
			dynamic_cast<WidgetSensorData*>(WidgetSensorData::createGSWidget(gsInterface, this)));
	//ui->cpgrid = dynamic_cast<WidgetCPGrid*>(WidgetCPGrid::createGSWidget(gsInterface, this));
	//ui->datamanipulator->connect(interface);
	//ui->sensordata = dynamic_cast<WidgetSensorData*>(WidgetSensorData::createGSWidget(gsInterface, this));
}

WidgetAntenna::~WidgetAntenna()
{
	delete ui;
}
