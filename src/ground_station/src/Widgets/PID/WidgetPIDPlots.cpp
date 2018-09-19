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
#include "ground_station/Widgets/PID/PIDCustomPlot.h"
#include "ground_station/Widgets/PID/WidgetPIDPlots.h"
#include "ui_WidgetPIDPlots.h"
#include "ground_station/IWidgetInterface.h"
#include "ground_station/IDataSignals.h"
#include "ground_station/ConfigManager.h"
#include <uavAP/Core/Logging/APLogger.h>

WidgetPIDPlots::WidgetPIDPlots(QWidget* parent) :
		QWidget(parent), ui(new Ui::WidgetPIDPlots)
{
	ui->setupUi(this);
	ui->columnsOnly->setChecked(1);
	this->update();
}

WidgetPIDPlots::~WidgetPIDPlots()
{
	delete ui;
}

void
WidgetPIDPlots::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	if (!interface)
	{
		APLOG_ERROR << "WidgetPIDPlots received null interface";
	}
	if (auto ds = interface->getIDataSignals().get())
		QObject::connect(ds.get(), SIGNAL(onPIDStati(const radio_comm::pidstati&)), this,
				SLOT(onPIDStati(const radio_comm::pidstati&)));
	else
		APLOG_ERROR << "Cannot connect WidgetPIDPlots to PIDStati. IDataSignals missing.";
	if (auto cm = interface->getPIDConfigurator().get())
	{
		int count = 0;
		for (auto& it : cm->getPIDMap())
		{
			//auto cp = std::make_shared<QCustomPlot>(this,it.second.name);
			auto cp = std::make_shared<PIDCustomPlot>(this, it.second.name);
			auto* cppt = cp.get();
			ui->PIDGrid->addWidget(cppt, 0, count);
			plots.insert(std::make_pair(it.first, cp));
			++count;
		}
	}
	else
	{
		APLOG_ERROR << "Cannot get PID Map in WidgetPIDPlots. ConfigManager missing.";
	}
}

void
WidgetPIDPlots::on_rowsOnly_pressed()
{
	clearGrid();
	int count = 0;
	for (auto& it : plots)
	{
		ui->PIDGrid->addWidget(it.second.get(), count, 0);
		it.second.get()->setVisible(true);
		++count;
	}
	this->update();
}

void
WidgetPIDPlots::on_columnsOnly_pressed()
{
	clearGrid();
	int count = 0;
	for (auto& it : plots)
	{
		ui->PIDGrid->addWidget(it.second.get(), 0, count);
		it.second.get()->setVisible(true);
		++count;
	}
	this->update();
}

void
WidgetPIDPlots::on_custom_pressed()
{
	clearGrid();
	int row = 0, col = 0;
	for (auto& it : plots)
	{
		ui->PIDGrid->addWidget(it.second.get(), row, col);
		it.second.get()->setVisible(true);
		++col;
		if (col == ui->numCols->text().toInt())
		{
			col = 0;
			++row;
		}
	}
	this->update();
}

void
WidgetPIDPlots::on_numCols_valueChanged(int)
{
	ui->custom->pressed();
	ui->custom->setChecked(1);
}

void
WidgetPIDPlots::onPIDStati(const radio_comm::pidstati& stati)
{
	for (auto& it : stati.stati)
	{
		auto plot = plots.find(it.id);
		if (plot == plots.end())
		{
			APLOG_WARN << "PID status id " << it.id << " does not match any plot";
			continue;
		}
		plot->second->addData(it.value, it.target);
	}
	this->update();
}

void
WidgetPIDPlots::clearGrid()
{
	for (auto& it : plots)
	{
		ui->PIDGrid->removeWidget(it.second.get());
		it.second.get()->setVisible(false);
	}
}
