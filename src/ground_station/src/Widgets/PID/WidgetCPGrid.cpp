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
#include "ground_station/IWidgetInterface.h"
#include "ground_station/Widgets/PID/WidgetCPGrid.h"
#include "ui_WidgetCPGrid.h"
#include "ground_station/Widgets/PID/PIDConfigPlot.h"
#include "ground_station/IDataSignals.h"
#include <QJsonDocument>
#include <QFileDialog>
#include <uavAP/Core/Logging/APLogger.h>
#include <QDebug>

WidgetCPGrid::WidgetCPGrid(QWidget* parent) :
		QWidget(parent), ui(new Ui::WidgetCPGrid)
{
	ui->setupUi(this);
}

void
WidgetCPGrid::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	if (!interface)
	{
		APLOG_ERROR << "WidgetCPGrid Cannot connect to interface";
		return;
	}
	if (auto ds = interface->getIDataSignals().get())
		QObject::connect(ds.get(), SIGNAL(onPIDStati(const radio_comm::pidstati&)), this,
				SLOT(onPIDStati(const radio_comm::pidstati&)));
	else
		APLOG_ERROR << "Cannot connect WidgetCPGrid to PIDStati. IDataSignals missing.";

	if (auto cm = interface->getPIDConfigurator().get())
	{
		int count = 0;
		for (auto& it : cm->getPIDMap())
		{
			auto cp = std::make_shared<PIDConfigPlot>(this, it.first, it.second.name,
					it.second.params);
			ui->gridLayout->addWidget(cp.get(), 0, count);
			cp->connect(cm);
			plots.insert(std::make_pair(it.first, cp));
			++count;
		}
	}
	else
		APLOG_ERROR << "Cannot connect WidgetCPGrid to PIDMap. ConfigManager missing.";
}

void
WidgetCPGrid::onPIDStati(const radio_comm::pidstati& stati)
{
	for (auto& it : stati.stati)
	{
		auto plot = plots.find(it.id);
		if (plot == plots.end())
		{
			APLOG_WARN << "PID status id " << it.id << " does not match any plot";
			continue;
		}
		plot->second->setData(it.value, it.target);
	}
	this->update();
}

void
WidgetCPGrid::on_saveGains_clicked()
{
	QJsonObject config;
	for (auto& it : plots)
	{
		QJsonObject gains;
		gains["kp"] = it.second->getkP();
		gains["ki"] = it.second->getkI();
		gains["kd"] = it.second->getkD();
		gains["ff"] = it.second->getFF();
		gains["imax"] = it.second->getIMax();
		config[it.second->title] = gains;
	}
	//QJsonObject config = cm_->getFlightConfig();
	QJsonDocument d(config);
	QFileDialog dialog;
	dialog.setWindowTitle("Save json configs");
	dialog.setFileMode(QFileDialog::AnyFile);
	dialog.setAcceptMode(QFileDialog::AcceptSave);
	QString filename;
	if (dialog.exec())
	{
		filename = dialog.selectedFiles().front();
	}
	else
	{
		APLOG_TRACE << "Cancelled json save.";
		return;
	}
	QFile saveFile(filename);
	if (!saveFile.open(QIODevice::WriteOnly))
	{
		APLOG_ERROR << "Could not open " << filename.toStdString() << " for saving.";
		return;
	}
	saveFile.write(d.toJson());
	saveFile.close();
}

void
WidgetCPGrid::on_loadGains_clicked()
{
	QFileDialog dialog;
	dialog.setWindowTitle("Load json configs");
	dialog.setFileMode(QFileDialog::ExistingFile);
	dialog.setAcceptMode(QFileDialog::AcceptOpen);
	QString filename;
	if (dialog.exec())
	{
		filename = dialog.selectedFiles().front();
	}
	else
	{
		APLOG_TRACE << "Cancelled json save.";
		return;
	}
	QFile loadFile(filename);
	if (!loadFile.open(QIODevice::ReadOnly))
	{
		APLOG_ERROR << "Could not open " << filename.toStdString() << " for loading.";
		return;
	}
	QByteArray confData = loadFile.readAll();
	loadFile.close();
	QJsonDocument loadDoc(QJsonDocument::fromJson(confData));
	QJsonObject loadData = loadDoc.object();
	for (QString s : loadData.keys())
	{
		for (auto& it : plots)
		{
			if (it.second->title == s)
			{
				it.second->setkP(loadData[s].toObject()["kp"].toDouble());
				it.second->setkI(loadData[s].toObject()["ki"].toDouble());
				it.second->setkD(loadData[s].toObject()["kd"].toDouble());
				it.second->setFF(loadData[s].toObject()["ff"].toDouble());
				it.second->setIMax(loadData[s].toObject()["imax"].toDouble());
			}
		}
	}
}

void
WidgetCPGrid::on_sendAllParams_clicked()
{
	for (auto& it : plots)
	{
		it.second->sendData();
	}
}

WidgetCPGrid::~WidgetCPGrid()
{
	delete ui;
}
